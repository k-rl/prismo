import threading
from collections.abc import Callable, Iterator
from typing import Any

import dask.array as da
import dill
import multiprocess
import napari
import numpy as np
import xarray as xr
import zarr as zr
from multiprocess.connection import Connection
from napari import Viewer
from qtpy.QtCore import QTimer
from zarr.errors import ContainsGroupError

from .control import Control
from .widgets import BoundarySelector, PositionSelector, init_widgets


class Relay:
    def __init__(self, pipe: Connection, path: str = ""):
        self._path = path
        self._pipe = pipe

    def subpath(self, path: str) -> "Relay":
        return Relay(self._pipe, self._path + path + "/")

    def get(self, route: str, *args: Any, **kwargs: Any) -> Any:
        self._pipe.send([self._path + route, args, kwargs])
        return self._pipe.recv()

    def post(self, route: str, *args: Any, **kwargs: Any):
        self._pipe.send([self._path + route, args, kwargs])


class Session:
    def __init__(
        self,
        init_client: Callable[[Viewer, Relay], Any],
        file: str | None = None,
        tile: np.ndarray | None = None,
        attrs: dict[str, Any] | None = None,
    ):
        def run_client(pipe: Connection):
            viewer = napari.Viewer()
            relay = Relay(pipe)
            # Save the client in a variable so it doesn't get garbage collected.
            _client = init_client(viewer, relay)
            napari.run()
            pipe.close()

        def run_router(pipe: Connection, quit: threading.Event):
            while not quit.is_set():
                try:
                    route, args, kwargs = pipe.recv()
                    result = self._routes[route](*args, **kwargs)
                    if result is not None:
                        pipe.send(result)
                except (BrokenPipeError, EOFError):
                    self.quit()
                except Exception as e:
                    self.quit()
                    raise e

        self._running = threading.Event()
        self._running.set()
        self._quit = threading.Event()
        ctx = multiprocess.get_context("spawn")
        self._pipe, child_pipe = ctx.Pipe()
        self._client_process = ctx.Process(target=run_client, args=(child_pipe,))
        self._workers: list[threading.Thread] = []
        self._router = threading.Thread(target=run_router, args=(self._pipe, self._quit))
        self._routes: dict[str, Callable[..., Any]] = {}
        self._arrays: dict[str, xr.DataArray] = {}
        self._array_lock = threading.Lock()
        self._file = file
        self._tile = tile
        self._attrs = attrs if attrs is not None else {}

    def start(self):
        self._client_process.start()
        for worker in self._workers:
            worker.start()
        self._router.start()

    def resume(self):
        self._running.set()

    def pause(self):
        self._running.clear()

    def quit(self):
        self._running.set()
        self._quit.set()
        if self._client_process.is_alive():
            self._client_process.terminate()
        self._pipe.close()

    def worker[T: Callable[..., Iterator[Any]]](self, func: T) -> T:
        def run_worker():
            for _ in func():
                self._running.wait()
                if self._quit.is_set():
                    break

        self._workers.append(threading.Thread(target=run_worker))

        return func

    def route[T: Callable[..., Any]](
        self, name: str, func: T | None = None
    ) -> T | Callable[[T], T]:
        if func is None:
            # route got called as a decorator.
            def decorator(func: T) -> T:
                self._routes[name] = func
                return func

            return decorator
        else:
            # route got called as a standard method.
            self._routes[name] = func
            return func

    def array(self, name: str, **dims: int | list[Any]) -> xr.DataArray:
        shape = tuple(x if isinstance(x, int) else len(x) for x in dims.values())
        xp = xr.DataArray(
            data=da.zeros(
                shape=shape + self._tile.shape,
                chunks=(1,) * len(dims) + self._tile.shape,
                dtype=self._tile.dtype,
            ),
            dims=tuple(dims.keys()) + ("y", "x"),
            name=name,
        )

        for dim_name, coords in dims.items():
            if not isinstance(coords, int):
                xp.coords[dim_name] = coords

        store = zr.storage.LocalStore(self._file)
        compressor = zr.codecs.BloscCodec(
            cname="zstd", clevel=5, shuffle=zr.codecs.BloscShuffle.bitshuffle
        )

        for attr_name, attr in self._attrs.items():
            xp.attrs[attr_name] = attr

        try:
            xp.to_dataset(promote_attrs=True, name="tile").to_zarr(
                store, group=name, compute=False, encoding={"tile": {"compressors": compressor}}
            )
        except ContainsGroupError as e:
            raise FileExistsError(f"{self._file}/{name} already exists.") from e

        # Xarray/Dask don't natively support disk-writeable zarr arrays so we have to manually
        # load the zarr array and patch in a modified dask array that writes to disk when
        # __setitem__ is called.
        zarr_tiles = zr.open(store, path=f"{name}/tile", mode="a")
        tiles = da.from_zarr(zarr_tiles)
        tiles.__class__ = DiskArray
        tiles._zarr_array = zarr_tiles
        xp.data = tiles

        with self._array_lock:
            self._arrays[name] = xp

        return xp

    @property
    def arrays(self) -> dict[str, xr.DataArray]:
        with self._array_lock:
            arrs = dict(self._arrays)
        return arrs

    def __del__(self):
        self.quit()


class LiveView:
    def __init__(self, viewer: Viewer, relay: Relay, widgets: dict[str, Callable[[Relay], Any]]):
        self._viewer = viewer
        self._relay = relay
        img = self._relay.get("img")
        self._viewer.add_image(img, name="live")
        self._timer = QTimer()
        self._timer.timeout.connect(self.update_img)
        self._timer.start(1000 // 30)
        tabify = False
        for name, widget in widgets.items():
            self._viewer.window.add_dock_widget(
                widget(self._relay), name=name, tabify=tabify, area="left"
            )
            tabify = True

    def update_img(self):
        img = self._relay.get("img")
        self._viewer.layers[0].data = img


def live(ctrl: Control) -> Session:
    widgets, widget_routes = init_widgets(ctrl)
    session = Session(lambda v, r: LiveView(v, r, widgets=widgets))

    img = ctrl.snap()

    @session.worker
    def snap():
        while True:
            img[:] = ctrl.snap()
            yield

    session.route("img", lambda: img)
    for name, func in widget_routes.items():
        session.route(name, func)

    session.start()

    return session


class AcquisitionView:
    def __init__(
        self,
        viewer: Viewer,
        relay: Relay,
        file: str,
        widgets: dict[str, Callable[[Relay], Any]],
        tiled: bool = False,
        multi: bool = False,
    ):
        self._viewer = viewer
        self._relay = relay
        self._file = file
        self._live_timer = QTimer()
        self._refresh_timer = QTimer()
        self._arrays: set[str] = set()
        self._imgs: dict[str, xr.DataArray] = {}
        self._contrast_set: set[str] = set()

        if tiled or multi:
            img = self._relay.get("img")
            self._viewer.add_image(img, name="live")
            self._live_timer.timeout.connect(self.update_img)
            self._live_timer.start(1000 // 30)
            if tiled:
                self._viewer.window.add_dock_widget(
                    BoundarySelector(self._relay, self.start_acq),
                    name="Acquisition Boundaries",
                    tabify=False,
                )
            elif multi:
                self._viewer.window.add_dock_widget(
                    PositionSelector(self._relay, self.start_acq),
                    name="Acquisition Positions",
                    tabify=False,
                )
        else:
            self._refresh_timer.timeout.connect(self.refresh)
            self._refresh_timer.start(1000)

        tabify = False
        for name, widget in widgets.items():
            self._viewer.window.add_dock_widget(
                widget(self._relay), name=name, tabify=tabify, area="left"
            )
            tabify = True

    def start_acq(self, *args: Any):
        self._live_timer.disconnect()
        self._live_timer.stop()
        self._viewer.layers.remove("live")

        self._relay.post("start_acq", *args)
        self._refresh_timer.timeout.connect(self.refresh)
        self._refresh_timer.start(1000)

    def refresh(self):
        arrays = self._relay.get("arrays")
        new_arrays = arrays - self._arrays
        self._arrays = arrays.union(self._arrays)
        for arr in new_arrays:
            xp = xr.open_zarr(self._file, group=arr)
            xp = xp["tile"].assign_attrs(xp.attrs)
            img = tiles_to_image(xp)

            layer_names = arr
            if "channel" in img.dims:
                layer_names = [f"{arr}: {c}" for c in xp.coords["channel"].to_numpy()]

            viewer_dims = self._viewer.dims.axis_labels[:-2] + ("y", "x")
            img = img.expand_dims([d for d in viewer_dims if d not in img.dims])
            img = img.transpose("channel", ..., *viewer_dims, missing_dims="ignore")

            self._viewer.add_image(
                img,
                channel_axis=0 if "channel" in img.dims else None,
                name=layer_names,
                multiscale=False,
                cache=False,
            )
            new_dims = tuple(d for d in img.dims if d not in viewer_dims and d != "channel")
            self._viewer.dims.axis_labels = new_dims + viewer_dims
            # Make sure new dimension sliders get initialized to be 0.
            self._viewer.dims.current_step = (0,) * len(new_dims) + self._viewer.dims.current_step[
                -len(new_dims) :
            ]
            # Save this array so we can set the contrast limits once a nonzero element gets added.
            self._imgs[arr] = img

        """
        # Set contrast limits when a layer gets updated for the first time.
        for arr, img in self._imgs.items():
            if "channel" in img.dims:
                for c in img.coords["channel"].to_numpy():
                    layer_name = f"{arr}: {c}"
                    subimg = img.sel(channel=c)
                    if layer_name not in self._contrast_set and subimg.any():
                        self._viewer.layers[layer_name].contrast_limits = (
                                0, subimg.max().to_numpy())
                        self._contrast_set.add(layer_name)
            else:
                if arr not in self._contrast_set and img.any():
                    self._viewer.layers[arr].contrast_limits = (0, img.max().to_numpy())
                    self._contrast_set.add(arr)
        """

        # Update each of the image layers.
        for layer in self._viewer.layers:
            layer.refresh()

    def update_img(self):
        img = self._relay.get("img")
        self._viewer.layers[0].data = img


def run(run_func: Callable[..., Iterator[Any]]) -> "Runner":
    class Runner:
        def __init__(self):
            self._running = threading.Event()
            self._running.set()
            self._quit = threading.Event()

            def run_worker():
                for _ in run_func(self):
                    self._running.wait()
                    if self._quit.is_set():
                        break

            self._worker = threading.Thread(target=run_worker)
            self._worker.start()

        def resume(self):
            self._running.set()

        def pause(self):
            self._running.clear()

        def quit(self):
            self._running.set()
            self._quit.set()

        def join(self):
            self._worker.join()

    return Runner()


def acq(ctrl: Control, file: str, acq_func: Callable[[Session], Iterator[Any]]) -> Session:
    widgets, widget_routes = init_widgets(ctrl)
    session = Session(
        lambda v, r: AcquisitionView(v, r, file=file, widgets=widgets),
        file,
        ctrl.snap(),
        dict(acq_func=dill.source.getsource(acq_func)),
    )

    @session.worker
    def acq():
        store = zr.storage.LocalStore(file)
        for _ in acq_func(session):
            for name, xp in session.arrays.items():
                xp.to_dataset(promote_attrs=True, name="tile").to_zarr(
                    store, group=name, compute=False, mode="a"
                )
            yield

    session.route("arrays", lambda: set(session.arrays.keys()))
    for name, func in widget_routes.items():
        session.route(name, func)

    session.start()
    return session


def multi_acq(
    ctrl: Control,
    file: str,
    acq_func: Callable[[Session, list[tuple[float, float]]], Iterator[Any]],
    overlap: float = 0.0,
) -> Session:
    tile = ctrl.snap()
    pos: list[list[tuple[float, float]] | None] = [None]
    acq_event = threading.Event()

    widgets, widget_routes = init_widgets(ctrl)
    session = Session(
        lambda v, r: AcquisitionView(v, r, file=file, widgets=widgets, multi=True),
        file,
        ctrl.snap(),
        dict(overlap=overlap, acq_func=dill.source.getsource(acq_func)),
    )

    @session.worker
    def acq():
        while not acq_event.is_set():
            tile[:] = ctrl.snap()
            yield

        store = zr.storage.LocalStore(file)
        for _ in acq_func(session, pos[0]):
            for name, xp in session.arrays.items():
                xp.to_dataset(promote_attrs=True, name="tile").to_zarr(
                    store, group=name, compute=False, mode="a"
                )
            yield

    @session.route("start_acq")
    def start_acq(xys):
        pos[0] = xys
        acq_event.set()

    session.route("img", lambda: tile)
    session.route("xy", lambda: ctrl.xy)
    session.route("arrays", lambda: set(session.arrays.keys()))
    for name, func in widget_routes.items():
        session.route(name, func)

    session.start()
    return session


def tiled_acq(
    ctrl: Control,
    file: str,
    acq_func: Callable[[Session, np.ndarray, np.ndarray], Iterator[Any]],
    overlap: float,
    top_left: tuple[float, float] | None = None,
    bot_right: tuple[float, float] | None = None,
) -> Session:
    tile = ctrl.snap()
    pos: list[np.ndarray | None] = [None, None]
    get_pos = top_left is None or bot_right is None
    acq_event = threading.Event()

    widgets, widget_routes = init_widgets(ctrl)
    # TODO: Write additional attrs e.g. px_len.
    session = Session(
        lambda v, r: AcquisitionView(v, r, file=file, widgets=widgets, tiled=get_pos),
        file,
        ctrl.snap(),
        dict(overlap=overlap, acq_func=dill.source.getsource(acq_func)),
    )

    @session.worker
    def acq():
        if get_pos:
            while not acq_event.is_set():
                tile[:] = ctrl.snap()
                yield
            xs, ys = pos
        else:
            xs, ys = tile_coords(ctrl, top_left, bot_right, overlap)

        store = zr.storage.LocalStore(file)
        for _ in acq_func(session, xs, ys):
            for name, xp in session.arrays.items():
                xp.to_dataset(promote_attrs=True, name="tile").to_zarr(
                    store, group=name, compute=False, mode="a"
                )
            yield

    @session.route("start_acq")
    def start_acq(top_left, bot_right):
        xs, ys = tile_coords(ctrl, top_left, bot_right, overlap)
        pos[0] = xs
        pos[1] = ys
        acq_event.set()

    session.route("img", lambda: tile)
    session.route("xy", lambda: ctrl.xy)
    session.route("arrays", lambda: set(session.arrays.keys()))
    for name, func in widget_routes.items():
        session.route(name, func)

    session.start()
    return session


class DiskArray(da.core.Array):
    __slots__ = tuple()

    def __setitem__(self, key, value):
        self._zarr_array[key] = value


def tile_coords(
    ctrl: Control,
    top_left: tuple[float, float],
    bot_right: tuple[float, float],
    overlap: float,
) -> tuple[np.ndarray, np.ndarray]:
    tile = ctrl.snap()
    width = tile.shape[1]
    height = tile.shape[0]
    overlap_x = int(round(overlap * width))
    overlap_y = int(round(overlap * height))
    delta_x = (width - overlap_x) * ctrl.px_len
    delta_y = (height - overlap_y) * ctrl.px_len
    xs = np.arange(top_left[0], bot_right[0] + delta_x - 1, delta_x)
    ys = np.arange(top_left[1], bot_right[1] + delta_y - 1, delta_y)
    return xs, ys


def tiles_to_image(xp: xr.DataArray) -> xr.DataArray:
    if "overlap" not in xp.attrs:
        return xp.transpose(..., "y", "x")

    if xp.attrs["overlap"] != 0:
        overlap_y = int(round(xp.attrs["overlap"] * xp.shape[-2]))
        overlap_x = int(round(xp.attrs["overlap"] * xp.shape[-1]))
        img = xp[..., :-overlap_y, :-overlap_x]
    else:
        img = xp

    if "row" in img.dims:
        img = img.transpose("row", "y", ...)
        img = xr.concat(img, dim="y")
    if "col" in img.dims:
        img = img.transpose("col", "x", ...)
        img = xr.concat(img, dim="x")

    img = img.transpose(..., "y", "x")
    return img
