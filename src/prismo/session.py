import threading
import warnings
from collections.abc import Callable, Iterator
from typing import Any

import dask.array as da
import multiprocess
import napari
import numpy as np
import xarray as xr
import zarr as zr
from multiprocess.connection import Connection

warnings.filterwarnings("ignore", message=".*itertools.*", category=DeprecationWarning)
from napari import Viewer  # noqa: E402
from zarr.errors import ContainsGroupError


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


class _DiskArray(da.core.Array):
    __slots__ = tuple()

    def __setitem__(self, key, value):
        self._zarr_array[key] = value


class Session:
    def __init__(
        self,
        init_view: Callable[[Viewer, Relay], Any] | None = None,
        file: str | None = None,
        tile: np.ndarray | None = None,
        attrs: dict[str, Any] | None = None,
    ):
        self._running = threading.Event()
        self._running.set()
        self._quit = threading.Event()
        self._workers: list[threading.Thread] = []
        self._routes: dict[str, Callable[..., Any]] = {}
        self._arrays: dict[str, xr.DataArray] = {}
        self._array_lock = threading.Lock()
        self._file = file
        self._tile = tile
        self._attrs = attrs if attrs is not None else {}
        self._pipe = None
        self._view_process = None
        self._router = None

        if init_view is None:
            return

        def run_view(pipe: Connection):
            viewer = napari.Viewer()
            relay = Relay(pipe)
            # Save the view in a variable so it doesn't get garbage collected.
            _view = init_view(viewer, relay)
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

        ctx = multiprocess.get_context("spawn")
        self._pipe, child_pipe = ctx.Pipe()
        self._view_process = ctx.Process(target=run_view, args=(child_pipe,))
        self._router = threading.Thread(target=run_router, args=(self._pipe, self._quit))

    def start(self):
        if self._view_process is not None:
            self._view_process.start()
            self._router.start()
        for worker in self._workers:
            worker.start()

    def resume(self):
        self._running.set()

    def pause(self):
        self._running.clear()

    def quit(self):
        self._running.set()
        self._quit.set()
        if self._view_process is not None and self._view_process.is_alive():
            self._view_process.terminate()
        if self._pipe is not None:
            self._pipe.close()

    def join(self):
        for worker in self._workers:
            worker.join()

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
                arr = np.asarray(coords)
                if arr.dtype.kind == "U":
                    arr = arr.astype(object)
                xp.coords[dim_name] = arr

        store = zr.storage.LocalStore(self._file)
        compressor = zr.codecs.BloscCodec(
            cname="zstd", clevel=5, shuffle=zr.codecs.BloscShuffle.bitshuffle
        )

        for attr_name, attr in self._attrs.items():
            xp.attrs[attr_name] = attr

        try:
            with warnings.catch_warnings():
                warnings.filterwarnings("ignore", message="Consolidated metadata")
                xp.to_dataset(promote_attrs=True, name="tile").to_zarr(
                    store,
                    group=name,
                    compute=False,
                    encoding={"tile": {"compressors": compressor}},
                )
        except ContainsGroupError as e:
            raise FileExistsError(f"{self._file}/{name} already exists.") from e

        # Xarray/Dask don't natively support disk-writeable zarr arrays so we have to manually
        # load the zarr array and patch in a modified dask array that writes to disk when
        # __setitem__ is called.
        zarr_tiles = zr.open(store, path=f"{name}/tile", mode="a")
        tiles = da.from_zarr(zarr_tiles)
        tiles.__class__ = _DiskArray
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
