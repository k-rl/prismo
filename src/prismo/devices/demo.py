import time

import numpy as np

_x = 0.0
_y = 0.0
_exposure = 10.0
_filter_state = 0


class Camera:
    def __init__(self, name: str, shape: tuple[int, int] = (512, 512)):
        self.name = name
        self._shape = shape

    def snap(self) -> np.ndarray:
        t = time.time()
        y, x = np.ogrid[: self._shape[0], : self._shape[1]]
        # Sine wave that shifts with time, position, and filter state
        phase = _filter_state * np.pi / 2
        freq = 1 + _filter_state * 0.2
        img = np.sin(x / 20 * freq + t + _x / 100 + phase) * np.sin(
            y / 20 * freq + t * 0.7 + _y / 100
        )
        # Normalize to uint16 range
        img = ((img + 1) / 2 * 65535).astype(np.uint16)
        return img

    def wait(self):
        pass

    @property
    def exposure(self) -> float:
        return _exposure

    @exposure.setter
    def exposure(self, new_exposure: float):
        global _exposure
        _exposure = new_exposure

    @property
    def px_len(self) -> float:
        return 1.0


class Stage:
    def __init__(self, name: str):
        self.name = name

    def wait(self):
        pass

    @property
    def x(self) -> float:
        return _x

    @x.setter
    def x(self, new_x: float):
        global _x
        _x = new_x

    @property
    def y(self) -> float:
        return _y

    @y.setter
    def y(self, new_y: float):
        global _y
        _y = new_y

    @property
    def xy(self) -> tuple[float, float]:
        return (_x, _y)

    @xy.setter
    def xy(self, new_xy: tuple[float, float]):
        global _x, _y
        _x, _y = new_xy


class Filter:
    def __init__(self, name: str, states: list[str] | None = None):
        self.name = name
        if states is None:
            self.states = ["A", "B", "C", "D"]
        else:
            self.states = states

    def wait(self):
        pass

    @property
    def state(self) -> int | str:
        return self.states[_filter_state]

    @state.setter
    def state(self, new_state: int | str):
        global _filter_state
        _filter_state = new_state if isinstance(new_state, int) else self.states.index(new_state)


class Valves:
    def __init__(self, name: str, valves: list[int] | None = None):
        self.name = name
        if valves is None:
            valves = list(range(48))
        self.valves = {k: 1 for k in valves}

    def __getitem__(self, key: int) -> int:
        return self.valves[key]

    def __setitem__(self, key: int, value: int | str):
        self.valves[key] = int((value != "off") and (value != 0))
