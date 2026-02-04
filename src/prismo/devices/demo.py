import numpy as np
from pymmcore import CMMCore


class Camera:
    def __init__(self, name: str, core: CMMCore):
        self.name = name
        self._core = core
        core.loadDevice(name, "DemoCamera", "DCam")
        core.initializeDevice(name)

    def snap(self) -> np.ndarray:
        self._core.setCameraDevice(self.name)
        self._core.snapImage()
        return self._core.getImage()

    def wait(self):
        self._core.waitForDevice(self.name)

    @property
    def exposure(self) -> float:
        return self._core.getExposure(self.name)

    @exposure.setter
    def exposure(self, new_exposure: float):
        self._core.setExposure(self.name, new_exposure)

    @property
    def px_len(self) -> float:
        # TODO: Find out the pixel length of the demo camera.
        return 1.0


class Stage:
    def __init__(self, name: str, core: CMMCore):
        self.name = name
        self._core = core
        core.loadDevice(name, "DemoCamera", "DXYStage")
        core.initializeDevice(name)

    def wait(self):
        self._core.waitForDevice(self.name)

    @property
    def x(self) -> float:
        return self._core.getXPosition(self.name)

    @x.setter
    def x(self, new_x: float):
        self._core.setXYPosition(self.name, new_x, self.y)

    @property
    def y(self) -> float:
        return self._core.getYPosition(self.name)

    @y.setter
    def y(self, new_y: float):
        self._core.setXYPosition(self.name, self.x, new_y)

    @property
    def xy(self) -> tuple[float, float]:
        return np.array(self._core.getXYPosition(self.name))

    @xy.setter
    def xy(self, new_xy: tuple[float, float]):
        self._core.setXYPosition(self.name, new_xy[0], new_xy[1])


class Filter:
    def __init__(self, name: str, core: CMMCore, states: list[str] | None = None):
        self.name = name
        self._core = core
        core.loadDevice(name, "DemoCamera", "DWheel")
        core.initializeDevice(name)

        n_states = self._core.getNumberOfStates(name)
        assert isinstance(n_states, int)
        if states is None:
            self.states = [i for i in range(n_states)]
        else:
            if len(states) < n_states:
                raise ValueError(
                    f"{name} requires {n_states} states (not {len(states)}) to be specified."
                )
            for i, state in enumerate(states):
                self._core.defineStateLabel(name, i, state)
            self.states = states

    def wait(self):
        self._core.waitForDevice(self.name)

    @property
    def state(self) -> int | str:
        if isinstance(self.states[0], int):
            return self._core.getState(self.name)
        else:
            return self._core.getStateLabel(self.name)

    @state.setter
    def state(self, new_state: int | str):
        if isinstance(new_state, int):
            self._core.setState(self.name, new_state)
        else:
            self._core.setStateLabel(self.name, new_state)


class Valves:
    def __init__(self, name: str, valves: list[int] | None = None):
        self.name = name
        if valves is None:
            valves = [i for i in range(48)]
        self.valves = {k: 1 for k in valves}

    def __getitem__(self, key: int) -> int:
        return self.valves[key]

    def __setitem__(self, key: int, value: int | str):
        self.valves[key] = int((value != "off") and (value != 0))
