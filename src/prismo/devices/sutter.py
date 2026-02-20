from typing import Literal

from pymmcore import CMMCore

from . import utils

VID = 0x1342
PID = 0x1003


class Filter:
    def __init__(
        self,
        name: str,
        core: CMMCore,
        filter: str,
        port: str | None = None,
        states: list[str] | None = None,
    ):
        self.name = name
        self.states: list[int] | list[str]
        self._core = core
        port = utils.load_port(core, vid=VID, pid=PID, port=port, timeout_ms=2000, baud_rate=12800)
        core.loadDevice(name, "SutterLambda", "Wheel-" + filter)
        core.setProperty(name, "Port", port)
        core.initializeDevice(name)

        n_states = self._core.getNumberOfStates(name)
        if states is None:
            self.states = [i for i in range(n_states)]
        else:
            self.states = states
            if len(self.states) < n_states:
                raise ValueError(
                    f"{name} requires {n_states} states (not {len(self.states)}) to be specified."
                )
            for i, state in enumerate(self.states):
                self._core.defineStateLabel(name, i, state)

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


class Shutter:
    def __init__(self, name: str, core: CMMCore, shutter: str, port: str | None = None):
        self.name = name
        self._core = core
        port = utils.load_port(core, vid=VID, pid=PID, port=port, timeout_ms=2000, baud_rate=12800)
        core.loadDevice(name, "SutterLambda", "Shutter-" + shutter)
        core.setProperty(name, "Port", port)
        core.initializeDevice(name)

    def wait(self):
        self._core.waitForDevice(self.name)

    @property
    def open(self) -> bool:
        return self._core.getShutterOpen(self.name)

    @open.setter
    def open(self, new_state: bool):
        self._core.setShutterOpen(self.name, new_state)

    @property
    def state(self) -> Literal["open", "closed"]:
        return "open" if self.open else "closed"

    @state.setter
    def state(self, new_state: Literal["open", "closed"]):
        self.open = new_state == "open"
