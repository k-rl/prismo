from typing import Literal

from pymmcore import CMMCore


class Light:
    def __init__(
        self, name: str, core: CMMCore, port: str, version: Literal["sola", "spectra"] = "sola"
    ):
        self.name = name
        self._core = core
        core.loadDevice(name, "LumencorSpectra", "Spectra")
        core.setProperty(name, "SetLE_Type", version.capitalize())
        core.setProperty(name, "Port", port)
        core.initializeDevice(name)

    def wait(self):
        self._core.waitForDevice(self.name)

    @property
    def state(self) -> int:
        return int(self._core.getProperty(self.name, "White_Level"))

    @state.setter
    def state(self, new_state: int):
        self._core.setProperty(self.name, "White_Level", new_state)
