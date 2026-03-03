from typing import Literal

from pymmcore import CMMCore

from . import utils


class SolaLight:
    def __init__(
        self,
        name: str,
        core: CMMCore,
        port: str | None = None,
        version: Literal["sola", "spectra"] = "sola",
    ):
        self.name = name
        self._core = core

        port = utils.load_port(core, vid=0x0403, pid=0x6001, port=port)
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


class RetraLight:
    def __init__(self, name: str, core: CMMCore, ip: str):
        self.name = name
        self._core = core

        core.loadDevice(name, "Lumencor", "LightEngine")
        core.setProperty(name, "Model", "Gen3")
        core.setProperty(name, "Connection", ip)
        core.initializeDevice(name)

    def wait(self):
        self._core.waitForDevice(self.name)

    @property
    def state(self) -> int:
        return int(self._core.getProperty(self.name, "UV340_Intensity"))

    @state.setter
    def state(self, new_state: int):
        self._core.setProperty(self.name, "UV340", 1)
        self._core.setProperty(self.name, "UV340_Intensity", new_state)
