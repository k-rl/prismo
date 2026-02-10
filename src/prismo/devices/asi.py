import numpy as np
from pymmcore import CMMCore


class Stage:
    def __init__(self, name: str, core: CMMCore, port: str):
        self.name = name
        self._core = core
        core.loadDevice(name, "ASIStage", "XYStage")
        core.setProperty(name, "Port", port)
        core.initializeDevice(name)

    def wait(self):
        self._core.waitForDevice(self.name)

    @property
    def x(self) -> float:
        return self._core.getXPosition(self.name)

    @x.setter
    def x(self, new_x: float):
        self._core.setXYPosition(self.name, float(new_x), self.y)

    @property
    def y(self) -> float:
        return self._core.getYPosition(self.name)

    @y.setter
    def y(self, new_y: float):
        self._core.setXYPosition(self.name, self.x, float(new_y))

    @property
    def xy(self) -> np.ndarray:
        return np.array(self._core.getXYPosition(self.name))

    @xy.setter
    def xy(self, new_xy: tuple[float, float]):
        self._core.setXYPosition(self.name, float(new_xy[0]), float(new_xy[1]))


class Focus:
    def __init__(self, name: str, core: CMMCore, port: str):
        self.name = name
        self._core = core
        core.loadDevice(name, "ASIStage", "ZStage")
        core.setProperty(name, "Port", port)
        core.setProperty(name, "Axis", "Z")
        core.initializeDevice(name)

    def wait(self):
        self._core.waitForDevice(self.name)

    @property
    def z(self) -> float:
        return self._core.getPosition(self.name)

    @z.setter
    def z(self, new_z: float):
        self._core.setPosition(self.name, float(new_z))
