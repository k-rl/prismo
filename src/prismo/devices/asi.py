import numpy as np

from .ports import Port

VID = 0x10C4
PID = 0xEA60


class Stage:
    def __init__(self, name: str, port: str | None = None):
        self.name = name
        self._port = Port(VID, PID, port, baudrate=9600, timeout=2.0)

    def _cmd(self, cmd: str) -> str:
        self._port.write((cmd + "\r").encode())
        response = self._port.readline().decode().strip()
        if response.startswith(":N"):
            raise RuntimeError(f"ASI error: {response}")
        return response

    def wait(self):
        while not self._cmd("/").startswith(":A"):
            pass

    def close(self):
        self._port.close()

    @property
    def x(self) -> float:
        resp = self._cmd("WHERE X")
        return float(resp.split("X=")[1].split()[0])

    @x.setter
    def x(self, new_x: float):
        self._cmd(f"MOVE X={float(new_x):.4f}")

    @property
    def y(self) -> float:
        resp = self._cmd("WHERE Y")
        return float(resp.split("Y=")[1].split()[0])

    @y.setter
    def y(self, new_y: float):
        self._cmd(f"MOVE Y={float(new_y):.4f}")

    @property
    def xy(self) -> np.ndarray:
        resp = self._cmd("WHERE X Y")
        x = float(resp.split("X=")[1].split()[0])
        y = float(resp.split("Y=")[1].split()[0])
        return np.array([x, y])

    @xy.setter
    def xy(self, new_xy: tuple[float, float]):
        self._cmd(f"MOVE X={float(new_xy[0]):.4f} Y={float(new_xy[1]):.4f}")


class Focus:
    def __init__(self, name: str, port: str | None = None):
        self.name = name
        self._port = Port(VID, PID, port, baudrate=9600, timeout=2.0)

    def _cmd(self, cmd: str) -> str:
        self._port.write((cmd + "\r").encode())
        response = self._port.readline().decode().strip()
        if response.startswith(":N"):
            raise RuntimeError(f"ASI error: {response}")
        return response

    def wait(self):
        while not self._cmd("/").startswith(":A"):
            pass

    def close(self):
        self._port.close()

    @property
    def z(self) -> float:
        resp = self._cmd("WHERE Z")
        return float(resp.split("Z=")[1].split()[0])

    @z.setter
    def z(self, new_z: float):
        self._cmd(f"MOVE Z={float(new_z):.4f}")
