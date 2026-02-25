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
        return float(resp.split(" ")[1])

    @x.setter
    def x(self, new_x: float):
        self._cmd(f"MOVE X={float(new_x):.4f}")

    @property
    def y(self) -> float:
        resp = self._cmd("WHERE Y")
        return float(resp.split(" ")[1])

    @y.setter
    def y(self, new_y: float):
        self._cmd(f"MOVE Y={float(new_y):.4f}")

    @property
    def xy(self) -> np.ndarray:
        resp = self._cmd("WHERE X Y").split(" ")
        x, y = float(resp[1]), float(resp[2])
        return np.array([x, y])

    @xy.setter
    def xy(self, new_xy: tuple[float, float]):
        self._cmd(f"MOVE X={float(new_xy[0]):.4f} Y={float(new_xy[1]):.4f}")

    def __iadd__(self, delta: tuple[float, float]) -> "Stage":
        self._cmd(f"MOVREL X={float(delta[0]):.4f} Y={float(delta[1]):.4f}")
        return self

    def __isub__(self, delta: tuple[float, float]) -> "Stage":
        self._cmd(f"MOVREL X={-float(delta[0]):.4f} Y={-float(delta[1]):.4f}")
        return self


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
        return float(resp.split(" ")[1])

    @z.setter
    def z(self, new_z: float):
        self._cmd(f"MOVE Z={float(new_z):.4f}")

    def __iadd__(self, delta: float) -> "Focus":
        self._cmd(f"MOVREL Z={float(delta):.4f}")
        return self

    def __isub__(self, delta: float) -> "Focus":
        self._cmd(f"MOVREL Z={-float(delta):.4f}")
        return self
