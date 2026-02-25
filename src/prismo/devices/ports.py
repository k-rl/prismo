import threading
import weakref

import serial
from serial.tools import list_ports

_serials: dict[str, serial.Serial] = {}
_refcounts: dict[str, int] = {}
_locks: dict[str, threading.Lock] = {}
_class_lock = threading.Lock()


class Port:
    def __init__(
        self, vid: int | None = None, pid: int | None = None, port: str | None = None, **kwargs
    ):
        if port is None:
            if vid is None or pid is None:
                raise ValueError("Either port or both vid and pid must be provided.")
            for p in list_ports.comports():
                if p.vid == vid and p.pid == pid:
                    port = p.device
                    break
        if port is None:
            raise ValueError(f"No port found with {vid=:#x} and {pid=:#x}")

        with _class_lock:
            if port not in _serials:
                _serials[port] = serial.Serial(port, **kwargs)
                _refcounts[port] = 0
                _locks[port] = threading.Lock()
            _refcounts[port] += 1

        self._name = port
        self._closed = False
        weakref.finalize(self, self.close)

    def write(self, data: bytes):
        with _locks[self._name]:
            _serials[self._name].write(data)

    def read(self, size: int) -> bytes:
        with _locks[self._name]:
            return _serials[self._name].read(size)

    def readline(self) -> bytes:
        with _locks[self._name]:
            return _serials[self._name].readline()

    def write_readline(self, data: bytes) -> bytes:
        with _locks[self._name]:
            _serials[self._name].write(data)
            return _serials[self._name].readline()

    def reset_input_buffer(self):
        with _locks[self._name]:
            _serials[self._name].reset_input_buffer()

    def close(self):
        with _class_lock:
            if self._closed:
                return
            self._closed = True
            _refcounts[self._name] -= 1
            if _refcounts[self._name] > 0:
                return
            s = _serials.pop(self._name)
            _refcounts.pop(self._name)
            _locks.pop(self._name)
        s.close()
