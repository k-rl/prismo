import weakref
from collections.abc import Buffer

import serial
from serial.tools import list_ports


class PacketStream:
    def __init__(self, device_id: int, timeout_s: int = 1):
        device_found = False
        for port in list_ports.comports():
            if port.vid == 0x303A and port.pid == 0x1001:
                self._socket = serial.Serial(port.device, baudrate=115200, timeout=timeout_s)
                try:
                    self._socket.reset_input_buffer()
                    self.write(bytes([0]))
                    result = self.read()
                    if len(result) == 2 and result[0] == 0 and result[1] == device_id:
                        device_found = True
                        break
                except Exception:
                    pass
                self._socket.close()
        if not device_found:
            raise ConnectionError(f"Could not find device with id {device_id}.")

        # Make sure we close the socket when the object is garbage collected.
        weakref.finalize(self, self.close)

    def write(self, request: Buffer):
        data = memoryview(request).cast("B")
        if len(data) == 0:
            return
        offset = 1
        offset_idx = 0
        out = bytearray([0])
        for b in data:
            if b != 0:
                out.append(b)
                offset += 1
            if b == 0 or offset == 255:
                out.append(0)
                out[offset_idx] = offset
                offset = 1
                offset_idx = len(out) - 1

        if offset_idx != len(out) - 1 or data[-1] == 0:
            out[offset_idx] = offset
            out.append(0)

        self._socket.write(out)

    def read(self) -> bytearray:
        size = 0
        while size == 0:
            size = self._timeout_read(1)[0]

        out = bytearray()
        while size != 0:
            buf = self._timeout_read(size - 1)
            if any(b == 0 for b in buf):
                while self._timeout_read(1)[0] != 0:
                    pass
                raise ValueError("Received unexpected zero byte in packet data.")
            out.extend(buf)
            prev_size = size
            size = self._timeout_read(1)[0]
            if size == 0:
                break
            elif prev_size != 255:
                out.append(0)

        return out

    def _timeout_read(self, size: int) -> bytes:
        out = self._socket.read(size)
        if len(out) < size:
            raise TimeoutError("Read timed out.")
        return out

    def close(self):
        self._socket.close()
