import math
import re
import struct
import time
from enum import IntEnum
from typing import Any, Literal

from . import packet

STEPS_PER_MM = 4096 / (math.pi * 24)


class Code(IntEnum):
    # Pump codes.
    INIT = 0x00
    GET_AIR_IN_LINE = 0x01
    GET_FLOW_RATE = 0x02
    SET_FLOW_UL_PER_MIN = 0x03
    SET_PUMP_RPM = 0x04
    GET_PUMP_RPM = 0x05
    GET_RMS_AMPS = 0x06
    SET_RMS_AMPS = 0x07
    GET_STOP_RMS_AMPS = 0x08
    SET_STOP_RMS_AMPS = 0x09
    GET_MOTOR_LOAD = 0x0A
    GET_FLOW_HISTORY = 0x0B
    GET_VALVE = 0x0C
    SET_VALVE = 0x0D
    GET_FLUSH_TIME = 0x0E
    SET_FLUSH_TIME = 0x0F
    GET_FLUSH_RPM = 0x10
    SET_FLUSH_RPM = 0x11
    GET_FLUSHING = 0x12
    # CNC codes.
    HOME = 0x80
    IS_HOMING = 0x81
    SET_POS = 0x82
    GET_POS = 0x83
    SET_SPEED = 0x84
    GET_SPEED = 0x85
    SET_ACCEL = 0x86
    GET_ACCEL = 0x87
    # Shared codes.
    FAIL = 0xFF


class Sipper:
    def __init__(
        self,
        name: str,
        origin: tuple[float, float, float] = (105.98, 18.5, 30.1),
        rows: int = 8,
        cols: int = 12,
        well_dist: float = 9.0,
        sip_rpm: float = 1.0,
        flush_rpm: float = 60.0,
        flush_time: float = 5.0,
    ):
        self.name = name
        self._socket = packet.PacketStream(device_id=0)
        self._ul_per_min = float("nan")
        self._origin = origin
        self._rows = rows
        self._cols = cols
        self._well_dist = well_dist
        self._sip_rpm = sip_rpm
        self.flush_time = flush_time
        self.flush_rpm = flush_rpm
        self.rms_amps = 0.3
        self.home()

    @property
    def air(self) -> bool:
        request = struct.pack(">B", Code.GET_AIR_IN_LINE)
        self._socket.write(request)
        return self._read(Code.GET_AIR_IN_LINE, "?")

    @property
    def flow_rate(self) -> float:
        request = struct.pack(">B", Code.GET_FLOW_RATE)
        self._socket.write(request)
        return self._read(Code.GET_FLOW_RATE, "d")

    @property
    def rpm(self) -> float:
        request = struct.pack(">B", Code.GET_PUMP_RPM)
        self._socket.write(request)
        return -self._read(Code.GET_PUMP_RPM, "d")

    @rpm.setter
    def rpm(self, rpm: float):
        request = struct.pack(">Bd", Code.SET_PUMP_RPM, -rpm)
        self._socket.write(request)
        self._read(Code.SET_PUMP_RPM)

    @property
    def ul_per_min(self) -> float:
        return self._ul_per_min

    @ul_per_min.setter
    def ul_per_min(self, ul_per_min: float):
        request = struct.pack(">Bd", Code.SET_FLOW_UL_PER_MIN, ul_per_min)
        self._socket.write(request)
        self._read(Code.SET_FLOW_UL_PER_MIN)
        self._ul_per_min = float(ul_per_min)

    @property
    def rms_amps(self) -> float:
        request = struct.pack(">B", Code.GET_RMS_AMPS)
        self._socket.write(request)
        return self._read(Code.GET_RMS_AMPS, "d")

    @rms_amps.setter
    def rms_amps(self, amps: float):
        request = struct.pack(">Bd", Code.SET_RMS_AMPS, amps)
        self._socket.write(request)
        self._read(Code.SET_RMS_AMPS)

    @property
    def stop_rms_amps(self) -> float:
        request = struct.pack(">B", Code.GET_STOP_RMS_AMPS)
        self._socket.write(request)
        return self._read(Code.GET_STOP_RMS_AMPS, "d")

    @stop_rms_amps.setter
    def stop_rms_amps(self, amps: float):
        request = struct.pack(">Bd", Code.SET_STOP_RMS_AMPS, amps)
        self._socket.write(request)
        self._read(Code.SET_STOP_RMS_AMPS)

    @property
    def motor_load(self) -> int:
        request = struct.pack(">B", Code.GET_MOTOR_LOAD)
        self._socket.write(request)
        return self._read(Code.GET_MOTOR_LOAD, "H")

    def flow_history(self) -> list[float]:
        request = struct.pack(">B", Code.GET_FLOW_HISTORY)
        self._socket.write(request)
        response = self._socket.read()
        code, length = struct.unpack(">BH", response[:3])
        if code != Code.GET_FLOW_HISTORY:
            raise RuntimeError(f"Expected {Code.GET_FLOW_HISTORY} got {code=}.")
        return list(struct.unpack(f">{length}d", response[3:]))

    @property
    def valve(self) -> Literal["flow", "waste"]:
        request = struct.pack(">B", Code.GET_VALVE)
        self._socket.write(request)
        return "flow" if self._read(Code.GET_VALVE, "?") else "waste"

    @valve.setter
    def valve(self, dir: Literal["flow", "waste"]):
        request = struct.pack(">B?", Code.SET_VALVE, dir == "flow")
        self._socket.write(request)
        self._read(Code.SET_VALVE)

    @property
    def flush_time(self) -> float:
        request = struct.pack(">B", Code.GET_FLUSH_TIME)
        self._socket.write(request)
        return self._read(Code.GET_FLUSH_TIME, "d")

    @flush_time.setter
    def flush_time(self, seconds: float):
        request = struct.pack(">Bd", Code.SET_FLUSH_TIME, seconds)
        self._socket.write(request)
        self._read(Code.SET_FLUSH_TIME)

    @property
    def flush_rpm(self) -> float:
        request = struct.pack(">B", Code.GET_FLUSH_RPM)
        self._socket.write(request)
        return -self._read(Code.GET_FLUSH_RPM, "d")

    @flush_rpm.setter
    def flush_rpm(self, rpm: float):
        request = struct.pack(">Bd", Code.SET_FLUSH_RPM, -rpm)
        self._socket.write(request)
        self._read(Code.SET_FLUSH_RPM)

    @property
    def flushing(self) -> bool:
        request = struct.pack(">B", Code.GET_FLUSHING)
        self._socket.write(request)
        return self._read(Code.GET_FLUSHING, "?")

    def home(self):
        request = struct.pack(">B", Code.HOME)
        self._socket.write(request)
        self._read(Code.HOME)
        while self.homing:
            time.sleep(0.01)

    @property
    def homing(self) -> bool:
        request = struct.pack(">B", Code.IS_HOMING)
        self._socket.write(request)
        return self._read(Code.IS_HOMING, "?")

    @property
    def cnc_speed(self) -> float:
        """Max speed in mm/s."""
        request = struct.pack(">B", Code.GET_SPEED)
        self._socket.write(request)
        return self._read(Code.GET_SPEED, "d") / STEPS_PER_MM

    @cnc_speed.setter
    def cnc_speed(self, value: float):
        request = struct.pack(">Bd", Code.SET_SPEED, value * STEPS_PER_MM)
        self._socket.write(request)
        self._read(Code.SET_SPEED)

    @property
    def cnc_accel(self) -> float:
        """Acceleration in mm/s²."""
        request = struct.pack(">B", Code.GET_ACCEL)
        self._socket.write(request)
        return self._read(Code.GET_ACCEL, "d") / STEPS_PER_MM

    @cnc_accel.setter
    def cnc_accel(self, value: float):
        request = struct.pack(">Bd", Code.SET_ACCEL, value * STEPS_PER_MM)
        self._socket.write(request)
        self._read(Code.SET_ACCEL)

    @property
    def xyz(self) -> tuple[float, float, float]:
        request = struct.pack(">B", Code.GET_POS)
        self._socket.write(request)
        sx, sy, sz = self._read(Code.GET_POS, "qqq")
        cx, cy, cz = sx / STEPS_PER_MM, sy / STEPS_PER_MM, sz / STEPS_PER_MM
        return (self._origin[0] - cx, cy - self._origin[1], self._origin[2] - cz)

    @xyz.setter
    def xyz(self, xyz: tuple[float, float, float]):
        cnc = (self._origin[0] - xyz[0], xyz[1] + self._origin[1], self._origin[2] - xyz[2])
        target = tuple(round(v * STEPS_PER_MM) for v in cnc)
        request = struct.pack(">Bqqq", Code.SET_POS, *target)
        self._socket.write(request)
        self._read(Code.SET_POS)
        request = struct.pack(">B", Code.GET_POS)
        self._socket.write(request)
        while self._read(Code.GET_POS, "qqq") != target:
            time.sleep(0.01)
            self._socket.write(request)

    @property
    def x(self) -> float:
        return self.xyz[0]

    @x.setter
    def x(self, value: float):
        _, y, z = self.xyz
        self.xyz = (value, y, z)

    @property
    def y(self) -> float:
        return self.xyz[1]

    @y.setter
    def y(self, value: float):
        x, _, z = self.xyz
        self.xyz = (x, value, z)

    @property
    def z(self) -> float:
        return self.xyz[2]

    @z.setter
    def z(self, value: float):
        x, y, _ = self.xyz
        self.xyz = (x, y, value)

    @property
    def well(self) -> str:
        x, y, _ = self.xyz
        col = round(x / self._well_dist)
        row = self._rows - 1 - round(y / self._well_dist)
        if not (0 <= col < self._cols and 0 <= row < self._rows) or self.z >= 1.0:
            return ""
        return f"{chr(ord('A') + row)}{col + 1}"

    @well.setter
    def well(self, well: str):
        if not well:
            self.z = self._origin[2]
            return
        if not re.fullmatch(r"[A-Za-z]\d+", well):
            raise ValueError(f"Invalid well format {well!r}, expected e.g. 'A1'.")

        # Convert well string to 0-indexed row/col.
        row = ord(well[0].upper()) - ord("A")
        col = int(well[1:]) - 1
        if not (0 <= row < self._rows and 0 <= col < self._cols):
            raise ValueError(f"Well {well!r} is outside the {self._rows}x{self._cols} plate.")

        # Lift sipper up.
        self.z = self._origin[2]
        # Move sipper over the well.
        x = col * self._well_dist
        y = (self._rows - 1 - row) * self._well_dist
        self.xyz = (x, y, self._origin[2])
        # Put sipper down.
        self.z = 0.0

    def sip(self, well: str):
        # Move the sipper up.
        self.well = ""
        # Clear out the line of any liquid.
        self.valve = "waste"
        self.rpm = self.flush_rpm
        while not self.air:
            time.sleep(0.01)
        # Move to the new well and sip liquid.
        self.well = well
        self.rpm = self._sip_rpm
        self.valve = "flow"
        # Wait until the MCU autoflushes until we get to this well's liquid.
        while self.flushing:
            time.sleep(0.01)

    def close(self):
        self._socket.close()

    def _read(self, assert_code: int, response_format: str = "") -> Any:
        response = self._socket.read()
        code = struct.unpack(">B", response[:1])[0]
        if code == 0xFF:
            raise RuntimeError("Device reported failure.")
        elif code != assert_code:
            raise RuntimeError(f"Expected {assert_code} got {code=}.")

        payload = struct.unpack(">" + response_format, response[1:])
        if len(payload) == 1:
            return payload[0]
        else:
            return payload
