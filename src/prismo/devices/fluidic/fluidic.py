import math
import re
import struct
import time
import typing
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Literal

from . import packet

STEPS_PER_MM = 4096 / (math.pi * 24)


class CncCode(IntEnum):
    INIT = 0x00
    HOME = 0x01
    IS_HOMING = 0x02
    SET_POS = 0x03
    GET_POS = 0x04
    SET_SPEED = 0x05
    GET_SPEED = 0x06
    SET_ACCEL = 0x07
    GET_ACCEL = 0x08
    FAIL = 0xFF


class PumpCode(IntEnum):
    INIT = 0x00
    FLOW_SENSOR_INFO = 0x01
    SET_FLOW_UL_PER_MIN = 0x02
    SET_PUMP_RPM = 0x03
    GET_PUMP_RPM = 0x04
    GET_RMS_AMPS = 0x05
    SET_RMS_AMPS = 0x06
    GET_STOP_RMS_AMPS = 0x07
    SET_STOP_RMS_AMPS = 0x08
    GET_STOP_MODE = 0x09
    SET_STOP_MODE = 0x0A
    GET_MOTOR_LOAD = 0x3F
    GET_FLOW_HISTORY = 0x4A
    GET_VALVE = 0x4B
    SET_VALVE = 0x4C
    GET_AIR_STOP = 0x4D
    SET_AIR_STOP = 0x4E
    FAIL = 0xFF


StopMode = Literal["normal", "freewheel", "low_side", "high_side"]


@dataclass
class SensorInfo:
    air: bool
    high_flow: bool
    exp_smoothing: bool
    ul_per_min: float
    degrees_c: float


@dataclass
class Sipper:
    def __init__(
        self,
        name: str,
        origin: tuple[float, float, float] = (105.98, 18.5, 30.1),
        rows: int = 8,
        cols: int = 12,
        well_dist: float = 9.0,
        sip_rpm: float = 1.0,
        waste_rpm: float = 60.0,
        flush_time: float = 5.0,
    ):
        self.name = name
        self._pump = packet.PacketStream(device_id=0)
        self._cnc = packet.PacketStream(device_id=1)
        self._ul_per_min = float("nan")
        self._origin = origin
        self._rows = rows
        self._cols = cols
        self._well_dist = well_dist
        self._sip_rpm = sip_rpm
        self._waste_rpm = waste_rpm
        self._flush_time = flush_time
        self.air_stop = True
        self.rms_amps = 0.3
        self.home()

    @property
    def air(self) -> bool:
        return self.sensor_info().air

    @property
    def high_flow(self) -> bool:
        return self.sensor_info().high_flow

    @property
    def flow_rate(self) -> float:
        return self.sensor_info().ul_per_min

    @property
    def rpm(self) -> float:
        request = struct.pack(">B", PumpCode.GET_PUMP_RPM)
        self._pump.write(request)
        return -self._read_pump(PumpCode.GET_PUMP_RPM, "d")

    @rpm.setter
    def rpm(self, rpm: float):
        request = struct.pack(">Bd", PumpCode.SET_PUMP_RPM, -rpm)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_PUMP_RPM)

    @property
    def ul_per_min(self) -> float:
        return self._ul_per_min

    @ul_per_min.setter
    def ul_per_min(self, ul_per_min: float):
        request = struct.pack(">Bd", PumpCode.SET_FLOW_UL_PER_MIN, ul_per_min)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_FLOW_UL_PER_MIN)
        self._ul_per_min = float(ul_per_min)

    def sensor_info(self) -> SensorInfo:
        request = struct.pack(">B", PumpCode.FLOW_SENSOR_INFO)
        self._pump.write(request)
        return SensorInfo(*self._read_pump(PumpCode.FLOW_SENSOR_INFO, "???dd"))

    @property
    def rms_amps(self) -> float:
        request = struct.pack(">B", PumpCode.GET_RMS_AMPS)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_RMS_AMPS, "d")

    @rms_amps.setter
    def rms_amps(self, amps: float):
        request = struct.pack(">Bd", PumpCode.SET_RMS_AMPS, amps)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_RMS_AMPS)

    @property
    def stop_rms_amps(self) -> float:
        request = struct.pack(">B", PumpCode.GET_STOP_RMS_AMPS)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_STOP_RMS_AMPS, "d")

    @stop_rms_amps.setter
    def stop_rms_amps(self, amps: float):
        request = struct.pack(">Bd", PumpCode.SET_STOP_RMS_AMPS, amps)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_STOP_RMS_AMPS)

    @property
    def stop_mode(self) -> StopMode:
        request = struct.pack(">B", PumpCode.GET_STOP_MODE)
        self._pump.write(request)
        return typing.get_args(StopMode)[self._read_pump(PumpCode.GET_STOP_MODE, "B")]

    @stop_mode.setter
    def stop_mode(self, mode: StopMode):
        request = struct.pack(">BB", PumpCode.SET_STOP_MODE, typing.get_args(StopMode).index(mode))
        self._pump.write(request)
        self._read_pump(PumpCode.SET_STOP_MODE)

    @property
    def motor_load(self) -> int:
        request = struct.pack(">B", PumpCode.GET_MOTOR_LOAD)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_MOTOR_LOAD, "H")

    def flow_history(self) -> list[float]:
        request = struct.pack(">B", PumpCode.GET_FLOW_HISTORY)
        self._pump.write(request)
        response = self._pump.read()
        code, length = struct.unpack(">BH", response[:3])
        if code != PumpCode.GET_FLOW_HISTORY:
            raise RuntimeError(f"Expected {PumpCode.GET_FLOW_HISTORY} got {code=}.")
        return list(struct.unpack(f">{length}d", response[3:]))

    @property
    def valve(self) -> Literal["flow", "waste"]:
        request = struct.pack(">B", PumpCode.GET_VALVE)
        self._pump.write(request)
        return "flow" if self._read_pump(PumpCode.GET_VALVE, "?") else "waste"

    @valve.setter
    def valve(self, dir: Literal["flow", "waste"]):
        request = struct.pack(">B?", PumpCode.SET_VALVE, dir == "flow")
        self._pump.write(request)
        self._read_pump(PumpCode.SET_VALVE)

    @property
    def air_stop(self) -> bool:
        request = struct.pack(">B", PumpCode.GET_AIR_STOP)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_AIR_STOP, "?")

    @air_stop.setter
    def air_stop(self, enabled: bool):
        request = struct.pack(">B?", PumpCode.SET_AIR_STOP, enabled)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_AIR_STOP)

    def home(self):
        request = struct.pack(">B", CncCode.HOME)
        self._cnc.write(request)
        self._read_cnc(CncCode.HOME)
        while self.homing:
            time.sleep(0.01)

    @property
    def homing(self) -> bool:
        request = struct.pack(">B", CncCode.IS_HOMING)
        self._cnc.write(request)
        return self._read_cnc(CncCode.IS_HOMING, "?")

    @property
    def cnc_speed(self) -> float:
        """Max speed in mm/s."""
        request = struct.pack(">B", CncCode.GET_SPEED)
        self._cnc.write(request)
        return self._read_cnc(CncCode.GET_SPEED, "d") / STEPS_PER_MM

    @cnc_speed.setter
    def cnc_speed(self, value: float):
        request = struct.pack(">Bd", CncCode.SET_SPEED, value * STEPS_PER_MM)
        self._cnc.write(request)
        self._read_cnc(CncCode.SET_SPEED)

    @property
    def cnc_accel(self) -> float:
        """Acceleration in mm/s²."""
        request = struct.pack(">B", CncCode.GET_ACCEL)
        self._cnc.write(request)
        return self._read_cnc(CncCode.GET_ACCEL, "d") / STEPS_PER_MM

    @cnc_accel.setter
    def cnc_accel(self, value: float):
        request = struct.pack(">Bd", CncCode.SET_ACCEL, value * STEPS_PER_MM)
        self._cnc.write(request)
        self._read_cnc(CncCode.SET_ACCEL)

    @property
    def xyz(self) -> tuple[float, float, float]:
        request = struct.pack(">B", CncCode.GET_POS)
        self._cnc.write(request)
        sx, sy, sz = self._read_cnc(CncCode.GET_POS, "qqq")
        cx, cy, cz = sx / STEPS_PER_MM, sy / STEPS_PER_MM, sz / STEPS_PER_MM
        return (self._origin[0] - cx, cy - self._origin[1], self._origin[2] - cz)

    @xyz.setter
    def xyz(self, xyz: tuple[float, float, float]):
        cnc = (self._origin[0] - xyz[0], xyz[1] + self._origin[1], self._origin[2] - xyz[2])
        target = tuple(round(v * STEPS_PER_MM) for v in cnc)
        request = struct.pack(">Bqqq", CncCode.SET_POS, *target)
        self._cnc.write(request)
        self._read_cnc(CncCode.SET_POS)
        request = struct.pack(">B", CncCode.GET_POS)
        self._cnc.write(request)
        while self._read_cnc(CncCode.GET_POS, "qqq") != target:
            time.sleep(0.01)
            self._cnc.write(request)

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
        self.air_stop = False
        self.valve = "waste"
        if self.well:
            self.well = ""
            # Sip up an air bubble.
            self.rpm = self._waste_rpm
            time.sleep(3)
            # Move to the new well and lower sipper into liquid.
            self.rpm = 0
            self.well = well
            self.rpm = self._waste_rpm
            # Sip until the air bubble reaches the sensor.
            while not self.air:
                time.sleep(0.01)
        else:
            self.well = well
            self.rpm = self._waste_rpm
            while self.air:
                time.sleep(0.01)

        # Flush the air bubble through to waste.
        time.sleep(self._flush_time)
        self.valve = "flow"
        self.rpm = self._sip_rpm
        self.air_stop = True

    def _read_pump(self, assert_code: int, response_format: str = "") -> Any:
        return self._read(self._pump, assert_code, response_format)

    def _read_cnc(self, assert_code: int, response_format: str = "") -> Any:
        return self._read(self._cnc, assert_code, response_format)

    def _read(
        self, socket: packet.PacketStream, assert_code: int, response_format: str = ""
    ) -> Any:
        response = socket.read()
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
