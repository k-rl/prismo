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
    GET_MICROSTEPS = 0x0F
    SET_MICROSTEPS = 0x10
    GET_BLANK_TIME = 0x1B
    SET_BLANK_TIME = 0x1C
    GET_HYSTERESIS_END = 0x1D
    SET_HYSTERESIS_END = 0x1E
    GET_HYSTERESIS_START = 0x1F
    SET_HYSTERESIS_START = 0x20
    GET_DECAY_TIME = 0x21
    SET_DECAY_TIME = 0x22
    GET_PWM_MAX_RPM = 0x23
    SET_PWM_MAX_RPM = 0x24
    GET_DRIVER_SWITCH_AUTOSCALE_LIMIT = 0x25
    SET_DRIVER_SWITCH_AUTOSCALE_LIMIT = 0x26
    GET_MAX_AMPLITUDE_CHANGE = 0x27
    SET_MAX_AMPLITUDE_CHANGE = 0x28
    GET_PWM_AUTOGRADIENT = 0x29
    SET_PWM_AUTOGRADIENT = 0x2A
    GET_PWN_AUTOSCALE = 0x2B
    SET_PWN_AUTOSCALE = 0x2C
    GET_PWM_FREQUENCY = 0x2D
    SET_PWM_FREQUENCY = 0x2E
    GET_PWM_GRADIENT = 0x2F
    SET_PWM_GRADIENT = 0x30
    GET_PWM_OFFSET = 0x31
    SET_PWM_OFFSET = 0x32
    GET_CHARGE_PUMP_UNDERVOLTAGE = 0x33
    GET_MICROSTEP_TIME = 0x3E
    GET_MOTOR_LOAD = 0x3F
    GET_MICROSTEP_CURRENT = 0x41
    GET_PWM_MODE = 0x43
    GET_CURRENT_SCALE = 0x44
    GET_TEMPERATURE = 0x45
    GET_FLOW_HISTORY = 0x4A
    GET_VALVE = 0x4B
    SET_VALVE = 0x4C
    GET_AIR_STOP = 0x4D
    SET_AIR_STOP = 0x4E
    FAIL = 0xFF


StopMode = Literal["normal", "freewheel", "low_side", "high_side"]
BlankTime = Literal[16, 24, 36, 54]
PwmFrequency = Literal[1024, 683, 512, 410]
TemperatureThreshold = Literal["normal", "120c", "143c", "150c", "157c"]


@dataclass
class SensorInfo:
    air: bool
    high_flow: bool
    exp_smoothing: bool
    ul_per_min: float
    degrees_c: float


@dataclass
class PwmState:
    auto_grad: int
    auto_offset: int
    scale: int
    offset: int


class Sipper:
    def __init__(
        self,
        name: str,
        last_well: tuple[float, float, float] = (6.98, 18.5, 30.1),
        rows: int = 8,
        cols: int = 12,
        well_dist: float = 9.0,
    ):
        self.name = name
        self._pump = packet.PacketStream(device_id=0)
        self._cnc = packet.PacketStream(device_id=1)
        self._ul_per_min = float("nan")
        self._last_well_x, self._last_well_y, self._well_z = last_well
        self._rows = rows
        self._cols = cols
        self._well_dist = well_dist

    @property
    def air(self) -> bool:
        return self.sensor_info().air

    @property
    def high_flow(self) -> bool:
        return self.sensor_info().high_flow

    @property
    def exp_smoothing(self) -> bool:
        return self.sensor_info().exp_smoothing

    @property
    def flow_rate(self) -> float:
        return self.sensor_info().ul_per_min

    @property
    def temperature(self) -> float:
        return self.sensor_info().degrees_c

    @property
    def rpm(self) -> float:
        request = struct.pack(">B", PumpCode.GET_PUMP_RPM)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_PUMP_RPM, "d")

    @rpm.setter
    def rpm(self, rpm: float):
        request = struct.pack(">Bd", PumpCode.SET_PUMP_RPM, rpm)
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
    def microsteps(self) -> int:
        request = struct.pack(">B", PumpCode.GET_MICROSTEPS)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_MICROSTEPS, "H")

    @microsteps.setter
    def microsteps(self, microsteps: int):
        request = struct.pack(">BH", PumpCode.SET_MICROSTEPS, microsteps)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_MICROSTEPS)

    @property
    def blank_time(self) -> BlankTime:
        request = struct.pack(">B", PumpCode.GET_BLANK_TIME)
        self._pump.write(request)
        return typing.get_args(BlankTime)[self._read_pump(PumpCode.GET_BLANK_TIME, "B")]

    @blank_time.setter
    def blank_time(self, time_value: BlankTime):
        request = struct.pack(
            ">BB", PumpCode.SET_BLANK_TIME, typing.get_args(BlankTime).index(time_value)
        )
        self._pump.write(request)
        self._read_pump(PumpCode.SET_BLANK_TIME)

    @property
    def hysteresis_end(self) -> int:
        request = struct.pack(">B", PumpCode.GET_HYSTERESIS_END)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_HYSTERESIS_END, "b")

    @hysteresis_end.setter
    def hysteresis_end(self, end: int):
        request = struct.pack(">Bb", PumpCode.SET_HYSTERESIS_END, end)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_HYSTERESIS_END)

    @property
    def hysteresis_start(self) -> int:
        request = struct.pack(">B", PumpCode.GET_HYSTERESIS_START)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_HYSTERESIS_START, "B")

    @hysteresis_start.setter
    def hysteresis_start(self, start: int):
        request = struct.pack(">BB", PumpCode.SET_HYSTERESIS_START, start)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_HYSTERESIS_START)

    @property
    def decay_time(self) -> int:
        request = struct.pack(">B", PumpCode.GET_DECAY_TIME)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_DECAY_TIME, "B")

    @decay_time.setter
    def decay_time(self, time_value: int):
        request = struct.pack(">BB", PumpCode.SET_DECAY_TIME, time_value)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_DECAY_TIME)

    @property
    def pwm_max_rpm(self) -> float:
        request = struct.pack(">B", PumpCode.GET_PWM_MAX_RPM)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_PWM_MAX_RPM, "d")

    @pwm_max_rpm.setter
    def pwm_max_rpm(self, rpm: float):
        request = struct.pack(">Bd", PumpCode.SET_PWM_MAX_RPM, rpm)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_PWM_MAX_RPM)

    @property
    def driver_switch_autoscale_limit(self) -> int:
        request = struct.pack(">B", PumpCode.GET_DRIVER_SWITCH_AUTOSCALE_LIMIT)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_DRIVER_SWITCH_AUTOSCALE_LIMIT, "B")

    @driver_switch_autoscale_limit.setter
    def driver_switch_autoscale_limit(self, limit: int):
        request = struct.pack(">BB", PumpCode.SET_DRIVER_SWITCH_AUTOSCALE_LIMIT, limit)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_DRIVER_SWITCH_AUTOSCALE_LIMIT)

    @property
    def max_amplitude_change(self) -> int:
        request = struct.pack(">B", PumpCode.GET_MAX_AMPLITUDE_CHANGE)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_MAX_AMPLITUDE_CHANGE, "B")

    @max_amplitude_change.setter
    def max_amplitude_change(self, change: int):
        request = struct.pack(">BB", PumpCode.SET_MAX_AMPLITUDE_CHANGE, change)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_MAX_AMPLITUDE_CHANGE)

    @property
    def pwm_autogradient(self) -> bool:
        request = struct.pack(">B", PumpCode.GET_PWM_AUTOGRADIENT)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_PWM_AUTOGRADIENT, "?")

    @pwm_autogradient.setter
    def pwm_autogradient(self, enable: bool):
        request = struct.pack(">B?", PumpCode.SET_PWM_AUTOGRADIENT, enable)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_PWM_AUTOGRADIENT)

    @property
    def pwn_autoscale(self) -> bool:
        request = struct.pack(">B", PumpCode.GET_PWN_AUTOSCALE)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_PWN_AUTOSCALE, "?")

    @pwn_autoscale.setter
    def pwn_autoscale(self, enable: bool):
        request = struct.pack(">B?", PumpCode.SET_PWN_AUTOSCALE, enable)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_PWN_AUTOSCALE)

    @property
    def pwm_frequency(self) -> PwmFrequency:
        request = struct.pack(">B", PumpCode.GET_PWM_FREQUENCY)
        self._pump.write(request)
        return typing.get_args(PwmFrequency)[self._read_pump(PumpCode.GET_PWM_FREQUENCY, "B")]

    @pwm_frequency.setter
    def pwm_frequency(self, frequency: PwmFrequency):
        request = struct.pack(
            ">BB", PumpCode.SET_PWM_FREQUENCY, typing.get_args(PwmFrequency).index(frequency)
        )
        self._pump.write(request)
        self._read_pump(PumpCode.SET_PWM_FREQUENCY)

    @property
    def pwm_gradient(self) -> int:
        request = struct.pack(">B", PumpCode.GET_PWM_GRADIENT)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_PWM_GRADIENT, "B")

    @pwm_gradient.setter
    def pwm_gradient(self, gradient: int):
        request = struct.pack(">BB", PumpCode.SET_PWM_GRADIENT, gradient)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_PWM_GRADIENT)

    @property
    def pwm_offset(self) -> int:
        request = struct.pack(">B", PumpCode.GET_PWM_OFFSET)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_PWM_OFFSET, "B")

    @pwm_offset.setter
    def pwm_offset(self, offset: int):
        request = struct.pack(">BB", PumpCode.SET_PWM_OFFSET, offset)
        self._pump.write(request)
        self._read_pump(PumpCode.SET_PWM_OFFSET)

    @property
    def charge_pump_undervoltage(self) -> bool:
        request = struct.pack(">B", PumpCode.GET_CHARGE_PUMP_UNDERVOLTAGE)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_CHARGE_PUMP_UNDERVOLTAGE, "?")

    @property
    def microstep_time(self) -> int:
        request = struct.pack(">B", PumpCode.GET_MICROSTEP_TIME)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_MICROSTEP_TIME, "I")

    @property
    def motor_load(self) -> int:
        request = struct.pack(">B", PumpCode.GET_MOTOR_LOAD)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_MOTOR_LOAD, "H")

    @property
    def microstep_current(self) -> tuple[int, int]:
        request = struct.pack(">B", PumpCode.GET_MICROSTEP_CURRENT)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_MICROSTEP_CURRENT, "hh")

    @property
    def pwm_mode(self) -> bool:
        request = struct.pack(">B", PumpCode.GET_PWM_MODE)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_PWM_MODE, "?")

    @property
    def current_scale(self) -> int:
        request = struct.pack(">B", PumpCode.GET_CURRENT_SCALE)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_CURRENT_SCALE, "B")

    @property
    def driver_temperature(self) -> TemperatureThreshold:
        request = struct.pack(">B", PumpCode.GET_TEMPERATURE)
        self._pump.write(request)
        return typing.get_args(TemperatureThreshold)[self._read_pump(PumpCode.GET_TEMPERATURE, "B")]

    def flow_history(self) -> list[float]:
        request = struct.pack(">B", PumpCode.GET_FLOW_HISTORY)
        self._pump.write(request)
        response = self._pump.read()
        code, length = struct.unpack(">BH", response[:3])
        if code != PumpCode.GET_FLOW_HISTORY:
            raise RuntimeError(f"Expected {PumpCode.GET_FLOW_HISTORY} got {code=}.")
        return list(struct.unpack(f">{length}d", response[3:]))

    @property
    def valve(self) -> bool:
        request = struct.pack(">B", PumpCode.GET_VALVE)
        self._pump.write(request)
        return self._read_pump(PumpCode.GET_VALVE, "?")

    @valve.setter
    def valve(self, open: bool):
        request = struct.pack(">B?", PumpCode.SET_VALVE, open)
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
        return (sx / STEPS_PER_MM, sy / STEPS_PER_MM, sz / STEPS_PER_MM)

    @xyz.setter
    def xyz(self, xyz: tuple[float, float, float]):
        target = tuple(round(v * STEPS_PER_MM) for v in xyz)
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
        dx = self._well_dist * (self._cols - 1)
        dy = self._well_dist * (self._rows - 1)
        col = round((self._last_well_x + dx - x) / self._well_dist)
        row = round((self._last_well_y + dy - y) / self._well_dist)
        if not (0 <= col < self._cols and 0 <= row < self._rows):
            return ""
        return f"{chr(ord('A') + row)}{col + 1}"

    @well.setter
    def well(self, well: str):
        if not re.fullmatch(r"[A-Za-z]\d+", well):
            raise ValueError(f"Invalid well format {well!r}, expected e.g. 'A1'.")
        row = ord(well[0].upper()) - ord("A")
        col = int(well[1:]) - 1
        if not (0 <= row < self._rows and 0 <= col < self._cols):
            raise ValueError(f"Well {well!r} is outside the {self._rows}x{self._cols} plate.")
        dx = self._well_dist * (self._cols - 1)
        dy = self._well_dist * (self._rows - 1)
        x = self._last_well_x + dx - col * self._well_dist
        y = self._last_well_y + dy - row * self._well_dist
        self.z = 0.0
        self.xyz = (x, y, 0.0)
        self.xyz = (x, y, self._well_z)

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
