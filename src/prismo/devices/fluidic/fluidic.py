import contextlib
import re
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from numbers import Real
from typing import Any

import numpy as np
import serial

from . import packet


class Code(IntEnum):
    INIT = 0x00
    FLOW_SENSOR_INFO = 0x01
    SET_PUMP_RPM = 0x02
    GET_PUMP_RPM = 0x03
    GET_RMS_AMPS = 0x0C
    SET_RMS_AMPS = 0x0D
    GET_STOP_RMS_AMPS = 0x70
    SET_STOP_RMS_AMPS = 0x71
    GET_STOP_MODE = 0x0E
    SET_STOP_MODE = 0x0F
    GET_POWERDOWN_DURATION_S = 0x10
    SET_POWERDOWN_DURATION_S = 0x11
    GET_POWERDOWN_DELAY_S = 0x12
    SET_POWERDOWN_DELAY_S = 0x13
    GET_MICROSTEPS = 0x14
    SET_MICROSTEPS = 0x15
    GET_FILTER_STEP_PULSES = 0x18
    SET_FILTER_STEP_PULSES = 0x19
    GET_DOUBLE_EDGE_STEP = 0x1A
    SET_DOUBLE_EDGE_STEP = 0x1B
    GET_INTERPOLATE_MICROSTEPS = 0x1C
    SET_INTERPOLATE_MICROSTEPS = 0x1D
    GET_SHORT_SUPPLY_PROTECT = 0x2C
    SET_SHORT_SUPPLY_PROTECT = 0x2D
    GET_SHORT_GROUND_PROTECT = 0x2E
    SET_SHORT_GROUND_PROTECT = 0x2F
    GET_BLANK_TIME = 0x30
    SET_BLANK_TIME = 0x31
    GET_HYSTERESIS_END = 0x32
    SET_HYSTERESIS_END = 0x33
    GET_HYSTERESIS_START = 0x34
    SET_HYSTERESIS_START = 0x35
    GET_DECAY_TIME = 0x36
    SET_DECAY_TIME = 0x37
    GET_PWM_MAX_RPM = 0x38
    SET_PWM_MAX_RPM = 0x39
    GET_DRIVER_SWITCH_AUTOSCALE_LIMIT = 0x3A
    SET_DRIVER_SWITCH_AUTOSCALE_LIMIT = 0x3B
    GET_MAX_AMPLITUDE_CHANGE = 0x3C
    SET_MAX_AMPLITUDE_CHANGE = 0x3D
    GET_PWM_AUTOGRADIENT = 0x3E
    SET_PWM_AUTOGRADIENT = 0x3F
    GET_PWN_AUTOSCALE = 0x40
    SET_PWN_AUTOSCALE = 0x41
    GET_PWM_FREQUENCY = 0x42
    SET_PWM_FREQUENCY = 0x43
    GET_PWM_GRADIENT = 0x44
    SET_PWM_GRADIENT = 0x45
    GET_PWM_OFFSET = 0x46
    SET_PWM_OFFSET = 0x47
    GET_CHARGE_PUMP_UNDERVOLTAGE = 0x4C
    GET_DRIVER_ERROR = 0x4D
    GET_IS_RESET = 0x4E
    GET_TRANSMISSION_COUNT = 0x4F
    GET_DIRECTION_PIN = 0x51
    GET_DISABLE_PWM_PIN = 0x52
    GET_STEP_PIN = 0x53
    GET_POWERDOWN_UART_PIN = 0x54
    GET_DIAGNOSTIC_PIN = 0x55
    GET_MICROSTEP2_PIN = 0x56
    GET_MICROSTEP1_PIN = 0x57
    GET_DISABLE_PIN = 0x58
    GET_MICROSTEP_TIME = 0x59
    GET_MOTOR_LOAD = 0x5A
    GET_MICROSTEP_POSITION = 0x5B
    GET_MICROSTEP_CURRENT = 0x5C
    GET_STOPPED = 0x5D
    GET_PWM_MODE = 0x5E
    GET_CURRENT_SCALE = 0x5F
    GET_TEMPERATURE = 0x60
    GET_OPEN_LOAD = 0x61
    GET_LOW_SIDE_SHORT = 0x62
    GET_GROUND_SHORT = 0x63
    GET_OVERTEMPERATURE = 0x64
    FAIL = 0xFF


STOP_CODES = ["normal", "freewheel", "low_side", "high_side"]
BLANK_TIMES = ["b16", "b24", "b36", "b54"]
PWM_FREQUENCIES = ["div1024", "div683", "div512", "div410"]
PHASE_STATUSES = ["none", "phase_a", "phase_b", "both_phases"]
TEMPERATURE_THRESHOLDS = ["normal", "temp_120c", "temp_143c", "temp_150c", "temp_157c"]
OVERTEMPERATURE_STATUSES = ["normal", "warning", "shutdown"]
BLANK_TIMES = ["b16", "b24", "b36", "b54"]
PWM_FREQUENCIES = ["div1024", "div683", "div512", "div410"]
PHASE_STATUSES = ["none", "phase_a", "phase_b", "both_phases"]
TEMPERATURE_THRESHOLDS = ["normal", "temp_120c", "temp_143c", "temp_150c", "temp_157c"]
OVERTEMPERATURE_STATUSES = ["normal", "warning", "shutdown"]


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


class FlowController:
    def __init__(self, name):
        self.name = name
        self._socket = packet.PacketStream()

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
        request = struct.pack(">B", Code.GET_PUMP_RPM)
        self._socket.write(request)
        return self._read_packet(Code.GET_PUMP_RPM, "d")

    @rpm.setter
    def rpm(self, rpm: Real):
        request = struct.pack(">Bd", Code.SET_PUMP_RPM, rpm)
        self._socket.write(request)
        self._read_packet(Code.SET_PUMP_RPM)

    def sensor_info(self) -> SensorInfo:
        request = struct.pack(">B", Code.FLOW_SENSOR_INFO)
        self._socket.write(request)
        return SensorInfo(*self._read_packet(Code.FLOW_SENSOR_INFO, "???dd"))

    @property
    def rms_amps(self) -> float:
        request = struct.pack(">B", Code.GET_RMS_AMPS)
        self._socket.write(request)
        return self._read_packet(Code.GET_RMS_AMPS, "d")

    @rms_amps.setter
    def rms_amps(self, amps: Real):
        request = struct.pack(">Bd", Code.SET_RMS_AMPS, amps)
        self._socket.write(request)
        self._read_packet(Code.SET_RMS_AMPS)

    @property
    def stop_rms_amps(self) -> float:
        request = struct.pack(">B", Code.GET_STOP_RMS_AMPS)
        self._socket.write(request)
        return self._read_packet(Code.GET_STOP_RMS_AMPS, "d")

    @stop_rms_amps.setter
    def stop_rms_amps(self, amps: Real):
        request = struct.pack(">Bd", Code.SET_STOP_RMS_AMPS, amps)
        self._socket.write(request)
        self._read_packet(Code.SET_STOP_RMS_AMPS)

    @property
    def stop_mode(self) -> str:
        request = struct.pack(">B", Code.GET_STOP_MODE)
        self._socket.write(request)
        return STOP_CODES[self._read_packet(Code.GET_STOP_MODE, "B")]

    @stop_mode.setter
    def stop_mode(self, mode: str):
        request = struct.pack(">BB", Code.SET_STOP_MODE, STOP_CODES.index(mode))
        self._socket.write(request)
        self._read_packet(Code.SET_STOP_MODE)

    @property
    def powerdown_duration_s(self) -> float:
        request = struct.pack(">B", Code.GET_POWERDOWN_DURATION_S)
        self._socket.write(request)
        return self._read_packet(Code.GET_POWERDOWN_DURATION_S, "d")

    @powerdown_duration_s.setter
    def powerdown_duration_s(self, duration: Real):
        request = struct.pack(">Bd", Code.SET_POWERDOWN_DURATION_S, duration)
        self._socket.write(request)
        self._read_packet(Code.SET_POWERDOWN_DURATION_S)

    @property
    def powerdown_delay_s(self) -> float:
        request = struct.pack(">B", Code.GET_POWERDOWN_DELAY_S)
        self._socket.write(request)
        return self._read_packet(Code.GET_POWERDOWN_DELAY_S, "d")

    @powerdown_delay_s.setter
    def powerdown_delay_s(self, delay: Real):
        request = struct.pack(">Bd", Code.SET_POWERDOWN_DELAY_S, delay)
        self._socket.write(request)
        self._read_packet(Code.SET_POWERDOWN_DELAY_S)

    @property
    def microsteps(self) -> int:
        request = struct.pack(">B", Code.GET_MICROSTEPS)
        self._socket.write(request)
        return self._read_packet(Code.GET_MICROSTEPS, "H")

    @microsteps.setter
    def microsteps(self, microsteps: int):
        request = struct.pack(">BH", Code.SET_MICROSTEPS, microsteps)
        self._socket.write(request)
        self._read_packet(Code.SET_MICROSTEPS)

    @property
    def filter_step_pulses(self) -> bool:
        request = struct.pack(">B", Code.GET_FILTER_STEP_PULSES)
        self._socket.write(request)
        return self._read_packet(Code.GET_FILTER_STEP_PULSES, "?")

    @filter_step_pulses.setter
    def filter_step_pulses(self, enable: bool):
        request = struct.pack(">B?", Code.SET_FILTER_STEP_PULSES, enable)
        self._socket.write(request)
        self._read_packet(Code.SET_FILTER_STEP_PULSES)

    @property
    def double_edge_step(self) -> bool:
        request = struct.pack(">B", Code.GET_DOUBLE_EDGE_STEP)
        self._socket.write(request)
        return self._read_packet(Code.GET_DOUBLE_EDGE_STEP, "?")

    @double_edge_step.setter
    def double_edge_step(self, enable: bool):
        request = struct.pack(">B?", Code.SET_DOUBLE_EDGE_STEP, enable)
        self._socket.write(request)
        self._read_packet(Code.SET_DOUBLE_EDGE_STEP)

    @property
    def interpolate_microsteps(self) -> bool:
        request = struct.pack(">B", Code.GET_INTERPOLATE_MICROSTEPS)
        self._socket.write(request)
        return self._read_packet(Code.GET_INTERPOLATE_MICROSTEPS, "?")

    @interpolate_microsteps.setter
    def interpolate_microsteps(self, enable: bool):
        request = struct.pack(">B?", Code.SET_INTERPOLATE_MICROSTEPS, enable)
        self._socket.write(request)
        self._read_packet(Code.SET_INTERPOLATE_MICROSTEPS)

    @property
    def short_supply_protect(self) -> bool:
        request = struct.pack(">B", Code.GET_SHORT_SUPPLY_PROTECT)
        self._socket.write(request)
        return self._read_packet(Code.GET_SHORT_SUPPLY_PROTECT, "?")

    @short_supply_protect.setter
    def short_supply_protect(self, enable: bool):
        request = struct.pack(">B?", Code.SET_SHORT_SUPPLY_PROTECT, enable)
        self._socket.write(request)
        self._read_packet(Code.SET_SHORT_SUPPLY_PROTECT)

    @property
    def short_ground_protect(self) -> bool:
        request = struct.pack(">B", Code.GET_SHORT_GROUND_PROTECT)
        self._socket.write(request)
        return self._read_packet(Code.GET_SHORT_GROUND_PROTECT, "?")

    @short_ground_protect.setter
    def short_ground_protect(self, enable: bool):
        request = struct.pack(">B?", Code.SET_SHORT_GROUND_PROTECT, enable)
        self._socket.write(request)
        self._read_packet(Code.SET_SHORT_GROUND_PROTECT)

    @property
    def blank_time(self) -> str:
        request = struct.pack(">B", Code.GET_BLANK_TIME)
        self._socket.write(request)
        return BLANK_TIMES[self._read_packet(Code.GET_BLANK_TIME, "B")]

    @blank_time.setter
    def blank_time(self, time_value):
        time_value = BLANK_TIMES.index(time_value.strip().lower())
        request = struct.pack(">BB", Code.SET_BLANK_TIME, time_value)
        self._socket.write(request)
        self._read_packet(Code.SET_BLANK_TIME)

    @property
    def hysteresis_end(self) -> int:
        request = struct.pack(">B", Code.GET_HYSTERESIS_END)
        self._socket.write(request)
        return self._read_packet(Code.GET_HYSTERESIS_END, "b")

    @hysteresis_end.setter
    def hysteresis_end(self, end: int):
        request = struct.pack(">Bb", Code.SET_HYSTERESIS_END, end)
        self._socket.write(request)
        self._read_packet(Code.SET_HYSTERESIS_END)

    @property
    def hysteresis_start(self) -> int:
        request = struct.pack(">B", Code.GET_HYSTERESIS_START)
        self._socket.write(request)
        return self._read_packet(Code.GET_HYSTERESIS_START, "B")

    @hysteresis_start.setter
    def hysteresis_start(self, start: int):
        request = struct.pack(">BB", Code.SET_HYSTERESIS_START, start)
        self._socket.write(request)
        self._read_packet(Code.SET_HYSTERESIS_START)

    @property
    def decay_time(self) -> int:
        request = struct.pack(">B", Code.GET_DECAY_TIME)
        self._socket.write(request)
        return self._read_packet(Code.GET_DECAY_TIME, "B")

    @decay_time.setter
    def decay_time(self, time_value: int):
        request = struct.pack(">BB", Code.SET_DECAY_TIME, time_value)
        self._socket.write(request)
        self._read_packet(Code.SET_DECAY_TIME)

    @property
    def pwm_max_rpm(self) -> float:
        request = struct.pack(">B", Code.GET_PWM_MAX_RPM)
        self._socket.write(request)
        return self._read_packet(Code.GET_PWM_MAX_RPM, "d")

    @pwm_max_rpm.setter
    def pwm_max_rpm(self, rpm: Real):
        request = struct.pack(">Bd", Code.SET_PWM_MAX_RPM, rpm)
        self._socket.write(request)
        self._read_packet(Code.SET_PWM_MAX_RPM)

    @property
    def driver_switch_autoscale_limit(self) -> int:
        request = struct.pack(">B", Code.GET_DRIVER_SWITCH_AUTOSCALE_LIMIT)
        self._socket.write(request)
        return self._read_packet(Code.GET_DRIVER_SWITCH_AUTOSCALE_LIMIT, "B")

    @driver_switch_autoscale_limit.setter
    def driver_switch_autoscale_limit(self, limit: int):
        request = struct.pack(">BB", Code.SET_DRIVER_SWITCH_AUTOSCALE_LIMIT, limit)
        self._socket.write(request)
        self._read_packet(Code.SET_DRIVER_SWITCH_AUTOSCALE_LIMIT)

    @property
    def max_amplitude_change(self) -> int:
        request = struct.pack(">B", Code.GET_MAX_AMPLITUDE_CHANGE)
        self._socket.write(request)
        return self._read_packet(Code.GET_MAX_AMPLITUDE_CHANGE, "B")

    @max_amplitude_change.setter
    def max_amplitude_change(self, change: int):
        request = struct.pack(">BB", Code.SET_MAX_AMPLITUDE_CHANGE, change)
        self._socket.write(request)
        self._read_packet(Code.SET_MAX_AMPLITUDE_CHANGE)

    @property
    def pwm_autogradient(self) -> bool:
        request = struct.pack(">B", Code.GET_PWM_AUTOGRADIENT)
        self._socket.write(request)
        return self._read_packet(Code.GET_PWM_AUTOGRADIENT, "?")

    @pwm_autogradient.setter
    def pwm_autogradient(self, enable: bool):
        request = struct.pack(">B?", Code.SET_PWM_AUTOGRADIENT, enable)
        self._socket.write(request)
        self._read_packet(Code.SET_PWM_AUTOGRADIENT)

    @property
    def pwn_autoscale(self) -> bool:
        request = struct.pack(">B", Code.GET_PWN_AUTOSCALE)
        self._socket.write(request)
        return self._read_packet(Code.GET_PWN_AUTOSCALE, "?")

    @pwn_autoscale.setter
    def pwn_autoscale(self, enable: bool):
        request = struct.pack(">B?", Code.SET_PWN_AUTOSCALE, enable)
        self._socket.write(request)
        self._read_packet(Code.SET_PWN_AUTOSCALE)

    @property
    def pwm_frequency(self) -> str:
        request = struct.pack(">B", Code.GET_PWM_FREQUENCY)
        self._socket.write(request)
        return PWM_FREQUENCIES[self._read_packet(Code.GET_PWM_FREQUENCY, "B")]

    @pwm_frequency.setter
    def pwm_frequency(self, frequency):
        frequency = PWM_FREQUENCIES.index(frequency.strip().lower())
        request = struct.pack(">BB", Code.SET_PWM_FREQUENCY, frequency)
        self._socket.write(request)
        self._read_packet(Code.SET_PWM_FREQUENCY)

    @property
    def pwm_gradient(self) -> int:
        request = struct.pack(">B", Code.GET_PWM_GRADIENT)
        self._socket.write(request)
        return self._read_packet(Code.GET_PWM_GRADIENT, "B")

    @pwm_gradient.setter
    def pwm_gradient(self, gradient: int):
        request = struct.pack(">BB", Code.SET_PWM_GRADIENT, gradient)
        self._socket.write(request)
        self._read_packet(Code.SET_PWM_GRADIENT)

    @property
    def pwm_offset(self) -> int:
        request = struct.pack(">B", Code.GET_PWM_OFFSET)
        self._socket.write(request)
        return self._read_packet(Code.GET_PWM_OFFSET, "B")

    @pwm_offset.setter
    def pwm_offset(self, offset: int):
        request = struct.pack(">BB", Code.SET_PWM_OFFSET, offset)
        self._socket.write(request)
        self._read_packet(Code.SET_PWM_OFFSET)

    @property
    def charge_pump_undervoltage(self) -> bool:
        request = struct.pack(">B", Code.GET_CHARGE_PUMP_UNDERVOLTAGE)
        self._socket.write(request)
        return self._read_packet(Code.GET_CHARGE_PUMP_UNDERVOLTAGE, "?")

    @property
    def driver_error(self) -> bool:
        request = struct.pack(">B", Code.GET_DRIVER_ERROR)
        self._socket.write(request)
        return self._read_packet(Code.GET_DRIVER_ERROR, "?")

    @property
    def is_reset(self) -> bool:
        request = struct.pack(">B", Code.GET_IS_RESET)
        self._socket.write(request)
        return self._read_packet(Code.GET_IS_RESET, "?")

    @property
    def transmission_count(self) -> int:
        request = struct.pack(">B", Code.GET_TRANSMISSION_COUNT)
        self._socket.write(request)
        return self._read_packet(Code.GET_TRANSMISSION_COUNT, "B")

    @property
    def direction_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_DIRECTION_PIN)
        self._socket.write(request)
        return self._read_packet(Code.GET_DIRECTION_PIN, "?")

    @property
    def disable_pwm_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_DISABLE_PWM_PIN)
        self._socket.write(request)
        return self._read_packet(Code.GET_DISABLE_PWM_PIN, "?")

    @property
    def step_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_STEP_PIN)
        self._socket.write(request)
        return self._read_packet(Code.GET_STEP_PIN, "?")

    @property
    def powerdown_uart_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_POWERDOWN_UART_PIN)
        self._socket.write(request)
        return self._read_packet(Code.GET_POWERDOWN_UART_PIN, "?")

    @property
    def diagnostic_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_DIAGNOSTIC_PIN)
        self._socket.write(request)
        return self._read_packet(Code.GET_DIAGNOSTIC_PIN, "?")

    @property
    def microstep2_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_MICROSTEP2_PIN)
        self._socket.write(request)
        return self._read_packet(Code.GET_MICROSTEP2_PIN, "?")

    @property
    def microstep1_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_MICROSTEP1_PIN)
        self._socket.write(request)
        return self._read_packet(Code.GET_MICROSTEP1_PIN, "?")

    @property
    def disable_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_DISABLE_PIN)
        self._socket.write(request)
        return self._read_packet(Code.GET_DISABLE_PIN, "?")

    @property
    def microstep_time(self) -> int:
        request = struct.pack(">B", Code.GET_MICROSTEP_TIME)
        self._socket.write(request)
        return self._read_packet(Code.GET_MICROSTEP_TIME, "I")

    @property
    def motor_load(self) -> int:
        request = struct.pack(">B", Code.GET_MOTOR_LOAD)
        self._socket.write(request)
        return self._read_packet(Code.GET_MOTOR_LOAD, "H")

    @property
    def microstep_position(self) -> int:
        request = struct.pack(">B", Code.GET_MICROSTEP_POSITION)
        self._socket.write(request)
        return self._read_packet(Code.GET_MICROSTEP_POSITION, "H")

    @property
    def microstep_current(self) -> tuple[int, int]:
        request = struct.pack(">B", Code.GET_MICROSTEP_CURRENT)
        self._socket.write(request)
        return self._read_packet(Code.GET_MICROSTEP_CURRENT, "hh")

    @property
    def stopped(self) -> bool:
        request = struct.pack(">B", Code.GET_STOPPED)
        self._socket.write(request)
        return self._read_packet(Code.GET_STOPPED, "?")

    @property
    def pwm_mode(self) -> bool:
        request = struct.pack(">B", Code.GET_PWM_MODE)
        self._socket.write(request)
        return self._read_packet(Code.GET_PWM_MODE, "?")

    @property
    def current_scale(self) -> int:
        request = struct.pack(">B", Code.GET_CURRENT_SCALE)
        self._socket.write(request)
        return self._read_packet(Code.GET_CURRENT_SCALE, "B")

    @property
    def driver_temperature(self) -> str:
        request = struct.pack(">B", Code.GET_TEMPERATURE)
        self._socket.write(request)
        return TEMPERATURE_THRESHOLDS[self._read_packet(Code.GET_TEMPERATURE, "B")]

    @property
    def open_load(self) -> str:
        request = struct.pack(">B", Code.GET_OPEN_LOAD)
        self._socket.write(request)
        return PHASE_STATUSES[self._read_packet(Code.GET_OPEN_LOAD, "B")]

    @property
    def low_side_short(self) -> str:
        request = struct.pack(">B", Code.GET_LOW_SIDE_SHORT)
        self._socket.write(request)
        return PHASE_STATUSES[self._read_packet(Code.GET_LOW_SIDE_SHORT, "B")]

    @property
    def ground_short(self) -> str:
        request = struct.pack(">B", Code.GET_GROUND_SHORT)
        self._socket.write(request)
        return PHASE_STATUSES[self._read_packet(Code.GET_GROUND_SHORT, "B")]

    @property
    def overtemperature(self) -> str:
        request = struct.pack(">B", Code.GET_OVERTEMPERATURE)
        self._socket.write(request)
        return OVERTEMPERATURE_STATUSES[self._read_packet(Code.GET_OVERTEMPERATURE, "B")]

    def _read_packet(self, assert_code: Code, response_format: str = "") -> Any:
        response = self._socket.read()
        code = struct.unpack(">B", response[:1])[0]
        if code == Code.FAIL:
            raise RuntimeError("Device reported failure.")
        elif code != assert_code:
            raise RuntimeError(f"Expected {assert_code} got {code=}.")

        payload = struct.unpack(">" + response_format, response[1:])
        if len(payload) == 1:
            return payload[0]
        else:
            return payload


class Sipper:
    def __init__(
        self,
        name,
        cnc_port,
        pump_port,
        valve_port,
        a1_pos,
        h12_pos,
        z_bottom,
        mapping=None,
        x_max=7500,
        y_max=6000,
        z_max=3000,
    ):
        self.name = name
        self._cnc_socket = serial.Serial(cnc_port, baudrate=9600, timeout=30)
        self._valve_socket = serial.Serial(valve_port, baudrate=9600, timeout=1)
        self._pump_socket = serial.Serial(pump_port, baudrate=9600, timeout=1)
        self._xyz = np.zeros(3)
        self._max = np.array([x_max, y_max, z_max])
        self._valve = 1
        self._frequency = 0
        self._voltage = 0
        self._a1 = a1_pos
        self._h12 = h12_pos
        self._z_bottom = z_bottom
        self._well = "none"
        self._mapping = mapping if mapping is not None else {}

        time.sleep(0.2)
        self._cnc_socket.reset_input_buffer()
        self._valve_socket.reset_input_buffer()
        self._pump_socket.reset_input_buffer()
        self.home()

    def pause(self):
        self.voltage = 0
        self.frequency = 0

    @property
    def xyz(self):
        return self._xyz

    @xyz.setter
    def xyz(self, value):
        value = np.array(value)
        if np.any(value < 0) or np.any(value > 1):
            raise ValueError("Coordinates must be in the range [0, 1].")
        with atomic_msg(self._cnc_socket, sleep_time=30):
            msg = struct.pack("<BIII", 1, *(value * self._max).astype(np.uint32))
            self._cnc_socket.write(msg)
            self._cnc_socket.read(1)
        self._xyz = value

    @property
    def x(self):
        return self._xyz[0]

    @x.setter
    def x(self, value):
        self.xyz = (value, self._xyz[1], self._xyz[2])

    @property
    def y(self):
        return self._xyz[1]

    @y.setter
    def y(self, value):
        self.xyz = (self._xyz[0], value, self._xyz[2])

    @property
    def z(self):
        return self._xyz[2]

    @z.setter
    def z(self, value):
        self.xyz = (self._xyz[0], self._xyz[1], value)

    def home(self):
        self._well = "none"
        self.pause()
        self._xyz = np.zeros(3)
        with atomic_msg(self._cnc_socket, sleep_time=30):
            msg = struct.pack("<B", 0)
            self._cnc_socket.write(msg)
            self._cnc_socket.read(1)

    @property
    def flow_rate(self):
        return self.sensor_info()[0]

    @property
    def temperature(self):
        return self.sensor_info()[1]

    @property
    def air(self):
        return (self.sensor_info()[2] & 0x01) == 1

    def sensor_info(self):
        with atomic_msg(self._valve_socket):
            msg = struct.pack("<B", 2)
            self._valve_socket.write(msg)
            error = self._valve_socket.read(1)
            if error != b"\x00":
                raise RuntimeError(read_byte_str(self._valve_socket))
            flow_rate, temperature, flags = struct.unpack("<ffH", self._valve_socket.read(10))
        return flow_rate, temperature, flags

    @property
    def valve(self):
        return self._valve

    @valve.setter
    def valve(self, value):
        value = 1 if value in [1, "open"] else 0
        with atomic_msg(self._valve_socket):
            msg = struct.pack("<B", value)
            self._valve_socket.write(msg)
            err = self._valve_socket.read(1)
            if err != b"\x00":
                raise RuntimeError(f"Failed to toggle the valve expected response 0 got {err}.")
        self._valve = value

    @property
    def frequency(self):
        return self._frequency

    @frequency.setter
    def frequency(self, value):
        self.set_pump(self._voltage, value)
        self._frequency = value

    @property
    def voltage(self):
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        self.set_pump(value, self._frequency)
        self._voltage = value

    def set_pump(self, voltage, frequency):
        with atomic_msg(self._pump_socket):
            self._pump_socket.write(struct.pack("<HB", frequency, voltage))
            err = self._pump_socket.read(1)
            if err != b"\x00":
                raise RuntimeError(f"Failed to set the pump expected response 0 got {err}.")

    @property
    def well(self):
        return self._well

    @well.setter
    def well(self, new_well):
        if new_well == "none":
            self.z = 0
            self._well = "none"
            return

        pos = self._mapping.get(new_well, new_well)

        if not re.fullmatch(r"[A-H]1?[0-9]", pos) or int(pos[1:]) > 12:
            raise ValueError("Plate position must be in the format [A-Z][1-12].")

        col = int(pos[1:]) - 1
        if self._a1[0] < self._h12[0]:
            x = self._a1 + col * (self._h12[0] - self._a1[0]) / 11
        else:
            col = 11 - col
            x = self._h12[0] + col * (self._a1[0] - self._h12[0]) / 11

        row = ord(pos[0]) - ord("A")
        if self._a1[1] < self._h12[1]:
            y = self._a1[1] + row * (self._h12[1] - self._a1[1]) / 7
        else:
            row = 7 - row
            y = self._h12[1] + row * (self._a1[1] - self._h12[1]) / 7

        self.z = 0
        self.xyz = (x, y, 0)
        self.z = self._z_bottom
        self._well = new_well

    def flow(self, well):
        self.pause()
        self.well = well
        self.valve = "closed"
        self.frequency = 200
        self.voltage = 250

    def purge(self, well):
        self.pause()
        self.well = well
        self.valve = "open"
        self.frequency = 200
        self.voltage = 250


@contextlib.contextmanager
def atomic_msg(socket, sleep_time=0.2):
    try:
        yield
    except Exception as e:
        time.sleep(sleep_time)
        socket.reset_input_buffer()
        raise e


def read_byte_str(socket):
    received_bytes = bytearray()
    while True:
        byte = socket.read(1)
        if not byte or byte == b"\x00":
            break
        received_bytes.extend(byte)
    return received_bytes.decode("ascii")
