import contextlib
import re
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from numbers import Number

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
    GET_COOLSTEP_THRESHOLD = 0x1E
    SET_COOLSTEP_THRESHOLD = 0x1F
    GET_STALLGUARD_THRESHOLD = 0x20
    SET_STALLGUARD_THRESHOLD = 0x21
    GET_COOLSTEP_LOWER_MIN_CURRENT = 0x22
    SET_COOLSTEP_LOWER_MIN_CURRENT = 0x23
    GET_COOLSTEP_CURRENT_DOWNSTEP_RATE = 0x24
    SET_COOLSTEP_CURRENT_DOWNSTEP_RATE = 0x25
    GET_STALLGUARD_HYSTERESIS = 0x26
    SET_STALLGUARD_HYSTERESIS = 0x27
    GET_CURRENT_UPSTEP = 0x28
    SET_CURRENT_UPSTEP = 0x29
    GET_COOLSTEP_STALLGUARD_THRESHOLD = 0x2A
    SET_COOLSTEP_STALLGUARD_THRESHOLD = 0x2B
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
    GET_INVERT_DIRECTION = 0x48
    SET_INVERT_DIRECTION = 0x49
    GET_VELOCITY = 0x4A
    SET_VELOCITY = 0x4B
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


class IndexOutput(IntEnum):
    PERIOD = 0
    OVERTEMPERATURE = 1
    MICROSTEP = 2


class BlankTime(IntEnum):
    B16 = 0
    B24 = 1
    B36 = 2
    B54 = 3


class PwmFrequency(IntEnum):
    DIV1024 = 0
    DIV683 = 1
    DIV512 = 2
    DIV410 = 3


class PhaseStatus(IntEnum):
    NONE = 0
    PHASE_A = 1
    PHASE_B = 2
    BOTH_PHASES = 3


class TemperatureThreshold(IntEnum):
    NORMAL = 0
    TEMP_120C = 1
    TEMP_143C = 3
    TEMP_150C = 7
    TEMP_157C = 15


class OvertemperatureStatus(IntEnum):
    NORMAL = 0
    WARNING = 1
    SHUTDOWN = 3


STOP_CODES = ["normal", "freewheel", "low_side", "high_side"]


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

    def set_rpm(self, rpm: float):
        self.rpm = rpm

    @property
    def rpm(self) -> float:
        request = struct.pack(">B", Code.GET_PUMP_RPM)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_PUMP_RPM)

    @rpm.setter
    def rpm(self, rpm: float):
        request = struct.pack(">Bd", Code.SET_PUMP_RPM, rpm)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_PUMP_RPM)

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
        return self._read_packet(assert_code=Code.GET_PUMP_RPM)

    @rpm.setter
    def rpm(self, rpm: float):
        request = struct.pack(">Bd", Code.SET_PUMP_RPM, rpm)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_PUMP_RPM)

    def sensor_info(self) -> SensorInfo:
        request = struct.pack(">B", Code.FLOW_SENSOR_INFO)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.FLOW_SENSOR_INFO)

    @property
    def rms_amps(self) -> float:
        request = struct.pack(">B", Code.GET_RMS_AMPS)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_RMS_AMPS)

    @rms_amps.setter
    def rms_amps(self, amps: Number):
        request = struct.pack(">Bd", Code.SET_RMS_AMPS, amps)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_RMS_AMPS)

    @property
    def stop_rms_amps(self) -> float:
        request = struct.pack(">B", Code.GET_STOP_RMS_AMPS)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_STOP_RMS_AMPS)

    @stop_rms_amps.setter
    def stop_rms_amps(self, amps: Number):
        request = struct.pack(">Bd", Code.SET_STOP_RMS_AMPS, amps)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_STOP_RMS_AMPS)

    @property
    def stop_mode(self) -> str:
        request = struct.pack(">B", Code.GET_STOP_MODE)
        self._socket.write(request)
        return STOP_CODES[self._read_packet(assert_code=Code.GET_STOP_MODE)]

    @stop_mode.setter
    def stop_mode(self, mode: str):
        request = struct.pack(">BB", Code.SET_STOP_MODE, STOP_CODES.index(mode))
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_STOP_MODE)

    @property
    def powerdown_duration_s(self) -> float:
        request = struct.pack(">B", Code.GET_POWERDOWN_DURATION_S)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_POWERDOWN_DURATION_S)

    @powerdown_duration_s.setter
    def powerdown_duration_s(self, duration: float):
        request = struct.pack(">Bd", Code.SET_POWERDOWN_DURATION_S, duration)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_POWERDOWN_DURATION_S)

    @property
    def powerdown_delay_s(self) -> float:
        request = struct.pack(">B", Code.GET_POWERDOWN_DELAY_S)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_POWERDOWN_DELAY_S)

    @powerdown_delay_s.setter
    def powerdown_delay_s(self, delay: float):
        request = struct.pack(">Bd", Code.SET_POWERDOWN_DELAY_S, delay)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_POWERDOWN_DELAY_S)

    @property
    def microsteps(self) -> int:
        request = struct.pack(">B", Code.GET_MICROSTEPS)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_MICROSTEPS)

    @microsteps.setter
    def microsteps(self, microsteps: int):
        request = struct.pack(">BH", Code.SET_MICROSTEPS, microsteps)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_MICROSTEPS)

    @property
    def filter_step_pulses(self) -> bool:
        request = struct.pack(">B", Code.GET_FILTER_STEP_PULSES)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_FILTER_STEP_PULSES)

    @filter_step_pulses.setter
    def filter_step_pulses(self, enable: bool):
        request = struct.pack(">B?", Code.SET_FILTER_STEP_PULSES, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_FILTER_STEP_PULSES)

    @property
    def double_edge_step(self) -> bool:
        request = struct.pack(">B", Code.GET_DOUBLE_EDGE_STEP)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_DOUBLE_EDGE_STEP)

    @double_edge_step.setter
    def double_edge_step(self, enable: bool):
        request = struct.pack(">B?", Code.SET_DOUBLE_EDGE_STEP, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_DOUBLE_EDGE_STEP)

    @property
    def interpolate_microsteps(self) -> bool:
        request = struct.pack(">B", Code.GET_INTERPOLATE_MICROSTEPS)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_INTERPOLATE_MICROSTEPS)

    @interpolate_microsteps.setter
    def interpolate_microsteps(self, enable: bool):
        request = struct.pack(">B?", Code.SET_INTERPOLATE_MICROSTEPS, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_INTERPOLATE_MICROSTEPS)

    @property
    def coolstep_threshold(self) -> int:
        request = struct.pack(">B", Code.GET_COOLSTEP_THRESHOLD)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_COOLSTEP_THRESHOLD)

    @coolstep_threshold.setter
    def coolstep_threshold(self, threshold: int):
        request = struct.pack(">BI", Code.SET_COOLSTEP_THRESHOLD, threshold)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_COOLSTEP_THRESHOLD)

    @property
    def stallguard_threshold(self) -> int:
        request = struct.pack(">B", Code.GET_STALLGUARD_THRESHOLD)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_STALLGUARD_THRESHOLD)

    @stallguard_threshold.setter
    def stallguard_threshold(self, threshold: int):
        request = struct.pack(">BB", Code.SET_STALLGUARD_THRESHOLD, threshold)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_STALLGUARD_THRESHOLD)

    @property
    def coolstep_lower_min_current(self) -> bool:
        request = struct.pack(">B", Code.GET_COOLSTEP_LOWER_MIN_CURRENT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_COOLSTEP_LOWER_MIN_CURRENT)

    @coolstep_lower_min_current.setter
    def coolstep_lower_min_current(self, enable: bool):
        request = struct.pack(">B?", Code.SET_COOLSTEP_LOWER_MIN_CURRENT, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_COOLSTEP_LOWER_MIN_CURRENT)

    @property
    def coolstep_current_downstep_rate(self) -> int:
        request = struct.pack(">B", Code.GET_COOLSTEP_CURRENT_DOWNSTEP_RATE)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_COOLSTEP_CURRENT_DOWNSTEP_RATE)

    @coolstep_current_downstep_rate.setter
    def coolstep_current_downstep_rate(self, rate: int):
        request = struct.pack(">BB", Code.SET_COOLSTEP_CURRENT_DOWNSTEP_RATE, rate)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_COOLSTEP_CURRENT_DOWNSTEP_RATE)

    @property
    def stallguard_hysteresis(self) -> int:
        request = struct.pack(">B", Code.GET_STALLGUARD_HYSTERESIS)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_STALLGUARD_HYSTERESIS)

    @stallguard_hysteresis.setter
    def stallguard_hysteresis(self, hysteresis: int):
        request = struct.pack(">BB", Code.SET_STALLGUARD_HYSTERESIS, hysteresis)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_STALLGUARD_HYSTERESIS)

    @property
    def current_upstep(self) -> int:
        request = struct.pack(">B", Code.GET_CURRENT_UPSTEP)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_CURRENT_UPSTEP)

    @current_upstep.setter
    def current_upstep(self, upstep: int):
        request = struct.pack(">BB", Code.SET_CURRENT_UPSTEP, upstep)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_CURRENT_UPSTEP)

    @property
    def coolstep_stallguard_threshold(self) -> int:
        request = struct.pack(">B", Code.GET_COOLSTEP_STALLGUARD_THRESHOLD)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_COOLSTEP_STALLGUARD_THRESHOLD)

    @coolstep_stallguard_threshold.setter
    def coolstep_stallguard_threshold(self, threshold: int):
        request = struct.pack(">BB", Code.SET_COOLSTEP_STALLGUARD_THRESHOLD, threshold)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_COOLSTEP_STALLGUARD_THRESHOLD)

    @property
    def short_supply_protect(self) -> bool:
        request = struct.pack(">B", Code.GET_SHORT_SUPPLY_PROTECT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_SHORT_SUPPLY_PROTECT)

    @short_supply_protect.setter
    def short_supply_protect(self, enable: bool):
        request = struct.pack(">B?", Code.SET_SHORT_SUPPLY_PROTECT, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_SHORT_SUPPLY_PROTECT)

    @property
    def short_ground_protect(self) -> bool:
        request = struct.pack(">B", Code.GET_SHORT_GROUND_PROTECT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_SHORT_GROUND_PROTECT)

    @short_ground_protect.setter
    def short_ground_protect(self, enable: bool):
        request = struct.pack(">B?", Code.SET_SHORT_GROUND_PROTECT, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_SHORT_GROUND_PROTECT)

    @property
    def blank_time(self) -> str:
        request = struct.pack(">B", Code.GET_BLANK_TIME)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_BLANK_TIME)

    @blank_time.setter
    def blank_time(self, time_value):
        if isinstance(time_value, str):
            time_value = BlankTime[time_value.strip().upper()]
        else:
            time_value = BlankTime(time_value)
        request = struct.pack(">BB", Code.SET_BLANK_TIME, int(time_value))
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_BLANK_TIME)

    @property
    def hysteresis_end(self) -> int:
        request = struct.pack(">B", Code.GET_HYSTERESIS_END)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_HYSTERESIS_END)

    @hysteresis_end.setter
    def hysteresis_end(self, end: int):
        request = struct.pack(">Bb", Code.SET_HYSTERESIS_END, end)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_HYSTERESIS_END)

    @property
    def hysteresis_start(self) -> int:
        request = struct.pack(">B", Code.GET_HYSTERESIS_START)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_HYSTERESIS_START)

    @hysteresis_start.setter
    def hysteresis_start(self, start: int):
        request = struct.pack(">BB", Code.SET_HYSTERESIS_START, start)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_HYSTERESIS_START)

    @property
    def decay_time(self) -> int:
        request = struct.pack(">B", Code.GET_DECAY_TIME)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_DECAY_TIME)

    @decay_time.setter
    def decay_time(self, time_value: int):
        request = struct.pack(">BB", Code.SET_DECAY_TIME, time_value)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_DECAY_TIME)

    @property
    def pwm_max_rpm(self) -> float:
        request = struct.pack(">B", Code.GET_PWM_MAX_RPM)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_PWM_MAX_RPM)

    @pwm_max_rpm.setter
    def pwm_max_rpm(self, rpm: float):
        request = struct.pack(">Bd", Code.SET_PWM_MAX_RPM, rpm)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_PWM_MAX_RPM)

    @property
    def driver_switch_autoscale_limit(self) -> int:
        request = struct.pack(">B", Code.GET_DRIVER_SWITCH_AUTOSCALE_LIMIT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_DRIVER_SWITCH_AUTOSCALE_LIMIT)

    @driver_switch_autoscale_limit.setter
    def driver_switch_autoscale_limit(self, limit: int):
        request = struct.pack(">BB", Code.SET_DRIVER_SWITCH_AUTOSCALE_LIMIT, limit)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_DRIVER_SWITCH_AUTOSCALE_LIMIT)

    @property
    def max_amplitude_change(self) -> int:
        request = struct.pack(">B", Code.GET_MAX_AMPLITUDE_CHANGE)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_MAX_AMPLITUDE_CHANGE)

    @max_amplitude_change.setter
    def max_amplitude_change(self, change: int):
        request = struct.pack(">BB", Code.SET_MAX_AMPLITUDE_CHANGE, change)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_MAX_AMPLITUDE_CHANGE)

    @property
    def pwm_autogradient(self) -> bool:
        request = struct.pack(">B", Code.GET_PWM_AUTOGRADIENT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_PWM_AUTOGRADIENT)

    @pwm_autogradient.setter
    def pwm_autogradient(self, enable: bool):
        request = struct.pack(">B?", Code.SET_PWM_AUTOGRADIENT, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_PWM_AUTOGRADIENT)

    @property
    def pwn_autoscale(self) -> bool:
        request = struct.pack(">B", Code.GET_PWN_AUTOSCALE)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_PWN_AUTOSCALE)

    @pwn_autoscale.setter
    def pwn_autoscale(self, enable: bool):
        request = struct.pack(">B?", Code.SET_PWN_AUTOSCALE, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_PWN_AUTOSCALE)

    @property
    def pwm_frequency(self) -> str:
        request = struct.pack(">B", Code.GET_PWM_FREQUENCY)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_PWM_FREQUENCY)

    @pwm_frequency.setter
    def pwm_frequency(self, frequency):
        if isinstance(frequency, str):
            frequency = PwmFrequency[frequency.strip().upper()]
        else:
            frequency = PwmFrequency(frequency)
        request = struct.pack(">BB", Code.SET_PWM_FREQUENCY, int(frequency))
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_PWM_FREQUENCY)

    @property
    def pwm_gradient(self) -> int:
        request = struct.pack(">B", Code.GET_PWM_GRADIENT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_PWM_GRADIENT)

    @pwm_gradient.setter
    def pwm_gradient(self, gradient: int):
        request = struct.pack(">BB", Code.SET_PWM_GRADIENT, gradient)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_PWM_GRADIENT)

    @property
    def pwm_offset(self) -> int:
        request = struct.pack(">B", Code.GET_PWM_OFFSET)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_PWM_OFFSET)

    @pwm_offset.setter
    def pwm_offset(self, offset: int):
        request = struct.pack(">BB", Code.SET_PWM_OFFSET, offset)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_PWM_OFFSET)

    @property
    def invert_direction(self) -> bool:
        request = struct.pack(">B", Code.GET_INVERT_DIRECTION)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_INVERT_DIRECTION)

    @invert_direction.setter
    def invert_direction(self, enable: bool):
        request = struct.pack(">B?", Code.SET_INVERT_DIRECTION, enable)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_INVERT_DIRECTION)

    @property
    def velocity(self) -> float:
        request = struct.pack(">B", Code.GET_VELOCITY)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_VELOCITY)

    @velocity.setter
    def velocity(self, rpm: float):
        request = struct.pack(">Bd", Code.SET_VELOCITY, rpm)
        self._socket.write(request)
        self._read_packet(assert_code=Code.SET_VELOCITY)

    @property
    def charge_pump_undervoltage(self) -> bool:
        request = struct.pack(">B", Code.GET_CHARGE_PUMP_UNDERVOLTAGE)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_CHARGE_PUMP_UNDERVOLTAGE)

    @property
    def driver_error(self) -> bool:
        request = struct.pack(">B", Code.GET_DRIVER_ERROR)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_DRIVER_ERROR)

    @property
    def is_reset(self) -> bool:
        request = struct.pack(">B", Code.GET_IS_RESET)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_IS_RESET)

    @property
    def transmission_count(self) -> int:
        request = struct.pack(">B", Code.GET_TRANSMISSION_COUNT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_TRANSMISSION_COUNT)

    @property
    def direction_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_DIRECTION_PIN)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_DIRECTION_PIN)

    @property
    def disable_pwm_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_DISABLE_PWM_PIN)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_DISABLE_PWM_PIN)

    @property
    def step_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_STEP_PIN)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_STEP_PIN)

    @property
    def powerdown_uart_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_POWERDOWN_UART_PIN)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_POWERDOWN_UART_PIN)

    @property
    def diagnostic_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_DIAGNOSTIC_PIN)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_DIAGNOSTIC_PIN)

    @property
    def microstep2_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_MICROSTEP2_PIN)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_MICROSTEP2_PIN)

    @property
    def microstep1_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_MICROSTEP1_PIN)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_MICROSTEP1_PIN)

    @property
    def disable_pin(self) -> bool:
        request = struct.pack(">B", Code.GET_DISABLE_PIN)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_DISABLE_PIN)

    @property
    def microstep_time(self) -> int:
        request = struct.pack(">B", Code.GET_MICROSTEP_TIME)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_MICROSTEP_TIME)

    @property
    def motor_load(self) -> int:
        request = struct.pack(">B", Code.GET_MOTOR_LOAD)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_MOTOR_LOAD)

    @property
    def microstep_position(self) -> int:
        request = struct.pack(">B", Code.GET_MICROSTEP_POSITION)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_MICROSTEP_POSITION)

    @property
    def microstep_current(self) -> tuple[int, int]:
        request = struct.pack(">B", Code.GET_MICROSTEP_CURRENT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_MICROSTEP_CURRENT)

    @property
    def stopped(self) -> bool:
        request = struct.pack(">B", Code.GET_STOPPED)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_STOPPED)

    @property
    def pwm_mode(self) -> bool:
        request = struct.pack(">B", Code.GET_PWM_MODE)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_PWM_MODE)

    @property
    def current_scale(self) -> int:
        request = struct.pack(">B", Code.GET_CURRENT_SCALE)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_CURRENT_SCALE)

    @property
    def driver_temperature(self) -> str:
        request = struct.pack(">B", Code.GET_TEMPERATURE)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_TEMPERATURE)

    @property
    def open_load(self) -> str:
        request = struct.pack(">B", Code.GET_OPEN_LOAD)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_OPEN_LOAD)

    @property
    def low_side_short(self) -> str:
        request = struct.pack(">B", Code.GET_LOW_SIDE_SHORT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_LOW_SIDE_SHORT)

    @property
    def ground_short(self) -> str:
        request = struct.pack(">B", Code.GET_GROUND_SHORT)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_GROUND_SHORT)

    @property
    def overtemperature(self) -> str:
        request = struct.pack(">B", Code.GET_OVERTEMPERATURE)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.GET_OVERTEMPERATURE)

    def _read_packet(self, assert_code=None):
        response = self._socket.read()
        code = struct.unpack(">B", response[:1])[0]
        if code == Code.FAIL:
            raise RuntimeError("Device reported failure.")
        if assert_code is not None and code != assert_code:
            raise RuntimeError(f"Expected {assert_code} got {code=}.")

        payload = response[1:]
        match code:
            case Code.FLOW_SENSOR_INFO:
                return SensorInfo(*struct.unpack(">???dd", payload))
            case Code.GET_PUMP_RPM:
                return struct.unpack(">d", payload)[0]
            case Code.GET_RMS_AMPS:
                return struct.unpack(">d", payload)[0]
            case Code.GET_STOP_RMS_AMPS:
                return struct.unpack(">d", payload)[0]
            case Code.GET_STOP_MODE:
                return struct.unpack(">B", payload)[0]
            case Code.GET_POWERDOWN_DURATION_S:
                return struct.unpack(">d", payload)[0]
            case Code.GET_POWERDOWN_DELAY_S:
                return struct.unpack(">d", payload)[0]
            case Code.GET_MICROSTEPS:
                return struct.unpack(">H", payload)[0]
            case Code.GET_FILTER_STEP_PULSES:
                return struct.unpack(">?", payload)[0]
            case Code.GET_DOUBLE_EDGE_STEP:
                return struct.unpack(">?", payload)[0]
            case Code.GET_INTERPOLATE_MICROSTEPS:
                return struct.unpack(">?", payload)[0]
            case Code.GET_COOLSTEP_THRESHOLD:
                return struct.unpack(">I", payload)[0]
            case Code.GET_STALLGUARD_THRESHOLD:
                return struct.unpack(">B", payload)[0]
            case Code.GET_COOLSTEP_LOWER_MIN_CURRENT:
                return struct.unpack(">?", payload)[0]
            case Code.GET_COOLSTEP_CURRENT_DOWNSTEP_RATE:
                return struct.unpack(">B", payload)[0]
            case Code.GET_STALLGUARD_HYSTERESIS:
                return struct.unpack(">B", payload)[0]
            case Code.GET_CURRENT_UPSTEP:
                return struct.unpack(">B", payload)[0]
            case Code.GET_COOLSTEP_STALLGUARD_THRESHOLD:
                return struct.unpack(">B", payload)[0]
            case Code.GET_SHORT_SUPPLY_PROTECT:
                return struct.unpack(">?", payload)[0]
            case Code.GET_SHORT_GROUND_PROTECT:
                return struct.unpack(">?", payload)[0]
            case Code.GET_BLANK_TIME:
                return BlankTime(struct.unpack(">B", payload)[0]).name.lower()
            case Code.GET_HYSTERESIS_END:
                return struct.unpack(">b", payload)[0]
            case Code.GET_HYSTERESIS_START:
                return struct.unpack(">B", payload)[0]
            case Code.GET_DECAY_TIME:
                return struct.unpack(">B", payload)[0]
            case Code.GET_PWM_MAX_RPM:
                return struct.unpack(">d", payload)[0]
            case Code.GET_DRIVER_SWITCH_AUTOSCALE_LIMIT:
                return struct.unpack(">B", payload)[0]
            case Code.GET_MAX_AMPLITUDE_CHANGE:
                return struct.unpack(">B", payload)[0]
            case Code.GET_PWM_AUTOGRADIENT:
                return struct.unpack(">?", payload)[0]
            case Code.GET_PWN_AUTOSCALE:
                return struct.unpack(">?", payload)[0]
            case Code.GET_PWM_FREQUENCY:
                return PwmFrequency(struct.unpack(">B", payload)[0]).name.lower()
            case Code.GET_PWM_GRADIENT:
                return struct.unpack(">B", payload)[0]
            case Code.GET_PWM_OFFSET:
                return struct.unpack(">B", payload)[0]
            case Code.GET_INVERT_DIRECTION:
                return struct.unpack(">?", payload)[0]
            case Code.GET_VELOCITY:
                return struct.unpack(">d", payload)[0]
            case Code.GET_CHARGE_PUMP_UNDERVOLTAGE:
                return struct.unpack(">?", payload)[0]
            case Code.GET_DRIVER_ERROR:
                return struct.unpack(">?", payload)[0]
            case Code.GET_IS_RESET:
                return struct.unpack(">?", payload)[0]
            case Code.GET_TRANSMISSION_COUNT:
                return struct.unpack(">B", payload)[0]
            case Code.GET_DIRECTION_PIN:
                return struct.unpack(">?", payload)[0]
            case Code.GET_DISABLE_PWM_PIN:
                return struct.unpack(">?", payload)[0]
            case Code.GET_STEP_PIN:
                return struct.unpack(">?", payload)[0]
            case Code.GET_POWERDOWN_UART_PIN:
                return struct.unpack(">?", payload)[0]
            case Code.GET_DIAGNOSTIC_PIN:
                return struct.unpack(">?", payload)[0]
            case Code.GET_MICROSTEP2_PIN:
                return struct.unpack(">?", payload)[0]
            case Code.GET_MICROSTEP1_PIN:
                return struct.unpack(">?", payload)[0]
            case Code.GET_DISABLE_PIN:
                return struct.unpack(">?", payload)[0]
            case Code.GET_MICROSTEP_TIME:
                return struct.unpack(">I", payload)[0]
            case Code.GET_MOTOR_LOAD:
                return struct.unpack(">H", payload)[0]
            case Code.GET_MICROSTEP_POSITION:
                return struct.unpack(">H", payload)[0]
            case Code.GET_MICROSTEP_CURRENT:
                return struct.unpack(">hh", payload)
            case Code.GET_STOPPED:
                return struct.unpack(">?", payload)[0]
            case Code.GET_PWM_MODE:
                return struct.unpack(">?", payload)[0]
            case Code.GET_CURRENT_SCALE:
                return struct.unpack(">B", payload)[0]
            case Code.GET_TEMPERATURE:
                return TemperatureThreshold(struct.unpack(">B", payload)[0]).name.lower()
            case Code.GET_OPEN_LOAD:
                return PhaseStatus(struct.unpack(">B", payload)[0]).name.lower()
            case Code.GET_LOW_SIDE_SHORT:
                return PhaseStatus(struct.unpack(">B", payload)[0]).name.lower()
            case Code.GET_GROUND_SHORT:
                return PhaseStatus(struct.unpack(">B", payload)[0]).name.lower()
            case Code.GET_OVERTEMPERATURE:
                return OvertemperatureStatus(struct.unpack(">B", payload)[0]).name.lower()
            case (
                Code.INIT
                | Code.SET_PUMP_RPM
                | Code.SET_RMS_AMPS
                | Code.SET_STOP_RMS_AMPS
                | Code.SET_STOP_MODE
                | Code.SET_POWERDOWN_DURATION_S
                | Code.SET_POWERDOWN_DELAY_S
                | Code.SET_MICROSTEPS
                | Code.SET_FILTER_STEP_PULSES
                | Code.SET_DOUBLE_EDGE_STEP
                | Code.SET_INTERPOLATE_MICROSTEPS
                | Code.SET_COOLSTEP_THRESHOLD
                | Code.SET_STALLGUARD_THRESHOLD
                | Code.SET_COOLSTEP_LOWER_MIN_CURRENT
                | Code.SET_COOLSTEP_CURRENT_DOWNSTEP_RATE
                | Code.SET_STALLGUARD_HYSTERESIS
                | Code.SET_CURRENT_UPSTEP
                | Code.SET_COOLSTEP_STALLGUARD_THRESHOLD
                | Code.SET_SHORT_SUPPLY_PROTECT
                | Code.SET_SHORT_GROUND_PROTECT
                | Code.SET_BLANK_TIME
                | Code.SET_HYSTERESIS_END
                | Code.SET_HYSTERESIS_START
                | Code.SET_DECAY_TIME
                | Code.SET_PWM_MAX_RPM
                | Code.SET_DRIVER_SWITCH_AUTOSCALE_LIMIT
                | Code.SET_MAX_AMPLITUDE_CHANGE
                | Code.SET_PWM_AUTOGRADIENT
                | Code.SET_PWN_AUTOSCALE
                | Code.SET_PWM_FREQUENCY
                | Code.SET_PWM_GRADIENT
                | Code.SET_PWM_OFFSET
                | Code.SET_INVERT_DIRECTION
                | Code.SET_VELOCITY
            ):
                return None
            case Code.GET_PUMP_RPM:
                return struct.unpack(">d", response[1:])[0]
            case _:
                raise RuntimeError(f"Unknown response {code=}.")


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
