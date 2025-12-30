import contextlib
import re
import struct
import time
from dataclasses import dataclass
from enum import IntEnum

import numpy as np
import serial

from . import packet


class Code(IntEnum):
    INIT = 0x00
    FLOW_SENSOR_INFO = 0x01
    SET_PUMP_RPM = 0x02
    FAIL = 0xFF


@dataclass
class SensorInfo:
    air: bool
    high_flow: bool
    exp_smoothing: bool
    ul_per_min: float
    degrees_c: float


class FlowController:
    def __init__(self, name):
        self.name = name
        self._socket = packet.PacketStream()

    def set_rpm(self, rpm: float):
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

    def sensor_info(self) -> SensorInfo:
        request = struct.pack(">B", Code.FLOW_SENSOR_INFO)
        self._socket.write(request)
        return self._read_packet(assert_code=Code.FLOW_SENSOR_INFO)

    def _read_packet(self, assert_code=None):
        response = self._socket.read()
        code = struct.unpack(">B", response[:1])[0]
        if assert_code is not None and code != assert_code:
            raise RuntimeError(f"Expected {assert_code} got {code=}.")

        match code:
            case Code.FLOW_SENSOR_INFO:
                return SensorInfo(*struct.unpack(">???dd", response[1:]))
            case Code.SET_PUMP_RPM:
                return None
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
