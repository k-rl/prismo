from collections.abc import Iterator
from typing import Literal

import numpy as np
import pymodbus.client

from .. import utils


class ValveDriver:
    def __init__(self, name: str, ip: str, num_valves: int = 48):
        self.name = name
        self._num_valves = num_valves
        self._client = pymodbus.client.ModbusTcpClient(ip)
        self._client.connect()

    def __getitem__(self, idx: int) -> Literal["closed", "open"]:
        if idx < 0 or idx >= self._num_valves:
            raise IndexError(f"Invalid valve index {idx}.")
        addr = idx + 512
        return "open" if self._client.read_coils(addr).bits[0] else "closed"

    def __setitem__(self, idx: int | str, state: Literal["closed", "open"]):
        if isinstance(idx, str):
            idx = int(idx)
        if idx < 0 or idx >= self._num_valves:
            raise IndexError(f"Invalid valve index {idx}.")
        self._client.write_coil(idx, state == "open")

    def __len__(self) -> int:
        return self._num_valves

    def __iter__(self) -> Iterator[Literal["closed", "open"]]:
        for v in range(self._num_valves):
            yield self[v]

    @property
    def valves(self) -> dict[str, str | int]:
        return {str(i): self[i] for i in range(self._num_valves)}

    @property
    def valve_states(self) -> dict[str, list[str | int]]:
        return {str(i): ["closed", "open"] for i in range(self._num_valves)}


class Valves:
    def __init__(self, valves: int | list[int], driver: ValveDriver):
        self._valves = utils.to_list(valves)
        self._driver = driver

    def __eq__(self, other: object) -> bool:
        is_open = [self._driver[v] == "open" for v in self._valves]
        if other == "open":
            return all(is_open)
        elif other == "closed":
            return not any(is_open)
        else:
            return False

    def __getitem__(self, idx: int) -> Literal["closed", "open"]:
        return self._driver[self._valves[idx]]

    def __setitem__(self, idx: int, state: Literal["closed", "open"]):
        self._driver[self._valves[idx]] = state

    def __len__(self) -> int:
        return len(self._valves)

    def __repr__(self) -> str:
        # TODO: Revisit whether we want repr to be unambiguous.
        return ", ".join(f"'{self[k]}'" for k in range(len(self._valves)))


class TreeValves:
    def __init__(
        self,
        driver: ValveDriver,
        zeros: list[int],
        ones: list[int],
        states: dict[str, str],
    ):
        # Convert states to always be dicts.
        processed: dict[str | int, str]
        if not isinstance(states, dict):
            items = states if states is not None else list(range(2 ** len(zeros)))
            processed = {s: f"{i:0{len(zeros)}b}" for i, s in enumerate(items)}
        else:
            processed = states

        # Make sure we have all states where one path is open.
        for i in range(2 ** len(zeros)):
            if i not in processed:
                processed[i] = f"{i:0{len(zeros)}b}"
        # Check validity of state dict values.
        for v in processed.values():
            if len(v) != len(zeros):
                raise ValueError(f"Invalid state {v}. States must be of length {len(zeros)}.")
            elif not all(x in ["0", "1", "o", "x"] for x in v):
                raise ValueError(f"Invalid state {v}. States must only contain 0, 1, o, and x.")

        self._labels_to_states = processed
        self._states_to_labels = {value: key for key, value in processed.items()}
        self._zeros = zeros
        self._ones = ones
        self._all = zeros + ones
        self._driver = driver

    @property
    def state(self) -> str | int:
        zeros_open = np.array([self._driver[v] == "open" for v in self._zeros])
        ones_open = np.array([self._driver[v] == "open" for v in self._ones])
        all_open = np.array([self._driver[v] == "open" for v in self._all])
        if np.all(all_open):
            return "open"
        elif not np.any(all_open):
            return "closed"
        else:
            state_str = ""
            for i in range(len(zeros_open)):
                if zeros_open[i] and ones_open[i]:
                    state_str += "o"
                elif zeros_open[i]:
                    state_str += "0"
                elif ones_open[i]:
                    state_str += "1"
                else:
                    state_str += "x"

            if state_str in self._states_to_labels:
                return self._states_to_labels[state_str]
            else:
                return "invalid"

    @state.setter
    def state(self, label: str | int):
        if label == "open":
            for v in self._all:
                self._driver[v] = "open"
        elif label == "closed":
            for v in self._all:
                self._driver[v] = "closed"
        else:
            new_state = self._labels_to_states[label]
            for v in self._all:
                self._driver[v] = "closed"

            for i, c in enumerate(new_state):
                if c == "o":
                    self._driver[self._zeros[i]] = "open"
                    self._driver[self._ones[i]] = "open"
                elif c == "1":
                    self._driver[self._ones[i]] = "open"
                elif c == "0":
                    self._driver[self._zeros[i]] = "open"

    def __repr__(self) -> str:
        return str(self.state)


class Chip:
    def __init__(
        self,
        name: str,
        driver: ValveDriver,
        mapping: dict[str, dict[str | Literal[0, 1], list[int] | str] | list[int] | int],
    ):
        self._mapping: dict[str, TreeValves | Valves]
        processed: dict[str, TreeValves | Valves] = {}
        for k, v in mapping.items():
            if isinstance(v, dict):
                zeros = v.pop(0)
                ones = v.pop(1)
                if not isinstance(zeros, list) or not isinstance(ones, list):
                    raise ValueError("0 and 1 should be a list of integers in tree mapping.")
                processed[k] = TreeValves(zeros=zeros, ones=ones, states=v, driver=driver)
            else:
                processed[k] = Valves(valves=v, driver=driver)
        # We can't directly set self._mapping = mapping since our overriden __setattr__
        # depends on self._mapping being set likewise for name.
        super().__setattr__("_mapping", processed)
        super().__setattr__("name", name)

    def __getattr__(self, key: str) -> str | int | Valves:
        v = self._mapping[key]
        if not isinstance(v, Valves):
            return v.state
        else:
            return v

    def __setattr__(self, key: str, state: str | int):
        v = self._mapping[key]
        if not isinstance(v, Valves):
            v.state = state
        else:
            for i in range(len(v)):
                v[i] = state

    def __getitem__(self, key: str) -> Literal["closed", "open"] | Valves:
        return self.__getattr__(key)

    def __setitem__(self, key: str, state: Literal["closed", "open"]):
        self.__setattr__(key, state)

    def close_all(self):
        for v in self.valves:
            self[v] = "closed"

    def open_all(self):
        for v in self.valves:
            self[v] = "open"

    @property
    def valves(self) -> dict[str, str | int]:
        result: dict[str, str | int] = {}
        for k, v in self._mapping.items():
            if isinstance(v, TreeValves):
                result[k] = v.state
            else:
                result[k] = "closed" if v == "closed" else "open"
        return result

    @property
    def valve_states(self) -> dict[str, list[str | int]]:
        result: dict[str, list[str | int]] = {}
        for k, v in self._mapping.items():
            if isinstance(v, TreeValves):
                states: list[str | int] = ["closed", "open"]
                states.extend(v._labels_to_states.keys())
                result[k] = states
            else:
                result[k] = ["closed", "open"]
        return result
