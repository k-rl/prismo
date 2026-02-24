from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class Camera(Protocol):
    exposure: float

    def snap(self) -> np.ndarray: ...


@runtime_checkable
class Close(Protocol):
    def close(self): ...


@runtime_checkable
class Focus(Protocol):
    z: float


@runtime_checkable
class Stage(Protocol):
    x: float
    y: float
    xy: tuple[float, float]


@runtime_checkable
class State(Protocol):
    state: str | int | float


@runtime_checkable
class Valved(Protocol):
    name: str
    valves: dict[str, str | int]
    valve_states: dict[str, list[str | int]]

    def __setitem__(self, key: str, state: str | int): ...


@runtime_checkable
class Wait(Protocol):
    def wait(self): ...


@runtime_checkable
class Zoom(Protocol):
    zoom: float
