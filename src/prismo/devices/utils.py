import re
from typing import Literal

from pymmcore import CMMCore
from serial.tools import list_ports


def load_port(
    core: CMMCore,
    vid: int,
    pid: int,
    port: str | None = None,
    timeout_ms: float = 500.0,
    baud_rate: int = 9600,
    dtr: bool = False,
    data_bits: int = 8,
    char_delay_ms: float = 0.0,
    fast_usb: bool = False,
    handshake: Literal["none", "software", "hardware"] = "none",
    parity: Literal["none", "even", "odd", "mark", "space"] = "none",
    stop_bits: Literal[1, 2] = 1,
    verbose: bool = True,
) -> str:
    # If an explicit port ID wasn't provided then just look for one that matches vid/pid.
    if port is None:
        for p in list_ports.comports():
            if p.vid == vid and p.pid == pid:
                port = p.device
                break

    if port is None:
        raise ValueError(f"No port found with {vid=} and {pid=}")

    # Only initialize the port if it hasn't already been loaded by another device.
    if port not in core.getLoadedDevices():
        core.loadDevice(port, "SerialManager", port)
        # Set port configurations before initializing.
        core.setProperty(port, "AnswerTimeout", timeout_ms)
        core.setProperty(port, "BaudRate", baud_rate)
        core.setProperty(port, "DTR", "Enable" if dtr else "Disable")
        core.setProperty(port, "DataBits", data_bits)
        core.setProperty(port, "DelayBetweenCharsMs", char_delay_ms)
        core.setProperty(port, "Fast USB to Serial", "Enable" if fast_usb else "Disable")
        core.setProperty(
            port, "Handshaking", "Off" if handshake == "none" else handshake.capitalize()
        )
        core.setProperty(port, "Parity", parity.capitalize())
        core.setProperty(port, "StopBits", str(stop_bits))
        core.setProperty(port, "Verbose", "1" if verbose else "0")
        # Initialize the configured port.
        core.initializeDevice(port)

    return port


def normalize_zooms(
    states: list[str] | list[int], zooms: list[float] | None = None
) -> dict[str | int, float]:
    if zooms is None:
        zooms = []
        for state in states:
            if isinstance(state, str) and (m := re.search(r"\d+", state)):
                zooms.append(int(m[0]))
            else:
                zooms.append(1)

    return {state: zoom for state, zoom in zip(states, zooms, strict=True)}
