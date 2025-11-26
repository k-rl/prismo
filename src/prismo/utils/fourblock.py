import logging

from ..microfluidic import Chip
from .general import _check_valve_mapping, sleep

logger = logging.getLogger(__name__)


def deadend_fill(
    chip: Chip,
    buffer: str,
):
    """Dead-end fill the device.

    Parameters:
    -----------
    chip :
        The device to dead-end fill.
    buffer :
        Name of the buffer input.
    """
    chip.close_all()
    chip.sandwich = "open"
    chip.inlet = "open"
    chip[buffer] = "open"


def purge_common_inlet(
    chip: Chip,
    flow: str,
    waste: str,
    wait_time: int = 5,
    keep_flow_open: bool = True,
):
    """Purge air from a common inlet for a set amount of time.

    Expects that flow and waste inlet valves are in the common valve
    location (i.e., common inlet flow valves 1–5).

    Parameters:
    -----------
    chip :
        The device to purge.
    flow :
        The name of the inlet being purged.
    waste :
        The name of the waste inlet to purge to.
    wait_time :
        Number of seconds to purge `flow` for.
    keep_flow_open :
        Whether to keep the flow valve open.

    Notes:
    ------
    Inlet1 always stays closed at the end of this function.

    Examples:
    ---------
    >>> purge_common_inlet(c.chip, "bBSA2", "waste1", wait_time=10)

    This will close the inlet1 control valve and open bBSA2 and waste1 flow
    valves for 10 seconds.
    """
    if flow not in chip.valves:
        raise ValueError(f"{flow} does not exist in {chip.name}.")
    if waste not in chip.valves:
        raise ValueError(f"{waste} does not exist in {chip.name}.")

    chip.inlet[0] = "closed"
    logger.info(f"Flowing {flow} to {waste} for {wait_time} seconds.")
    chip[flow] = "open"
    chip[waste] = "open"
    sleep(wait_time)

    if not keep_flow_open:
        chip[flow] = "closed"

    chip[waste] = "closed"
    logger.info(f"Done flowing {flow} to {waste}.")


def purge_block_inlets(
    chip: Chip,
    wait_time: int = 5,
    keep_block0_open: bool = False,
):
    """Purges all four block-specific inlets for a set amount of time.

    Expects that block inlets contain low-pressure lines to flow to the
    adjacent block outlets, between block1 and block2 control lines.

    Parameters:
    -----------
    chip :
        The prismo.devices.microfluidic.Chip object for 4-block device.
    wait_time :
        Number of seconds to purge `flow`.
    keep_block0_open :
        Whether to keep the block1 control valve open.
    """
    # Close inlets 1 and 2
    chip.inlet = "closed"

    # Flow upper block inlets to lower (waste) inlets
    logger.info(f"Purging all block inlets for {wait_time} seconds.")
    chip.block_inlet = "open"
    sleep(wait_time)

    if not keep_block0_open:
        logger.info("Leaving block 0 open.")
        chip.block_inlet[0] = "closed"
    chip.block_inlet[1] = "closed"
    logger.info("Done purging block inlets.")


def pattern_anti_gfp(
    chip: Chip,
    waste: str = "waste1",
    bbsa: str = "bbsa2",
    na: str = "na3",
    anti_gfp: str = "in4",
    pbs: str = "in5",
    outlet: int = -1,
):
    """Pattern a chip device to add a bbsa-na-anti_gfp pedestal under the button.

    Parameters:
    -----------
    chip :
        The device to pattern.
    waste :
        The waste valve for purging air from other inlets.
    bbsa :
        Inlet containing biotinylated bovine serum albumin.
    na :
        Inlet containing neutravidin.
    anti_gfp :
        Inlet containing gfp antibodies.
    pbs :
        Inlet containing phosphate buffer saline.
    outlet :
        Which outlet to use ("out2" = common, "out1" = block-specific).
    """
    # Check valve mappings for the non-hard-coded valves
    valve_args = [waste, bbsa, na, anti_gfp, pbs, outlet]
    for valve in valve_args:
        _check_valve_mapping(chip, valve)

    logger.info(
        ">>> Patterning\n"
        f"Starting anti GFP patterning script for device {chip.name}.\n"
        "NOTE: Passivation with BSA should already have been done.\n"
        f"Valve mappings: {waste=}, {bbsa=}, {na=}, {anti_gfp=}, {pbs=}, {outlet=}"
    )
    chip.close_all()
    logger.info(f"Closed all valves for device {chip.name}")

    # Prep device flow state; need sandR, sandL, inlet2, and outlet open
    chip.sandwich = "open"
    chip.outlet[outlet] = "open"

    # Flow with buttons closed
    logger.info(
        f">>> Step 1: BBSA flow\nFlushing {bbsa} to {waste} for 5 sec, then closing {waste}."
    )
    purge_common_inlet(chip, bbsa, waste, wait_time=5)

    logger.info(f"Flushing {bbsa=} through device with buttons closed for 5 min.")
    chip.inlet = "open"
    sleep(5 * 60)

    logger.info(f"Opening buttons; flowing {bbsa=} for 35 min.")
    chip.button = "open"
    sleep(35 * 60)

    logger.info(f"Done flowing {bbsa=}.")
    chip[bbsa] = "closed"

    logger.info(f"Flushing {pbs=} to {waste} for 30 sec, then closing {waste}.")
    purge_common_inlet(chip, pbs, waste, wait_time=30)

    logger.info(f"Flushing {pbs=} through device with buttons open for 10 min.")
    chip.inlet = "open"
    sleep(10 * 60)

    logger.info(f"Done flowing PBS ({pbs}).")
    chip[pbs] = "closed"

    logger.info(
        ">>> Step 2: Neutravidin flow\n"
        f"Flushing NA ({na}) to {waste} for 30 sec, then closing {waste}."
    )
    purge_common_inlet(chip, na, waste, wait_time=30)

    logger.info(f"Flushing {na=} through device with buttons open for 30 min.")
    chip.inlet = "open"
    sleep(30 * 60)

    logger.info(f"Done flowing {na=}. Flowing {pbs=} through device for 10 min.")
    chip[na] = "closed"
    chip[pbs] = "open"
    sleep(10 * 60)

    logger.info(f"Done flowing {pbs=}.")
    chip[pbs] = "closed"

    logger.info(
        f">>> Step 3: bBSA quench\nFlowing {bbsa} for 35 min with buttons closed to quench walls."
    )
    chip.button = "closed"
    chip[bbsa] = "open"
    sleep(35 * 60)

    logger.info(f"Done flowing {bbsa=}. Flowing {pbs=} through device for 10 min.")
    chip[bbsa] = "closed"
    chip[pbs] = "open"
    sleep(10 * 60)

    logger.info(f"Done flowing {pbs=}.")
    chip[pbs] = "closed"

    logger.info(
        ">>> Step 4: anti_gfp flow\n"
        f"Flushing {anti_gfp=} to {waste} for 30 sec, then closing {waste}."
    )
    purge_common_inlet(chip, anti_gfp, waste, wait_time=30)

    logger.info(f"Flowing anti_gfp ({anti_gfp}) through device for 30 sec.")
    chip.inlet = "open"
    sleep(30)

    logger.info(f"Opened buttons. Flowing anti_gfp ({anti_gfp}) through device for 13.3 min.")
    chip.button = "open"
    sleep(int(13.3 * 60))

    logger.info(f"Flowing anti_gfp ({anti_gfp}) through device with buttons closed for 30 sec.")
    chip.button = "closed"
    sleep(30)

    logger.info(f"Done flowing {anti_gfp=} through device. Washing with {pbs=}.")
    chip[anti_gfp] = "closed"
    chip[pbs] = "open"
    sleep(10 * 60)

    logger.info(f"Done flowing {pbs=}. Closing outlet.")
    deadend_fill(chip, buffer=pbs)
    logger.info(">>> Done with patterning.")


def sds_wash(
    chip: Chip,
    waste: str = "waste1",
    sds: str = "bBSA2",
    pbs: str = "in5",
    outlet: int = -1,
    wash_lagoons: bool = True,
    keep_neck_open: bool = False,
):
    """Wash the chip with SDS one time. To be used within a loop to repeat
    washes multiple times and acquire images after each wash.

    Parameters:
    -----------
    chip :
        The prismo.devices.microfluidic.Chip object for 4-block device.
    waste :
        The waste valve for purging air from other inlets.
    sds :
        Inlet containing SDS.
    pbs :
        Inlet containing wash PBS.
    outlet :
        Which outlet to use ('out2' = common, 'out1' = block-specific).
    wash_lagoons :
        Whether to wash with necks open or closed.
    keep_neck_open :
        Whether to leave necks open or closed.
    """
    for valve in [waste, sds, pbs, outlet]:
        if valve not in chip.valves:
            raise ValueError(f"Valve {valve} does not exist in {chip.name}.")

    logger.info(">>> SDS Wash\nValve mappings: {waste=}, {sds=}, {pbs=}, {outlet=}")

    deadend_fill(chip, buffer=pbs)
    chip[pbs] = "closed"

    # Flow SDS over device for 5 minutes.
    chip.neck = "open" if wash_lagoons else "closed"
    purge_common_inlet(chip, sds, waste)
    chip.inlet = "open"
    chip.outlet[outlet] = "open"
    sleep(5 * 60)

    chip[sds] = "closed"
    chip[outlet] = "closed"

    # Wash with PBS for 10 minutes.
    logger.info(">>> Washing with PBS\n")
    purge_common_inlet(chip, pbs, waste)
    chip.inlet = "open"
    chip[outlet] = "open"
    sleep(10 * 60)

    chip[outlet] = "closed"
    chip.neck = "open" if keep_neck_open else "closed"
