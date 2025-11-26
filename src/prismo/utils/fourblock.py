from pprint import pp

from ..microfluidic import Chip
from .general import _check_valve_mapping, _timestamp, sleep


def deadend_fill(
    chip: Chip,
    buffer: str,
) -> None:
    """Dead-end fill the device.

    Parameters:
    -----------
    chip :
        The device to dead-end fill.
    buffer :
        Name of buffer line.
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
    verbose: bool = True,
) -> None:
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
    verbose :
        Whether to print each step.

    Notes:
    ------
    Inlet1 always stays closed at the end of this function.

    Examples:
    ---------
    >>> purge_common_inlet(c.chip, "bBSA2", "waste1", wait_time=10, verbose=False)

    This will close the inlet1 control valve and open bBSA2 and waste1 flow
    valves for 10 seconds without printing anything out.
    """
    if flow not in chip.valves:
        raise ValueError(f"{flow} does not exist in {chip.name}.")
    if waste not in chip.valves:
        raise ValueError(f"{waste} does not exist in {chip.name}.")

    chip.inlet[0] = "closed"
    if verbose:
        print(f"Flowing {flow} to {waste} for {wait_time} seconds.")
    chip[flow] = "open"
    chip[waste] = "open"
    sleep(wait_time)

    if not keep_flow_open:
        chip[flow] = "closed"

    chip[waste] = "closed"
    if verbose:
        print(f"Done flowing {flow} to {waste}.")


def purge_block_inlets(
    chip: Chip,
    wait_time: int = 5,
    keep_block1_open: bool = False,
    verbose: bool = True,
) -> None:
    """Purges all four block-specific inlets for a set amount of time.

    Expects that block inlets contain low-pressure lines to flow to the
    adjacent block outlets, between block1 and block2 control lines.

    Parameters:
    -----------
    chip :
        The prismo.devices.microfluidic.Chip object for 4-block device.
    wait_time :
        Number of seconds to purge `flow`.
    keep_block1_open :
        Whether to keep the block1 control valve open. (Dead-end fill.)
    verbose :
        Whether to print each step.

    Returns:
    --------
    None :
        This function controls flow on a 4-block device; nothing is
        returned.

    Notes:
    ------
    None.

    Examples:
    ---------
    >>> purge_block_inlets(c.chip, wait_time=10, keep_block1_open=True)
    """
    # Close inlets 1 and 2
    chip.inlet = "closed"

    # Flow upper block inlets to lower (waste) inlets
    if verbose:
        print(f"Purging all block inlets for {wait_time} seconds.")
    chip.block_inlet = "open"
    sleep(wait_time)

    if not keep_block1_open:
        if verbose:
            print("Leaving block 1 open.")
        chip.block_inlet[0] = "closed"
    chip.block_inlet[1] = "closed"
    if verbose:
        print("Done purging block inlets.")


def pattern_anti_gfp(
    chip: Chip,
    waste: str = "waste1",
    bbsa: str = "bBSA2",
    na: str = "na3",
    anti_gfp: str = "in4",
    pbs: str = "in5",
    outlet: int = -1,
    verbose: bool = True,
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
    verbose :
        Whether to print each step.
    """
    # Check valve mappings for the non-hard-coded valves
    valve_args = [waste, bbsa, na, anti_gfp, pbs, outlet]
    for valve in valve_args:
        _check_valve_mapping(chip, valve)

    if verbose:
        print(f">>> Patterning - {_timestamp()}")
        print(f"Starting anti_gfp patterning script for device {chip.name}.")
        print("NOTE: Passivation with BSA should already have been done.")
        print("Valve mappings:")
        pp(
            {
                "waste": waste,
                "bBSA": bbsa,
                "NA": na,
                "anti_gfp": anti_gfp,
                "PBS": pbs,
                "outlet": outlet,
            }
        )

    chip.close_all()
    if verbose:
        print(f"Closed all valves for device {chip.name}")

    # Prep device flow state; need sandR, sandL, inlet2, and outlet open
    chip.sandwich = "open"
    chip.outlet[outlet] = "open"

    # Flow with buttons closed
    if verbose:
        print(f">>> Step 1: bBSA flow - {_timestamp()}")
        print(f"Flushing {bbsa} to {waste} for 5 sec, then closing {waste}.")

    purge_common_inlet(chip, bbsa, waste, wait_time=5, verbose=False)
    chip.inlet = "open"

    if verbose:
        print(f"Flushing {bbsa} through device with buttons closed for 5 min.")
    sleep(5 * 60)

    # Open buttons for 35 min
    chip.button = "open"
    if verbose:
        print(f"Opening buttons; flowing {bbsa} for 35 min.")
    sleep(35 * 60)

    chip[bbsa] = "closed"
    if verbose:
        print(f"Done flowing {bbsa}.")

    # Flush with PBS
    if verbose:
        print(f"Flushing PBS ({pbs}) to {waste} for 30 sec, then closing {waste}.")
    purge_common_inlet(chip, pbs, waste, wait_time=30, verbose=False)
    chip.inlet = "open"

    if verbose:
        print(f"Flushing PBS ({pbs}) through device with buttons open for 10 min.")
    sleep(10 * 60)

    chip[pbs] = "closed"
    if verbose:
        print(f"Done flowing PBS ({pbs}).")

    # Neutravidin
    if verbose:
        print(f">>> Step 2: Neutravidin flow - {_timestamp()}")
        print(f"Flushing NA ({na}) to {waste} for 30 sec, then closing {waste}.")
    purge_common_inlet(chip, na, waste, wait_time=30, verbose=False)
    chip.inlet = "open"

    if verbose:
        print(f"Flushing NA ({na}) through device with buttons open for 30 min.")
    sleep(30 * 60)

    chip[na] = "closed"
    if verbose:
        print(f"Done flowing NA ({na}). Flowing PBS ({pbs}) through device for 10 min.")

    # Wash with PBS
    chip[pbs] = "open"
    sleep(10 * 60)

    chip[pbs] = "closed"
    if verbose:
        print(f"Done flowing PBS ({pbs}).")

    # Quench walls with bBSA
    if verbose:
        print(f">>> Step 3: bBSA quench - {_timestamp()}")
        print(f"Flowing {bbsa} for 35 min with buttons closed to quench walls.")
    chip.button = "closed"
    chip[bbsa] = "open"
    sleep(35 * 60)

    chip[bbsa] = "closed"
    if verbose:
        print(f"Done flowing {bbsa}. Flowing PBS ({pbs}) through device for 10 min.")

    # Wash with PBS
    chip[pbs] = "open"
    sleep(10 * 60)

    chip[pbs] = "closed"
    if verbose:
        print(f"Done flowing PBS ({pbs}).")

    # Anti-GFP flowing
    if verbose:
        print(f">>> Step 4: anti_gfp flow - {_timestamp()}")
        print(f"Flushing anti_gfp ({anti_gfp}) to {waste} for 30 sec, then closing {waste}.")
    purge_common_inlet(chip, anti_gfp, waste, wait_time=30, verbose=False)
    chip.inlet = "open"

    if verbose:
        print(f"Flowing anti_gfp ({anti_gfp}) through device for 30 sec.")
    sleep(30)
    chip.button = "open"
    if verbose:
        print(f"Opened buttons. Flowing anti_gfp ({anti_gfp}) through device for 13.3 min.")
    sleep(int(13.3 * 60))

    if verbose:
        print(f"Flowing anti_gfp ({anti_gfp}) through device with buttons closed for 30 sec.")
    chip.button = "closed"
    sleep(30)

    chip[anti_gfp] = "closed"
    if verbose:
        print(f"Done flowing GFP antibody ({anti_gfp}) through device. Washing with PBS ({pbs}).")

    # Final PBS wash
    chip[pbs] = "open"
    sleep(10 * 60)

    if verbose:
        print(f"Done flowing PBS ({pbs}). Closing outlet.")

    # Close outlet to dead-end fill
    chip[outlet] = "closed"

    if verbose:
        print(f">>> Done with patterning - {_timestamp()}")


def sds_wash(
    chip: Chip,
    waste: str = "waste1",
    sds: str = "bBSA2",
    pbs: str = "in5",
    outlet: int = -1,
    wash_lagoons: bool = True,
    keep_neck_open: bool = False,
    verbose: bool = True,
) -> None:
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
    verbose :
        Whether to print each step.
    """
    for valve in [waste, sds, pbs, outlet]:
        if valve not in chip.valves:
            raise ValueError(f"Valve {valve} does not exist in {chip.name}.")

    if verbose:
        print(f">>> SDS Wash - {_timestamp()}")
        print("Valve mappings:")
        pp(
            {
                "waste": waste,
                "SDS": sds,
                "PBS": pbs,
                "outlet": outlet,
            }
        )

    # Set dead-end fill flow state
    deadend_fill(chip, buffer=pbs)
    chip[pbs] = "closed"

    # Flow SDS over device for 5 minutes.
    chip.neck = "open" if wash_lagoons else "closed"
    purge_common_inlet(chip, sds, waste, verbose=False)
    chip.inlet = "open"
    chip.outlet[outlet] = "open"
    sleep(5 * 60)

    chip[sds] = "closed"
    chip[outlet] = "closed"

    # Wash with PBS for 10 min
    if verbose:
        print(f">>> Washing with PBS - {_timestamp()}")
    purge_common_inlet(chip, pbs, waste, verbose=False)
    chip.inlet = "open"
    chip[outlet] = "open"
    sleep(10 * 60)

    chip[outlet] = "closed"
    chip.neck = "open" if keep_neck_open else "closed"
