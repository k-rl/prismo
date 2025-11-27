__version__ = "0.1.0"

__all__ = ["acq", "live", "load", "multi_acq", "tiled_acq", "utils"]

import logging
import sys

from beartype import BeartypeConf
from beartype.claw import beartype_all, beartype_this_package
from prismo.control import load
from prismo.gui import acq, live, multi_acq, tiled_acq

from . import utils

beartype_this_package()
beartype_all(conf=BeartypeConf(violation_type=UserWarning))

class IndentFormatter(logging.Formatter):
    def format(self, record):
        formatted_record = super().format(record)
        return formatted_record.replace("\n", "\n    ")


logger = logging.getLogger(__name__)
handler = logging.StreamHandler(sys.stdout)

formatter = IndentFormatter(
    "{asctime} - {message}",
    style="{",
    datefmt="%Y-%m-%d %H:%M:%S",
)
handler.setFormatter(formatter)
logger.addHandler(handler)
logger.setLevel(logging.INFO)
