"""
rpg_svo Python bindings

This package provides Python bindings for the Semi-Direct Visual Odometry system.
"""

from ._version import __version__, __url__, __dependencies__

try:
    from .svo_cpp import (
        PinholeCamera,
        AbstractCamera,
        SE3d,
        Frame,
        Stage,
        SVO,
        set_svo_config,
    )

except ImportError as e:
    # This provides a much better error message if the C++ part failed.
    # Include the original error for more detailed debugging.
    raise ImportError(
        "Failed to import the compiled svo C++ core (svo_cpp.so).\n"
        "Please make sure the package was installed correctly after a full compilation.\n"
        f"Original error: {e}"
    ) from e

# ---- APIs -----
# This list defines the public API of the package.
# When a user runs 'from svo_cpp import *', these are the names that get imported.
__all__ = [
    # Core Classes
    "SVO",
    "PinholeCamera",
    "AbstractCamera",
    "Frame",
    "SE3d",

    # Enums
    "Stage",

    # Functions
    "set_svo_config",

    # Metadata
    "__version__",
    "__url__",
    "__dependencies__",
]