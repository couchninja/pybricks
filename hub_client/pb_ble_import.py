"""Import pb_ble submodules without loading pb_ble.__init__.

pybricks-ble's package init eagerly imports VirtualBLE, which subclasses
``pybricks.hubs._common.BLE`` — removed in pybricks 4.0. We only need the
BlueZ broadcaster stack, which still works.
"""

from __future__ import annotations

import importlib
import sys
import types
from importlib.util import find_spec
from types import ModuleType


def load_bluezdbus() -> ModuleType:
    if "pb_ble.bluezdbus" in sys.modules:
        return sys.modules["pb_ble.bluezdbus"]

    spec = find_spec("pb_ble")
    if spec is None or not spec.submodule_search_locations:
        raise ImportError("pb_ble is not installed")

    root = spec.submodule_search_locations[0]
    if "pb_ble" not in sys.modules:
        pkg = types.ModuleType("pb_ble")
        pkg.__path__ = [root]
        pkg.__package__ = "pb_ble"
        sys.modules["pb_ble"] = pkg

    return importlib.import_module("pb_ble.bluezdbus")
