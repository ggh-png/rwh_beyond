"""Compatibility import; implementation lives in rwh_beyond_sam3_bridge."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("rwh_beyond_sam3_bridge.segmentation_client")
