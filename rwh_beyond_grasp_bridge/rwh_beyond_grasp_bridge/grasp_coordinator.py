"""Compatibility import; implementation lives in rwh_beyond_grasp_pipeline."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("rwh_beyond_grasp_pipeline.grasp_coordinator")
