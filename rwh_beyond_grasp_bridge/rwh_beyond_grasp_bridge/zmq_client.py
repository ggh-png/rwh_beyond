"""Compatibility import; implementation lives in rwh_beyond_model_utils."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("rwh_beyond_model_utils.zmq_client")
