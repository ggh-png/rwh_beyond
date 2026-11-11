"""Compatibility import; implementation lives in rwh_beyond_graspgenx_bridge."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("rwh_beyond_graspgenx_bridge.mock_zmq_server")
