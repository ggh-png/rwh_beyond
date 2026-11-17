"""Structured bridge errors shared by model action servers and clients."""
from rwh_beyond_msgs.msg import ErrorCode
from .zmq_client import ProtocolError, RequestCanceled, ServerError


class ActionFailure(RuntimeError):
    def __init__(self, code, domain, name, source, message):
        super().__init__(message)
        self.code, self.domain, self.name, self.source = code, domain, name, source

    @classmethod
    def from_result(cls, result):
        if result.error_code == 0 or not result.error_domain or not result.error_source:
            return cls(ErrorCode.INVALID_RESPONSE, 'transport', 'INVALID_RESPONSE',
                       '/segment_image', 'Failed action returned no structured error')
        return cls(result.error_code, result.error_domain, result.error_name,
                   result.error_source, result.error_msg)


def set_failure(result, error, domain, source):
    if isinstance(error, ActionFailure):
        code, domain, name, source = error.code, error.domain, error.name, error.source
    else:
        name = ('CANCELED' if isinstance(error, RequestCanceled) else
                'TIMEOUT' if isinstance(error, TimeoutError) else
                'SERVER_ERROR' if isinstance(error, ServerError) else
                'INVALID_RESPONSE' if isinstance(error, ProtocolError) else
                'INVALID_INPUT' if isinstance(error, (ValueError, AssertionError)) else
                'INTERNAL_ERROR')
        code = getattr(ErrorCode, name)
    result.success = False
    result.error_code, result.error_domain = code, domain
    result.error_name, result.error_source = name, source
    result.error_msg = str(error)
    result.message = f'{name}: {error}'
