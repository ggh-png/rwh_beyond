#!/usr/bin/env python3
"""Tiny CUDA Driver API probe; run INSIDE a GPU-enabled container.

Uses one thread and four bytes of device memory; no torch, model or nvcc install.
Does not reset the device or change other processes.
"""
import ctypes as c
import json

cuda = c.CDLL('libcuda.so.1')

def check(name, *args):
    status = getattr(cuda, name)(*args)
    if status:
        text = c.c_char_p()
        cuda.cuGetErrorString(status, c.byref(text))
        raise RuntimeError(f'{name}: {text.value.decode() if text.value else status}')

check('cuInit', 0)
device = c.c_int()
check('cuDeviceGet', c.byref(device), 0)
name = c.create_string_buffer(256)
check('cuDeviceGetName', name, len(name), device)
major, minor, driver = c.c_int(), c.c_int(), c.c_int()
check('cuDeviceComputeCapability', c.byref(major), c.byref(minor), device)
check('cuDriverGetVersion', c.byref(driver))
context, module, function = c.c_void_p(), c.c_void_p(), c.c_void_p()
allocation = c.c_uint64()
check('cuDevicePrimaryCtxRetain', c.byref(context), device)
try:
    check('cuCtxSetCurrent', context)
    free, total = c.c_size_t(), c.c_size_t()
    check('cuMemGetInfo_v2', c.byref(free), c.byref(total))
    ptx = f'''.version 7.1
.target sm_{major.value}{minor.value}
.address_size 64
.visible .entry write_answer(.param .u64 output) {{
.reg .b64 %rd;
.reg .b32 %r;
ld.param.u64 %rd, [output];
mov.u32 %r, 42;
st.global.u32 [%rd], %r;
ret;
}}
'''.encode()
    jit_log = c.create_string_buffer(8192)
    options = (c.c_int * 2)(5, 6)
    values = (c.c_void_p * 2)(c.cast(jit_log, c.c_void_p), c.c_void_p(len(jit_log)))
    try:
        check('cuModuleLoadDataEx', c.byref(module), c.c_char_p(ptx), 2, options, values)
    except RuntimeError as error:
        raise RuntimeError(str(error) + ': ' + jit_log.value.decode()) from error
    check('cuModuleGetFunction', c.byref(function), module, b'write_answer')
    check('cuMemAlloc_v2', c.byref(allocation), c.c_size_t(4))
    arguments = (c.c_void_p*1)(c.cast(c.byref(allocation), c.c_void_p))
    check('cuLaunchKernel', function, 1,1,1, 1,1,1, 0, c.c_void_p(), arguments, c.c_void_p())
    check('cuCtxSynchronize')
    answer = c.c_int()
    check('cuMemcpyDtoH_v2', c.byref(answer), allocation, c.c_size_t(4))
    assert answer.value == 42, answer.value
    print(json.dumps(dict(gpu=name.value.decode(), compute_capability=[major.value,minor.value],
        cuda_driver_api=driver.value, free_vram_mib=free.value//1048576,
        total_vram_mib=total.value//1048576, kernel_result=answer.value, status='PASS'), indent=2))
finally:
    if allocation.value: check('cuMemFree_v2', allocation)
    if module.value: check('cuModuleUnload', module)
    check('cuCtxSetCurrent', c.c_void_p())
    check('cuDevicePrimaryCtxRelease_v2', device)
