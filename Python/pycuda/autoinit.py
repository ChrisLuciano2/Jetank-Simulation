"""
pycuda.autoinit — simulation shim.

On real hardware, importing this module initialises the CUDA device context.
In simulation it is a no-op; importing it is enough.
"""
