"""
pycuda.driver — simulation shim for the PyCUDA CUDA driver API.

Students write (unchanged):
    import pycuda.driver as cuda
    import pycuda.autoinit

    d_input  = cuda.mem_alloc(size)
    d_output = cuda.mem_alloc(size)
    stream   = cuda.Stream()

    cuda.memcpy_htod_async(d_input,  input_np,  stream)
    context.execute_async_v2(bindings=[int(d_input), int(d_output)],
                             stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(output_np, d_output, stream)
    stream.synchronize()

How the simulation works
─────────────────────────
1. mem_alloc()         → returns a _Allocation object (unique integer id)
2. memcpy_htod_async() → stores the source numpy array in a global dict
3. execute_async_v2()  → marks the stream "inference pending" (see tensorrt shim)
4. memcpy_dtoh_async() → registers (dest_np, src_alloc_id) in the stream
5. stream.synchronize()→ if inference is pending:
     a. queries Unity for visible objects via sim_client.send_query()
     b. fills the output numpy array in (1, 5, 8400) TRT detection format
"""

import numpy as np
import sim_client
import tensorrt   # import to access _streams registry and OUTPUT_SHAPE


# ─── Global allocation store ─────────────────────────────────────────────────
# Maps allocation id (int) → numpy array (set by memcpy_htod_async)
_alloc_data: dict = {}


class _Allocation:
    """Represents a block of simulated device memory."""

    def __init__(self, size: int):
        self._id   = id(self)
        self._size = size
        _alloc_data[self._id] = None

    def __int__(self)   -> int: return self._id
    def __index__(self) -> int: return self._id
    def __repr__(self)  -> str: return f"<SimAlloc id={self._id} size={self._size}>"


def mem_alloc(size: int) -> _Allocation:
    """Allocate simulated device memory.  Returns an allocation handle."""
    return _Allocation(size)


# ─── Async copy functions ────────────────────────────────────────────────────

def memcpy_htod_async(dest, src: np.ndarray, stream):
    """
    Simulate host→device copy.
    Stores the numpy array in the allocation registry so the inference
    step can (optionally) inspect it.
    """
    alloc_id = int(dest)
    _alloc_data[alloc_id] = np.ascontiguousarray(src, dtype=np.float32)


def memcpy_dtoh_async(dest: np.ndarray, src, stream):
    """
    Simulate device→host copy.
    Registers (dest, src_id) on the stream — the actual copy happens
    in stream.synchronize() after inference.
    """
    stream._pending_dtoh.append((dest, int(src)))


# ─── Stream ──────────────────────────────────────────────────────────────────

class Stream:
    """
    Simulated CUDA stream.  synchronize() is where the actual Unity query
    and output-buffer population take place.
    """

    def __init__(self):
        self.handle            = id(self)
        self._pending_inference = False
        self._pending_dtoh:    list = []   # list of (dest_np, src_alloc_id)
        self._bindings         = None
        self._engine           = None
        # Register in the global stream table so the TRT context can find us
        tensorrt._streams[self.handle] = self

    def synchronize(self):
        """
        Wait for all async operations to complete.
        In simulation: if inference was requested, query Unity for detections,
        then copy results into the registered output buffer.
        """
        if self._pending_inference and self._bindings is not None:
            self._run_sim_inference()
        self._pending_inference = False
        self._bindings          = None
        self._engine            = None

    # ── Internal ─────────────────────────────────────────────────────────────

    def _run_sim_inference(self):
        """
        Query Unity for currently visible 'cough_drop_container' objects
        and populate the output allocation in YOLOv8-style TRT format.

        Output shape: (1, 5, 8400) where axis-1 is [x1, y1, x2, y2, conf].
        After postprocess()'s transpose: each row becomes [x1, y1, x2, y2, conf].
        """
        resp       = sim_client.send_query({"command": "detect_objects"})
        detections = resp.get("objects", [])

        # Build output tensor: shape (1, 5, 8400) filled with zeros (no detection)
        out = np.zeros(tensorrt.OUTPUT_SHAPE, dtype=np.float32)

        for i, det in enumerate(detections):
            if i >= 8400:
                break
            x1   = float(det.get("x1",   0))
            y1   = float(det.get("y1",   0))
            x2   = float(det.get("x2",   0))
            y2   = float(det.get("y2",   0))
            conf = float(det.get("conf", tensorrt.DETECTION_CONF))
            # Fill column i of the (5, 8400) sub-array
            out[0, 0, i] = x1
            out[0, 1, i] = y1
            out[0, 2, i] = x2
            out[0, 3, i] = y2
            out[0, 4, i] = conf

        # Write result into the output allocation
        output_alloc_id = self._find_output_alloc()
        if output_alloc_id is not None:
            _alloc_data[output_alloc_id] = out

        # Perform all pending device→host copies now that the result is ready
        for dest_np, src_id in self._pending_dtoh:
            data = _alloc_data.get(src_id)
            if data is not None and dest_np is not None:
                try:
                    np.copyto(dest_np, data)
                except ValueError:
                    dest_np[...] = data
        self._pending_dtoh = []

    def _find_output_alloc(self):
        """
        Identify the output allocation id using the engine's binding metadata.
        Falls back to assuming index 1 is output if the engine is unavailable.
        """
        if self._bindings is None:
            return None

        # self._engine is the tensorrt.Context; Context._engine is the Engine
        engine = getattr(self._engine, "_engine", None)

        if engine is not None:
            for i, b in enumerate(self._bindings):
                if b is not None and not engine.get_binding_is_input(i):
                    return int(b)

        # Fallback: output is the last non-None binding (index 1 for 2-binding engines)
        for b in reversed(self._bindings):
            if b is not None:
                return int(b)
        return None
