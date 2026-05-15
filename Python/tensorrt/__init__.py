"""
tensorrt — simulation shim for NVIDIA TensorRT.

Students write (unchanged from real JetBot code):

    import tensorrt as trt
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())
    context = engine.create_execution_context()
    d_input  = cuda.mem_alloc(trt.volume(input_shape)  * np.float32().nbytes)
    d_output = cuda.mem_alloc(trt.volume(output_shape) * np.float32().nbytes)
    context.execute_async_v2(bindings=[int(d_input), int(d_output)],
                             stream_handle=stream.handle)

In simulation:
  - The engine file is never read or used.
  - execute_async_v2() marks a flag on the stream.
  - stream.synchronize() queries Unity for visible objects and fills the
    output buffer with detections in the exact (1, 5, 8400) TRT format.

The output shape (1, 5, 8400) is hardcoded to match the YOLOv8-style engine
used by the JetBot.  Each detection row is [x1, y1, x2, y2, conf].
"""

import numpy as np

# Output shape constants matching the student's JetBot engine
OUTPUT_SHAPE     = (1, 5, 8400)
DETECTION_CONF   = 0.92          # synthetic confidence for Unity detections
FRAME_W          = 640           # input frame width expected by preprocess()
FRAME_H          = 480           # input frame height


def volume(shape) -> int:
    """Return total number of elements (same as np.prod)."""
    result = 1
    for s in shape:
        result *= s
    return result


# ─── Logger ──────────────────────────────────────────────────────────────────

class Logger:
    WARNING = 0
    INFO    = 1
    ERROR   = 2

    def __init__(self, severity=WARNING):
        self._severity = severity


# ─── Runtime (context manager) ───────────────────────────────────────────────

class Runtime:
    def __init__(self, logger=None):
        self._logger = logger

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def deserialize_cuda_engine(self, data):
        """
        In simulation the serialized engine bytes are ignored.
        Returns a mock Engine that delegates detections to Unity.
        """
        return Engine()


# ─── Engine ──────────────────────────────────────────────────────────────────

class Engine:
    """
    Mock TensorRT engine.

    Binding layout (hardcoded to match YOLOv8 export):
      index 0 → "images"  (input,  shape 1×3×640×640)
      index 1 → "output0" (output, shape 1×5×8400)
    """

    num_bindings = 2

    _BINDING_NAMES  = {0: "images", 1: "output0"}
    _BINDING_SHAPES = {0: (1, 3, 640, 640), 1: OUTPUT_SHAPE}
    _BINDING_INDEX  = {"images": 0, "output0": 1}

    def create_execution_context(self) -> "Context":
        return Context(self)

    def get_binding_index(self, name: str) -> int:
        return self._BINDING_INDEX.get(name, 0)

    def get_binding_shape(self, index: int):
        return self._BINDING_SHAPES.get(index, OUTPUT_SHAPE)

    def get_binding_name(self, index: int) -> str:
        return self._BINDING_NAMES.get(index, "")

    def get_binding_is_input(self, index: int) -> bool:
        return index == 0


# ─── Context ─────────────────────────────────────────────────────────────────

# Global stream registry so Context can look up the stream from its handle
_streams: dict = {}


class Context:
    def __init__(self, engine: Engine):
        self._engine = engine

    def execute_async_v2(self, bindings: list, stream_handle: int):
        """
        Mark the stream as pending inference.
        The actual Unity query happens in stream.synchronize().
        """
        stream = _streams.get(stream_handle)
        if stream is not None:
            stream._pending_inference = True
            stream._bindings = bindings
            stream._engine   = self._engine
