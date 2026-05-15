"""
jetson_utils — simulation shim for NVIDIA jetson-utils.

Students write:
    import jetson_utils
    camera = jetson_utils.videoSource("csi://0")
    img    = camera.Capture()
    arr    = jetson_utils.cudaToNumpy(img)

In simulation:
  - videoSource() connects to Unity and requests JPEG frames via TCP (port 5556).
  - videoOutput() opens a local cv2 window.
  - cudaToNumpy() / cudaFromNumpy() are simple numpy wrappers (no real CUDA).

The returned "CUDA images" are CudaImageSim objects that wrap numpy arrays.
All attributes and methods match the real jetson_utils interface so student
code runs without changes.
"""

import base64
import time
import numpy as np
import cv2
import sim_client


# ─── CUDA image wrapper ──────────────────────────────────────────────────────

class CudaImageSim:
    """
    Wraps a numpy (H, W, C) uint8 array so it looks like a real
    jetson_utils CUDA image.  No GPU memory is actually used.
    """

    def __init__(self, array: np.ndarray):
        # Store as RGB (real jetson_utils images are RGB)
        self._array = np.ascontiguousarray(array, dtype=np.uint8)
        self.shape  = self._array.shape

    def __repr__(self):
        return f"CudaImageSim(shape={self.shape})"


# ─── Public conversion functions ─────────────────────────────────────────────

def cudaToNumpy(img: CudaImageSim) -> np.ndarray:
    """
    Convert a jetson_utils CUDA image to a numpy uint8 array.
    The returned array has shape (H, W, C) and matches what the
    real function returns (channel order depends on source).
    """
    if isinstance(img, CudaImageSim):
        return img._array.copy()
    # Passthrough for plain numpy arrays (convenience)
    return np.asarray(img, dtype=np.uint8)


def cudaFromNumpy(array: np.ndarray) -> CudaImageSim:
    """Wrap a numpy array in a CudaImageSim so it can be passed to Render()."""
    return CudaImageSim(array)


# ─── VideoSource (camera) ────────────────────────────────────────────────────

class _VideoSource:
    """
    Simulates jetson_utils.videoSource.
    Captures frames from Unity by sending a "get_frame" query on port 5556.
    """

    def __init__(self, uri, argv=None):
        self._uri    = uri
        self._open   = True
        self._width  = 640
        self._height = 480
        print(f"[jetson_utils] videoSource opened: {uri} (→ Unity SimQueryServer)")

    def Capture(self, timeout=1000):
        """
        Request a frame from Unity.
        Returns a CudaImageSim (RGB uint8) or None if Unity is not running.
        """
        if not self._open:
            return None

        resp = sim_client.send_query({"command": "get_frame"})
        if not resp or resp.get("status") != "ok":
            return None

        jpeg_b64 = resp.get("jpeg", "")
        if not jpeg_b64:
            return None

        try:
            jpeg_bytes = base64.b64decode(jpeg_b64)
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return None
            # Convert BGR→RGB to match real jetson_utils output
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            return CudaImageSim(rgb)
        except Exception as e:
            print(f"[jetson_utils] Capture decode error: {e}")
            return None

    def GetWidth(self)  -> int: return self._width
    def GetHeight(self) -> int: return self._height
    def IsStreaming(self) -> bool: return self._open

    def Close(self):
        self._open = False
        print("[jetson_utils] camera closed")


# ─── VideoOutput (display) ───────────────────────────────────────────────────

class _VideoOutput:
    """
    Simulates jetson_utils.videoOutput.
    Renders frames to a local OpenCV window (or records to a file).
    """

    def __init__(self, uri, argv=None):
        self._uri       = uri
        self._streaming = True
        self._writer    = None
        self._window    = None

        if uri.startswith("file://"):
            path = uri[len("file://"):]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(path, fourcc, 30, (640, 480))
            print(f"[jetson_utils] videoOutput → file: {path}")
        else:
            self._window = "JetBot Simulation"
            print(f"[jetson_utils] videoOutput → display window")

    def IsStreaming(self) -> bool:
        if self._window:
            # Allow 'q' key to stop the stream (same convention as real display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self._streaming = False
        return self._streaming

    def Render(self, img):
        """Display or record one frame."""
        if not self._streaming:
            return

        arr = cudaToNumpy(img) if isinstance(img, CudaImageSim) else np.asarray(img)

        # Convert RGB→BGR for OpenCV
        if arr.ndim == 3 and arr.shape[2] == 3:
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif arr.ndim == 3 and arr.shape[2] == 4:
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        else:
            bgr = arr

        if self._writer:
            resized = cv2.resize(bgr, (640, 480))
            self._writer.write(resized)
        if self._window:
            cv2.imshow(self._window, bgr)

    def Close(self):
        self._streaming = False
        if self._writer:
            self._writer.release()
        if self._window:
            cv2.destroyAllWindows()
        print("[jetson_utils] display closed")


# ─── Factory functions ───────────────────────────────────────────────────────

def videoSource(uri: str, argv: list | None = None) -> _VideoSource:
    """
    Open a camera source.  In simulation any URI (csi://0, etc.) connects
    to the Unity virtual camera.
    """
    return _VideoSource(uri, argv)


def videoOutput(uri: str = "display://0",
                argv: list | None = None) -> _VideoOutput:
    """
    Open a display or file output.
    'display://0' opens a cv2 window; 'file://name.mp4' records to a file.
    """
    return _VideoOutput(uri, argv)
