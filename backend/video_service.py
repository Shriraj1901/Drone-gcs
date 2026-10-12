"""
video_service.py - reads video (webcam / RTSP / UDP / test pattern)
in its own thread and serves it to the browser as an MJPEG stream.

Low latency trick: the capture thread always keeps ONLY the newest frame.
Slow viewers skip old frames instead of building up a delay.
"""
import asyncio
import os
import threading
import time

# Low-latency settings for FFmpeg (RTSP/UDP). Must be set BEFORE importing cv2.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay")

import cv2
import numpy as np

import config


class _TestPattern:
    """Fake 'camera' that draws a moving bar - for testing without hardware."""
    def __init__(self):
        self._n = 0

    def isOpened(self):
        return True

    def set(self, *args):
        pass

    def read(self):
        time.sleep(1 / 30)
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[:, :, 0] = 40
        x = (self._n * 4) % 600
        cv2.rectangle(frame, (x, 0), (x + 40, 480), (0, 160, 255), -1)
        cv2.putText(frame, "TEST VIDEO (no camera)", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, time.strftime("%H:%M:%S"), (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, f"frame {self._n}", (30, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        self._n += 1
        return True, frame

    def release(self):
        pass


class VideoService:
    def __init__(self):
        self._thread = None
        self._stop = threading.Event()
        self._cond = threading.Condition()   # also guards the fields below
        self._jpeg = None
        self._frame_id = 0
        self._running = False
        self._status = {"state": "stopped", "message": "Video stopped",
                        "source": "", "fps": 0.0}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def start(self, source_type: str, value: str) -> dict:
        if source_type not in ("test", "webcam", "rtsp", "udp"):
            raise ValueError("Unknown video source type")
        if source_type == "rtsp" and not value:
            raise ValueError("Please enter an RTSP URL")
        self.stop()
        stop = threading.Event()
        with self._cond:
            self._stop = stop
            self._running = True
            self._jpeg = None
            self._status = {"state": "starting",
                            "message": f"Opening {source_type} ...",
                            "source": f"{source_type}:{value}", "fps": 0.0}
        self._thread = threading.Thread(
            target=self._capture_loop, args=(stop, source_type, value),
            daemon=True, name="video")
        self._thread.start()
        return self.get_status()

    def stop(self) -> None:
        self._stop.set()
        with self._cond:
            self._running = False
            self._cond.notify_all()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=3)
        self._thread = None
        with self._cond:
            self._jpeg = None
            self._status = {"state": "stopped", "message": "Video stopped",
                            "source": "", "fps": 0.0}

    def get_status(self) -> dict:
        with self._cond:
            return {**self._status, "running": self._running}

    async def mjpeg_stream(self):
        """Async generator used by /video_feed."""
        last_id = -1
        while True:
            jpeg, last_id = await asyncio.to_thread(self._wait_for_frame, last_id, 1.0)
            if not self._running:
                break
            if jpeg is None:
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _wait_for_frame(self, last_id: int, timeout: float):
        with self._cond:
            self._cond.wait_for(
                lambda: self._frame_id != last_id or not self._running, timeout)
            if self._jpeg is not None and self._frame_id != last_id:
                return self._jpeg, self._frame_id
            return None, last_id

    def _set_status(self, stop, state: str, message: str, fps=None) -> None:
        with self._cond:
            if self._stop is stop and not stop.is_set():
                self._status["state"] = state
                self._status["message"] = message
                if fps is not None:
                    self._status["fps"] = fps

    @staticmethod
    def _open(source_type: str, value: str):
        if source_type == "test":
            return _TestPattern()
        if source_type == "webcam":
            backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
            cap = cv2.VideoCapture(int(value or 0), backend)
        else:
            if source_type == "rtsp":
                url = value
            else:  # udp
                url = f"udp://0.0.0.0:{int(value or config.DEFAULT_UDP_VIDEO_PORT)}"
            # 5 s timeouts so a wrong URL fails fast instead of hanging
            cap = cv2.VideoCapture(
                url, cv2.CAP_FFMPEG,
                [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                 cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000])
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # keep latency low
        except Exception:
            pass
        return cap

    def _capture_loop(self, stop, source_type: str, value: str) -> None:
        while not stop.is_set():
            cap = None
            try:
                cap = self._open(source_type, value)
                if not cap.isOpened():
                    raise RuntimeError(
                        "Could not open video source. Check the URL/port/index.")
                self._set_status(stop, "streaming", "Streaming")

                fails, frames, t_fps = 0, 0, time.time()
                while not stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        fails += 1
                        if fails > 30:
                            raise RuntimeError("Video stream lost")
                        time.sleep(0.05)
                        continue
                    fails = 0

                    h, w = frame.shape[:2]
                    if w > config.VIDEO_MAX_WIDTH:
                        scale = config.VIDEO_MAX_WIDTH / w
                        frame = cv2.resize(frame, (config.VIDEO_MAX_WIDTH, int(h * scale)))

                    ok, buf = cv2.imencode(
                        ".jpg", frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), config.VIDEO_JPEG_QUALITY])
                    if not ok:
                        continue

                    with self._cond:
                        if self._stop is stop:
                            self._jpeg = buf.tobytes()
                            self._frame_id += 1
                            self._cond.notify_all()

                    frames += 1
                    if time.time() - t_fps >= 1.0:
                        self._set_status(stop, "streaming", "Streaming",
                                         fps=round(frames / (time.time() - t_fps), 1))
                        frames, t_fps = 0, time.time()
            except Exception as e:
                self._set_status(stop, "error", f"{e} - retrying...")
            finally:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
            stop.wait(2.0)   # wait, then try to reopen