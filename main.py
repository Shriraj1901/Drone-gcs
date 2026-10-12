"""
main.py - starts the web server.   Run with:  python main.py
Then open http://localhost:8000
"""
import asyncio
import json
import logging
import sys
import threading
import webbrowser
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

import config
from backend.mavlink_service import MavlinkService, list_serial_ports
from backend.models import CommandRequest, ConnectRequest, VideoRequest
from backend.video_service import VideoService

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(name)s: %(message)s")


def resource_path(relative: str) -> Path:
    """Find bundled files both when running normally and when packed as an .exe."""
    base = getattr(sys, "_MEIPASS", Path(__file__).parent)
    return Path(base) / relative


FRONTEND_DIR = resource_path("frontend")

mav = MavlinkService()
video = VideoService()


@asynccontextmanager
async def lifespan(app: FastAPI):
    yield                      # app runs here
    mav.disconnect()           # clean shutdown on Ctrl+C
    video.stop()


app = FastAPI(title="Drone GCS", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")


# ---------------- pages ----------------
@app.get("/")
def index():
    return FileResponse(FRONTEND_DIR / "index.html")


# ---------------- settings / ports ----------------
@app.get("/api/config")
def get_config():
    return {
        "conn_type": config.DEFAULT_CONN_TYPE,
        "serial_port": config.DEFAULT_SERIAL_PORT,
        "baud": config.DEFAULT_BAUD,
        "udp": config.DEFAULT_UDP,
        "tcp": config.DEFAULT_TCP,
        "video_type": config.DEFAULT_VIDEO_TYPE,
        "rtsp_url": config.DEFAULT_RTSP_URL,
        "udp_video_port": config.DEFAULT_UDP_VIDEO_PORT,
        "map_center": [config.SIM_HOME_LAT, config.SIM_HOME_LON],
    }


@app.get("/api/ports")
def get_ports():
    return list_serial_ports()


# ---------------- drone connection ----------------
# These are plain "def" (not async) so FastAPI runs them in a worker thread.
@app.post("/api/connect")
def connect(req: ConnectRequest):
    if req.conn_type not in ("sim", "serial", "udp", "tcp"):
        raise HTTPException(400, "Unknown connection type")
    if req.conn_type == "serial" and not req.serial_port:
        raise HTTPException(400, "Please choose a COM port")
    mav.connect(req)
    return mav.snapshot()["connection"]


@app.post("/api/disconnect")
def disconnect():
    mav.disconnect()
    return mav.snapshot()["connection"]


@app.get("/api/status")
def status():
    return mav.snapshot()


# ---------------- Phase 2: commands ----------------
@app.post("/api/command")
def command(req: CommandRequest):
    """Arm / disarm / takeoff / land / rtl / mode. The safety rules live in
    MavlinkService.send_command, so they apply no matter who calls this."""
    if req.command not in ("arm", "disarm", "takeoff", "land", "rtl", "mode"):
        raise HTTPException(400, "Unknown command")
    return mav.send_command(req.command, req.mode.strip().upper(), req.altitude)


# ---------------- video ----------------
@app.post("/api/video/start")
def video_start(req: VideoRequest):
    try:
        return video.start(req.source_type, req.value.strip())
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/api/video/stop")
def video_stop():
    video.stop()
    return video.get_status()


@app.get("/api/video/status")
def video_status():
    return video.get_status()


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(
        video.mjpeg_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store"})


# ---------------- live telemetry ----------------
@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    period = 1.0 / config.TELEMETRY_RATE_HZ
    try:
        while True:
            await ws.send_text(json.dumps(mav.snapshot()))
            await asyncio.sleep(period)
    except (WebSocketDisconnect, RuntimeError):
        pass


if __name__ == "__main__":
    url = f"http://localhost:{config.PORT}"
    print(f"\n  Drone GCS running at {url}   (Ctrl+C to stop)\n")
    threading.Timer(1.5, lambda: webbrowser.open(url)).start()
    uvicorn.run(app, host=config.HOST, port=config.PORT, log_level="info")