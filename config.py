"""
config.py - ALL settings in one place.
Change values here (or in a .env file) instead of hunting through the code.
"""
import os
from dotenv import load_dotenv

load_dotenv()  # reads an optional .env file in the project folder


def _get(name: str, default: str) -> str:
    return os.getenv(name, default)


# ---------- Web server ----------
HOST = _get("GCS_HOST", "127.0.0.1")
PORT = int(_get("GCS_PORT", "8000"))

# ---------- Connection defaults (shown in the UI) ----------
DEFAULT_CONN_TYPE = _get("GCS_CONN_TYPE", "sim")        # sim | serial | udp | tcp
DEFAULT_SERIAL_PORT = _get("GCS_SERIAL_PORT", "COM5")   # Linux: /dev/ttyUSB0
DEFAULT_BAUD = int(_get("GCS_BAUD", "57600"))
DEFAULT_UDP = _get("GCS_UDP", "udp:127.0.0.1:14550")    # ArduPilot SITL default
DEFAULT_TCP = _get("GCS_TCP", "tcp:127.0.0.1:5760")

# ---------- Telemetry timing ----------
TELEMETRY_RATE_HZ = 10        # how often the browser gets updates
HEARTBEAT_TIMEOUT = 3.0       # seconds without heartbeat => "LINK LOST"
HEARTBEAT_WAIT = 5.0          # seconds to wait for the first heartbeat
RECONNECT_AFTER_S = 10.0      # seconds of silence before reopening the port
RECONNECT_DELAY = 3.0         # pause between reconnect attempts

# ---------- Simulation mode (fake drone) ----------
SIM_HOME_LAT = -35.363261     # same default spot as ArduPilot SITL
SIM_HOME_LON = 149.165230
SIM_RADIUS_M = 120.0          # radius of the circle the fake drone flies
SIM_SPEED_MS = 8.0            # fake ground speed (m/s)
SIM_RATE_HZ = 10

# ---------- Video ----------
DEFAULT_VIDEO_TYPE = "test"                       # test | webcam | rtsp | udp
DEFAULT_RTSP_URL = _get("GCS_RTSP_URL", "rtsp://192.168.1.10:554/stream")
DEFAULT_UDP_VIDEO_PORT = int(_get("GCS_UDP_VIDEO_PORT", "5600"))
VIDEO_MAX_WIDTH = 960         # frames wider than this are shrunk (less lag)
VIDEO_JPEG_QUALITY = 70       # 1-100, lower = faster and blockier
# ---------- Phase 2: commands ----------
COMMAND_TIMEOUT_S = 5.0       # how long we wait for the drone's reply (ACK)
MIN_TAKEOFF_ALT_M = 1.0       # safety limits for the Takeoff button
MAX_TAKEOFF_ALT_M = 30.0
MESSAGE_LOG_SIZE = 25         # how many drone messages the panel keeps
SIM_CLIMB_MS = 2.5            # fake drone climb speed (m/s)
SIM_AUTO_DISARM_S = 10.0      # fake drone disarms itself if you do not take off
# ---------- Phase 2B: calibration ----------
CAL_COMMAND_TIMEOUT_S = 15.0  # accel/level start with a gyro calibration, which can take a few seconds
SIM_COMPASS_RATES = (5.0, 4.0)  # fake compass progress in % per second (compass 0 and 1)
SIM_ACCEL_SAMPLE_S = 2.5      # fake drone "samples" each position for this long


# ---------- Phase 2C: mission planning ----------
MAX_WAYPOINTS = 60            # most waypoints allowed in one mission
MIN_WP_ALT_M = 2.0            # waypoint altitude limits (metres above home)
MAX_WP_ALT_M = 120.0
MIN_WP_SPEED_MS = 0.5         # waypoint speed limits (0 means "leave the speed as it is")
MAX_WP_SPEED_MS = 15.0
MAX_WP_DISTANCE_M = 2000.0    # farthest a waypoint may be from home
MIN_MISSION_BATTERY_PCT = 25  # refuse to start a mission below this battery level
MISSION_TIMEOUT_S = 20.0      # base time allowed for an upload/download (plus 1 s per item)
MISSION_RETRY_S = 1.5         # resend a message if the drone has not answered in this time
MISSION_MAX_RETRIES = 5
SIM_WP_SPEED_MS = 10.0        # fake drone speed on a mission when no speed is set
SIM_AUTO_DISARM_S = 30.0      # overrides the 10 s above: gives you time to confirm the Start dialog