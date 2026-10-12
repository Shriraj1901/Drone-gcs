"""
models.py - the "shapes" of the data passed around the app.
"""
from dataclasses import dataclass, asdict, field
from typing import Optional

from pydantic import BaseModel

import config

GPS_FIX_TEXT = {
    0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix",
    4: "DGPS", 5: "RTK Float", 6: "RTK Fixed", 7: "Static", 8: "PPP",
}

# Flight-mode numbers used inside ArduCopter (the drone only understands numbers).
COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
    "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9, "POSHOLD": 16, "BRAKE": 17,
}

# Modes the "Set mode" dropdown may request (Land and RTL have their own buttons).
ALLOWED_MODES = ("STABILIZE", "ALT_HOLD", "LOITER", "POSHOLD", "GUIDED", "AUTO")


@dataclass
class Telemetry:
    """Latest known drone state. Sent to the browser ~10 times per second."""
    has_position: bool = False
    lat: float = 0.0
    lon: float = 0.0
    alt_rel: float = 0.0          # metres above home
    alt_msl: float = 0.0          # metres above sea level
    groundspeed: float = 0.0      # m/s
    climb: float = 0.0            # m/s
    heading: float = 0.0          # degrees, 0 = North
    roll: float = 0.0
    pitch: float = 0.0
    battery_voltage: float = 0.0  # volts
    battery_percent: int = -1     # -1 = unknown
    gps_fix: int = 0
    gps_fix_text: str = "No GPS"
    satellites: int = 0
    hdop: float = 0.0
    mode: str = "---"
    armed: bool = False
    link_ok: bool = False
    home_lat: Optional[float] = None
    home_lon: Optional[float] = None
    status_text: str = ""         # last message from the flight controller
    # ---- Phase 2 ----
    vehicle_type: str = "Unknown"
    is_copter: bool = False
    messages: list = field(default_factory=list)   # recent drone messages

    def to_dict(self) -> dict:
        return asdict(self)


class ConnectRequest(BaseModel):
    """What the browser sends when you press Connect."""
    conn_type: str = config.DEFAULT_CONN_TYPE
    serial_port: str = config.DEFAULT_SERIAL_PORT
    baud: int = config.DEFAULT_BAUD
    udp: str = config.DEFAULT_UDP
    tcp: str = config.DEFAULT_TCP


class VideoRequest(BaseModel):
    """What the browser sends when you press Start video."""
    source_type: str = config.DEFAULT_VIDEO_TYPE
    value: str = ""


class CommandRequest(BaseModel):
    """What the browser sends when you confirm a command."""
    command: str                  # arm | disarm | takeoff | land | rtl | mode
    mode: str = ""                # only used by "mode"
    altitude: float = 0.0         # only used by "takeoff" (metres)