"""
mavlink_service.py - talks to the flight controller (or the simulator).

It runs in its OWN THREAD so it never blocks the web server or the video.
The thread keeps a Telemetry object up to date; the web server just reads it.

PHASE 2: commands. The web server never writes to the port itself. It puts a
command in a queue; THIS thread sends it and waits for the drone's COMMAND_ACK.
"""
import logging
import math
import os
import queue
import threading
import time

# Use MAVLink 2 (what modern ArduPilot uses). Must be set before importing mavutil.
os.environ.setdefault("MAVLINK20", "1")

import serial
from serial.tools import list_ports
from pymavlink import mavutil

import config
from backend.models import (ALLOWED_MODES, COPTER_MODES, ConnectRequest,
                            GPS_FIX_TEXT, Telemetry)
from backend.simulator import FakeDrone

log = logging.getLogger("mavlink")

# MAV_TYPE numbers that are multicopters
COPTER_TYPES = {2, 3, 4, 13, 14, 15, 29}
VEHICLE_NAMES = {1: "Plane", 2: "Quadcopter", 3: "Coaxial copter", 4: "Helicopter",
                 5: "Antenna tracker", 10: "Rover", 12: "Submarine",
                 13: "Hexacopter", 14: "Octocopter", 15: "Tricopter",
                 29: "Dodecacopter"}

MAV_RESULT_TEXT = {0: "accepted", 1: "temporarily rejected (try again)",
                   2: "denied", 3: "not supported by this firmware",
                   4: "failed", 5: "in progress", 6: "cancelled"}


def list_serial_ports() -> list:
    """Return the COM ports Windows (or Linux) can see."""
    ports = [{"device": p.device, "description": p.description}
             for p in list_ports.comports()]
    return sorted(ports, key=lambda p: p["device"])


class PendingCommand:
    """One command travelling through: queue -> sent -> acknowledged."""

    def __init__(self, kind, label, mav_command=None, params=None, mode="", alt=0.0):
        self.kind = kind                  # arm | disarm | takeoff | mode
        self.label = label                # text shown to the user
        self.mav_command = mav_command    # MAVLink command number
        self.params = params or [0] * 7   # the 7 command parameters
        self.mode = mode
        self.alt = alt
        self.created = time.time()
        self.sent_at = 0.0
        self.cancelled = False
        self.done = threading.Event()
        self.ok = False
        self.message = ""

    def finish(self, ok: bool, message: str) -> None:
        self.ok = ok
        self.message = message
        self.done.set()


class MavlinkService:
    def __init__(self):
        self._lock = threading.Lock()          # protects the data below
        self._telem = Telemetry()
        self._conn = {"state": "disconnected", "message": "Not connected",
                      "conn_type": "", "target": ""}
        self._thread = None
        self._stop = threading.Event()
        self._last_heartbeat = 0.0
        # --- Phase 2: commands ---
        self._cmd_lock = threading.Lock()      # one command at a time
        self._cmd_queue = queue.Queue()        # web thread -> mavlink thread
        self._pending = []                     # sent, waiting for ACK (mavlink thread only)

    # ------------------------------------------------------------------
    # Public API (called by main.py)
    # ------------------------------------------------------------------
    def connect(self, req: ConnectRequest) -> None:
        self.disconnect()                      # stop any previous connection
        stop = threading.Event()
        self._stop = stop
        with self._lock:
            self._telem = Telemetry()
            self._conn = {"state": "connecting", "message": "Starting...",
                          "conn_type": req.conn_type,
                          "target": self._target_text(req)}
        self._thread = threading.Thread(
            target=self._run, args=(req, stop), daemon=True, name="mavlink")
        self._thread.start()

    def disconnect(self) -> None:
        self._stop.set()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=3)
        self._thread = None
        self._abort_commands("connection closed.")
        with self._lock:
            self._telem = Telemetry()
            self._conn = {"state": "disconnected", "message": "Disconnected",
                          "conn_type": "", "target": ""}

    def snapshot(self) -> dict:
        """Everything the browser needs, as a plain dict."""
        with self._lock:
            return {"telemetry": self._telem.to_dict(),
                    "connection": dict(self._conn),
                    "time": time.time()}

    # ------------------------------------------------------------------
    # Phase 2: commands (called from the web server thread)
    # ------------------------------------------------------------------
    def send_command(self, kind: str, mode: str = "", alt: float = 0.0) -> dict:
        """Check safety rules, queue the command, wait for the drone's answer."""
        with self._cmd_lock:                   # never two commands at once
            error = self._check_allowed(kind, mode, alt)
            if error:
                return {"ok": False, "message": error}
            cmd = self._make_command(kind, mode, alt)
            self._cmd_queue.put(cmd)
            if not cmd.done.wait(config.COMMAND_TIMEOUT_S + 2.0):
                cmd.cancelled = True           # never send it late!
                return {"ok": False,
                        "message": "No answer. Command cancelled - check the connection."}
            return {"ok": cmd.ok, "message": cmd.message}

    def _check_allowed(self, kind: str, mode: str, alt: float):
        """SAFETY RULES (enforced here, not just in the buttons).
        Returns an error text, or None when the command is allowed."""
        with self._lock:
            t = self._telem
            state = self._conn["state"]
            link_ok, armed = t.link_ok, t.armed
            alt_rel, cur_mode, is_copter = t.alt_rel, t.mode, t.is_copter

        if state != "connected":
            return "Not connected to a drone."
        if not link_ok:
            return "Link lost - command NOT sent."
        if not is_copter:
            return "Commands are only enabled for multicopters in this version."

        if kind == "arm":
            if armed:
                return "Already armed."
        elif kind == "disarm":
            if not armed:
                return "Already disarmed."
            if alt_rel > 1.0:
                return ("Refused: the drone looks airborne (%.1f m). "
                        "Use LAND instead of disarming." % alt_rel)
        elif kind == "takeoff":
            if not armed:
                return "Arm the drone first."
            if cur_mode != "GUIDED":
                return "Takeoff needs GUIDED mode. Set GUIDED, arm, then take off."
            if not (config.MIN_TAKEOFF_ALT_M <= alt <= config.MAX_TAKEOFF_ALT_M):
                return "Takeoff altitude must be %.0f to %.0f m." % (
                    config.MIN_TAKEOFF_ALT_M, config.MAX_TAKEOFF_ALT_M)
            if alt_rel > 2.0:
                return "The drone is already flying."
        elif kind in ("land", "rtl"):
            if not armed:
                return "The drone is not armed."
        elif kind == "mode":
            if mode not in ALLOWED_MODES:
                return "Mode %s is not allowed from the app." % (mode or "?")
            if mode == cur_mode:
                return "Already in %s mode." % mode
        else:
            return "Unknown command."
        return None

    @staticmethod
    def _make_command(kind: str, mode: str, alt: float) -> PendingCommand:
        m = mavutil.mavlink
        if kind == "arm":
            return PendingCommand("arm", "Arm", m.MAV_CMD_COMPONENT_ARM_DISARM,
                                  [1, 0, 0, 0, 0, 0, 0])
        if kind == "disarm":
            return PendingCommand("disarm", "Disarm", m.MAV_CMD_COMPONENT_ARM_DISARM,
                                  [0, 0, 0, 0, 0, 0, 0])
        if kind == "takeoff":
            return PendingCommand("takeoff", "Takeoff to %g m" % alt,
                                  m.MAV_CMD_NAV_TAKEOFF,
                                  [0, 0, 0, 0, 0, 0, alt], alt=alt)
        # land / rtl / mode are all "set flight mode" commands
        name = {"land": "LAND", "rtl": "RTL"}.get(kind, mode)
        return PendingCommand("mode", "Mode " + name, m.MAV_CMD_DO_SET_MODE,
                              [m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                               COPTER_MODES[name], 0, 0, 0, 0, 0], mode=name)

    # ------------------------------------------------------------------
    # Phase 2: commands (run inside the MAVLink thread)
    # ------------------------------------------------------------------
    def _process_commands(self, master, fake=None) -> None:
        """Send queued commands, and give up on ones that never got an ACK."""
        while True:
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            if cmd.cancelled or time.time() - cmd.created > config.COMMAND_TIMEOUT_S:
                cmd.finish(False, cmd.label + ": expired before sending.")
                continue
            if fake is not None:                       # simulation mode
                self._finish_with_code(cmd, fake.handle_command(cmd.kind, cmd.mode, cmd.alt))
            else:                                      # real drone
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    cmd.mav_command, 0, *cmd.params)
                cmd.sent_at = time.time()
                self._pending.append(cmd)

        # commands sent but never acknowledged
        now = time.time()
        for cmd in list(self._pending):
            if now - cmd.sent_at < config.COMMAND_TIMEOUT_S:
                continue
            self._pending.remove(cmd)
            with self._lock:
                armed, mode = self._telem.armed, self._telem.mode
            worked = ((cmd.kind == "arm" and armed) or
                      (cmd.kind == "disarm" and not armed) or
                      (cmd.kind == "mode" and mode == cmd.mode))
            if worked:
                cmd.finish(True, cmd.label + ": done (no ACK, but the drone reports it).")
            else:
                cmd.finish(False, cmd.label + ": no reply from the drone. Check the "
                           "HUD and messages before trying again.")

    @staticmethod
    def _finish_with_code(cmd: PendingCommand, code: int) -> None:
        word = MAV_RESULT_TEXT.get(code, "result %s" % code)
        if code == 0:
            cmd.finish(True, "%s: %s by the drone." % (cmd.label, word))
        else:
            cmd.finish(False, "%s: %s. See 'Drone messages' for the reason." % (cmd.label, word))

    def _on_ack(self, msg) -> None:
        """A COMMAND_ACK arrived: match it to the command we sent."""
        for cmd in list(self._pending):
            if cmd.mav_command != msg.command:
                continue
            if msg.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                cmd.sent_at = time.time()              # drone is busy, keep waiting
                return
            self._pending.remove(cmd)
            self._finish_with_code(cmd, msg.result)
            return

    def _abort_commands(self, text: str) -> None:
        """Fail every waiting command (link closed, disconnected, ...)."""
        while True:
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            cmd.finish(False, "%s: %s" % (cmd.label, text))
        for cmd in self._pending:
            cmd.finish(False, "%s: %s" % (cmd.label, text))
        self._pending = []

    def _push_message(self, severity: int, text: str) -> None:
        """Add a line to the message log. The caller must hold self._lock."""
        t = self._telem
        t.messages.append({"time": time.strftime("%H:%M:%S"),
                           "sev": int(severity), "text": text})
        del t.messages[:-config.MESSAGE_LOG_SIZE]
        t.status_text = text

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    @staticmethod
    def _target_text(req: ConnectRequest) -> str:
        if req.conn_type == "serial":
            return f"{req.serial_port} @ {req.baud}"
        if req.conn_type == "udp":
            return req.udp
        if req.conn_type == "tcp":
            return req.tcp
        return "built-in simulator"

    def _set_conn(self, stop: threading.Event, state: str, message: str) -> None:
        """Update the status text, but only if this thread is still the active one."""
        with self._lock:
            if self._stop is stop and not stop.is_set():
                self._conn["state"] = state
                self._conn["message"] = message

    def _run(self, req: ConnectRequest, stop: threading.Event) -> None:
        if req.conn_type == "sim":
            try:
                self._run_sim(stop)
            finally:
                self._abort_commands("simulation stopped.")
            return

        target = self._target_text(req)
        # Outer loop = auto-reconnect. It keeps trying until you press Disconnect.
        while not stop.is_set():
            master = None
            try:
                self._set_conn(stop, "connecting", f"Opening {target} ...")
                master = self._open(req)

                self._set_conn(stop, "connecting",
                               f"Waiting for heartbeat on {target} ...")
                if self._wait_heartbeat(master, stop):
                    self._last_heartbeat = time.time()
                    self._request_streams(master)
                    self._set_conn(
                        stop, "connected",
                        f"Connected to {target} (system {master.target_system})")
                    self._read_loop(master, stop)
                elif not stop.is_set():
                    self._set_conn(
                        stop, "error",
                        "No heartbeat received. Check: COM port, baud rate (57600 "
                        "for SiK), drone powered, both radios paired.")
            except serial.SerialException as e:
                self._set_conn(
                    stop, "error",
                    f"Cannot open {req.serial_port}: {e}. Port busy (Mission "
                    "Planner open?) or wrong COM number?")
            except OSError as e:
                self._set_conn(stop, "error", f"Connection problem: {e}")
            except Exception as e:  # never let the thread die silently
                log.exception("MAVLink thread error")
                self._set_conn(stop, "error", f"Unexpected error: {e}")
            finally:
                if master is not None:
                    try:
                        master.close()
                    except Exception:
                        pass
                self._abort_commands("link closed.")
                with self._lock:
                    self._telem.link_ok = False

            stop.wait(config.RECONNECT_DELAY)   # pause, then retry

    def _run_sim(self, stop: threading.Event) -> None:
        fake = FakeDrone()
        self._set_conn(stop, "connected", "Simulation mode: fake telemetry")
        while not stop.is_set():
            self._process_commands(None, fake)
            with self._lock:
                fake.step(self._telem)
                for severity, text in fake.drain():
                    self._push_message(severity, text)
            stop.wait(1.0 / config.SIM_RATE_HZ)

    @staticmethod
    def _open(req: ConnectRequest):
        # source_system=255 / component=190 is how ground stations identify themselves
        if req.conn_type == "serial":
            return mavutil.mavlink_connection(
                req.serial_port, baud=req.baud,
                source_system=255, source_component=190)
        address = req.udp if req.conn_type == "udp" else req.tcp
        return mavutil.mavlink_connection(
            address, source_system=255, source_component=190)

    @staticmethod
    def _wait_heartbeat(master, stop: threading.Event) -> bool:
        deadline = time.time() + config.HEARTBEAT_WAIT
        while time.time() < deadline and not stop.is_set():
            msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is None:
                continue
            # Ignore heartbeats from other ground stations / companion computers
            if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                master.target_system = msg.get_srcSystem()
                master.target_component = msg.get_srcComponent()
                return True
        return False

    @staticmethod
    def _request_streams(master) -> None:
        """Ask the flight controller to send the data we need, at sensible rates."""
        ts, tc = master.target_system, master.target_component
        m = mavutil.mavlink

        # Method 1 (old, works everywhere): request whole "streams" at X Hz.
        for stream_id, hz in [(m.MAV_DATA_STREAM_EXTENDED_STATUS, 2),  # battery, GPS
                              (m.MAV_DATA_STREAM_POSITION, 5),          # position
                              (m.MAV_DATA_STREAM_EXTRA1, 5),            # attitude
                              (m.MAV_DATA_STREAM_EXTRA2, 5)]:           # VFR_HUD
            master.mav.request_data_stream_send(ts, tc, stream_id, hz, 1)

        # Method 2 (newer): request individual messages with SET_MESSAGE_INTERVAL.
        set_interval = getattr(m, "MAV_CMD_SET_MESSAGE_INTERVAL", 511)
        for msg_id, hz in [(1, 2),     # SYS_STATUS (battery)
                           (24, 2),    # GPS_RAW_INT (fix, satellites)
                           (30, 5),    # ATTITUDE
                           (33, 5),    # GLOBAL_POSITION_INT
                           (74, 5)]:   # VFR_HUD
            master.mav.command_long_send(ts, tc, set_interval, 0,
                                         msg_id, int(1_000_000 / hz), 0, 0, 0, 0, 0)
            time.sleep(0.03)

        # Ask for the home position (message 242) once.
        request_msg = getattr(m, "MAV_CMD_REQUEST_MESSAGE", 512)
        master.mav.command_long_send(ts, tc, request_msg, 0, 242, 0, 0, 0, 0, 0, 0)

    @staticmethod
    def _send_gcs_heartbeat(master) -> None:
        try:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                      0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
        except Exception:
            pass

    def _read_loop(self, master, stop: threading.Event) -> None:
        last_gcs_hb = 0.0
        link_was_ok = True
        while not stop.is_set():
            now = time.time()
            if now - last_gcs_hb >= 1.0:       # we send our own heartbeat at 1 Hz
                self._send_gcs_heartbeat(master)
                last_gcs_hb = now

            msg = master.recv_match(blocking=True, timeout=0.2)
            if msg is not None:
                self._handle(msg, master)

            self._process_commands(master)     # Phase 2: send queued commands

            # --- link status from heartbeat age ---
            silent_for = time.time() - self._last_heartbeat
            link_ok = silent_for < config.HEARTBEAT_TIMEOUT
            with self._lock:
                self._telem.link_ok = link_ok
            if link_ok and not link_was_ok:
                self._request_streams(master)   # FC may have rebooted: ask again
                self._set_conn(stop, "connected", "Link restored")
            elif not link_ok and link_was_ok:
                self._set_conn(stop, "connected",
                               "LINK LOST - no heartbeat, waiting...")
            link_was_ok = link_ok

            if silent_for > config.RECONNECT_AFTER_S:
                self._set_conn(stop, "connecting", "Link dead, reopening port...")
                return                          # outer loop reconnects

    def _handle(self, msg, master) -> None:
        """Copy values from a MAVLink message into our Telemetry object."""
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            return
        if msg.get_srcSystem() != master.target_system:
            return                              # not from our drone

        with self._lock:
            t = self._telem

            if mtype == "HEARTBEAT":
                if (msg.get_srcComponent() != master.target_component or
                        msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                    return
                self._last_heartbeat = time.time()
                t.armed = bool(msg.base_mode &
                               mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                t.mode = master.flightmode or mavutil.mode_string_v10(msg)
                t.vehicle_type = VEHICLE_NAMES.get(msg.type, "Vehicle type %d" % msg.type)
                t.is_copter = msg.type in COPTER_TYPES

            elif mtype == "GLOBAL_POSITION_INT":
                t.lat = msg.lat / 1e7
                t.lon = msg.lon / 1e7
                t.alt_msl = msg.alt / 1000.0
                t.alt_rel = msg.relative_alt / 1000.0
                t.has_position = not (msg.lat == 0 and msg.lon == 0)
                if msg.hdg != 65535:
                    t.heading = msg.hdg / 100.0
                # Fallback home: first good GPS position (replaced by HOME_POSITION)
                if (t.home_lat is None and t.has_position and t.gps_fix >= 3):
                    t.home_lat, t.home_lon = t.lat, t.lon

            elif mtype == "VFR_HUD":
                t.groundspeed = msg.groundspeed
                t.climb = msg.climb
                t.heading = float(msg.heading)

            elif mtype == "ATTITUDE":
                t.roll = math.degrees(msg.roll)
                t.pitch = math.degrees(msg.pitch)

            elif mtype == "SYS_STATUS":
                if msg.voltage_battery != 65535:
                    t.battery_voltage = msg.voltage_battery / 1000.0
                t.battery_percent = msg.battery_remaining   # -1 = unknown

            elif mtype == "GPS_RAW_INT":
                t.gps_fix = msg.fix_type
                t.gps_fix_text = GPS_FIX_TEXT.get(msg.fix_type, "Unknown")
                t.satellites = 0 if msg.satellites_visible == 255 else msg.satellites_visible
                t.hdop = 0.0 if msg.eph == 65535 else msg.eph / 100.0

            elif mtype == "HOME_POSITION":
                t.home_lat = msg.latitude / 1e7
                t.home_lon = msg.longitude / 1e7

            elif mtype == "COMMAND_ACK":
                self._on_ack(msg)               # Phase 2

            elif mtype == "STATUSTEXT":
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode(errors="ignore")
                text = str(text).strip("\x00").strip()
                if text:
                    self._push_message(msg.severity, text)