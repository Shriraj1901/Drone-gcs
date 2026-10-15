"""
mavlink_service.py - talks to the flight controller (or the simulator).

It runs in its OWN THREAD so it never blocks the web server or the video.
The thread keeps a Telemetry object up to date; the web server just reads it.

PHASE 2A: commands. The web server never writes to the port itself. It puts a
command in a queue; THIS thread sends it and waits for the drone's COMMAND_ACK.

PHASE 2B: calibration. Compass, accelerometer and level calibration use the same
command path. While a calibration runs, messages from the drone (MAG_CAL_PROGRESS,
MAG_CAL_REPORT, and position requests) update Telemetry.calibration, which the
browser shows live.

PHASE 2C: missions. Uploading / downloading waypoints is a longer conversation with the
drone; it lives in mission_service.py (MissionManager) and is plugged in below.
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
from backend.models import (ACCEL_FAILED, ACCEL_POSITIONS, ACCEL_SUCCESS, ALLOWED_MODES,
                            CAL_ACTIVE, COPTER_MODES, MSG_COMPASS, MSG_SAMPLING,
                            ConnectRequest, GPS_FIX_TEXT, Telemetry, accel_message,
                            blank_compass, new_calibration)
from backend.mission_service import MissionManager
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

# ---- Phase 2B: calibration constants ----
CAL_START_KINDS = ("cal_compass", "cal_accel", "cal_level")
CAL_CANCEL_KINDS = ("cal_cancel_compass", "cal_cancel_accel")
FLIGHT_KINDS = ("arm", "disarm", "takeoff", "land", "rtl", "mode")
CMD_START_MAG_CAL = 42424        # MAV_CMD_DO_START_MAG_CAL
CMD_CANCEL_MAG_CAL = 42426       # MAV_CMD_DO_CANCEL_MAG_CAL
CMD_ACCELCAL_POS = 42429         # MAV_CMD_ACCELCAL_VEHICLE_POS
MAG_STATUS_TEXT = {0: "Not started", 1: "Waiting", 2: "Step 1", 3: "Step 2",
                   4: "Success", 5: "Failed", 6: "Bad orientation", 7: "Bad radius"}


def list_serial_ports() -> list:
    """Return the COM ports Windows (or Linux) can see."""
    ports = [{"device": p.device, "description": p.description}
             for p in list_ports.comports()]
    return sorted(ports, key=lambda p: p["device"])


class PendingCommand:
    """One command travelling through: queue -> sent -> acknowledged."""

    def __init__(self, kind, label, mav_command=None, params=None, mode="",
                 alt=0.0, value=0.0, timeout=None):
        self.kind = kind                  # arm | disarm | takeoff | mode | cal_* ...
        self.label = label                # text shown to the user
        self.mav_command = mav_command    # MAVLink command number
        self.params = params or [0] * 7   # the 7 command parameters
        self.mode = mode
        self.alt = alt
        self.value = value                # extra number (accel position)
        self.timeout = timeout or config.COMMAND_TIMEOUT_S
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
        # RLock: the same thread may take this lock twice (handlers call each other).
        self._lock = threading.RLock()
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
        # --- Phase 2C: missions ---
        self.missions = MissionManager(self)

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
        self.missions.abort("connection closed.")
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
    # Commands (called from the web server thread)
    # ------------------------------------------------------------------
    def send_command(self, kind: str, mode: str = "", alt: float = 0.0) -> dict:
        """Check safety rules, queue the command, wait for the drone's answer."""
        with self._cmd_lock:                   # never two commands at once
            value = 0.0
            label = None
            # "Start mission" = the mission safety checks, then a normal switch to AUTO
            if kind == "mission_start":
                problem = self.missions.check_start()
                if problem:
                    return {"ok": False, "message": problem}
                kind, mode, label = "mode", "AUTO", "Start mission (AUTO)"
            # "cal_cancel" and "accel_ready" depend on what the calibration is doing now
            elif kind == "cal_cancel":
                kind = self._resolve_cancel()
                if kind is None:
                    return {"ok": False, "message": "No calibration is running."}
            elif kind == "accel_ready":
                value, problem = self._accel_position_to_confirm()
                if problem:
                    return {"ok": False, "message": problem}

            error = self._check_allowed(kind, mode, alt)
            if error:
                return {"ok": False, "message": error}
            cmd = self._make_command(kind, mode, alt, value)
            if label:
                cmd.label = label
            self._cmd_queue.put(cmd)
            if not cmd.done.wait(cmd.timeout + 2.0):
                cmd.cancelled = True           # never send it late!
                return {"ok": False,
                        "message": "No answer. Command cancelled - check the connection."}
            return {"ok": cmd.ok, "message": cmd.message}

    # ---- Phase 2C: missions (the work is done in mission_service.py) ----
    def upload_mission(self, takeoff_alt: float, finish: str, waypoints: list) -> dict:
        return self.missions.upload(takeoff_alt, finish, waypoints)

    def download_mission(self) -> dict:
        return self.missions.download()

    def clear_mission(self) -> dict:
        return self.missions.clear()

    def _resolve_cancel(self):
        with self._lock:
            c = self._telem.calibration
            if c["state"] not in CAL_ACTIVE:
                return None
            return "cal_cancel_compass" if c["kind"] == "compass" else "cal_cancel_accel"

    def _accel_position_to_confirm(self):
        """Which position is the drone waiting for? Returns (position, error_text)."""
        with self._lock:
            c = self._telem.calibration
            if c["kind"] != "accel" or c["state"] != "waiting_position":
                return 0, "The drone is not asking for a position right now."
            return c["position"], None

    def _check_allowed(self, kind: str, mode: str, alt: float):
        """SAFETY RULES (enforced here, not just in the buttons).
        Returns an error text, or None when the command is allowed."""
        with self._lock:
            t = self._telem
            state = self._conn["state"]
            link_ok, armed = t.link_ok, t.armed
            alt_rel, cur_mode, is_copter = t.alt_rel, t.mode, t.is_copter
            cal_busy = t.calibration["state"] in CAL_ACTIVE

        if state != "connected":
            return "Not connected to a drone."
        if not link_ok:
            return "Link lost - command NOT sent."
        if kind in FLIGHT_KINDS and not is_copter:
            return "Flight commands are only enabled for multicopters in this version."

        if kind == "arm":
            if armed:
                return "Already armed."
            if cal_busy:
                return "A calibration is running. Finish or cancel it before arming."
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
        # ---- Phase 2B ----
        elif kind in CAL_START_KINDS:
            if armed or alt_rel > 1.0:
                return "Calibration is only allowed while DISARMED and on the ground."
            if cal_busy:
                return "A calibration is already running. Finish or cancel it first."
        elif kind in CAL_CANCEL_KINDS or kind == "accel_ready":
            pass                               # already checked in send_command
        elif kind == "reboot":
            if armed or alt_rel > 1.0:
                return "Refused: never reboot the flight controller while armed or airborne."
            if cal_busy:
                return "Finish or cancel the calibration first."
        else:
            return "Unknown command."
        return None

    @staticmethod
    def _make_command(kind: str, mode: str, alt: float, value: float = 0.0) -> PendingCommand:
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

        # ---- Phase 2B ----
        # Compass: p1 = which compasses (0 = all), p2 = retry (0), p3 = autosave (1),
        #          p4 = delay (0), p5 = auto-reboot (0: we ask you to reboot)
        if kind == "cal_compass":
            return PendingCommand(
                kind, "Compass calibration",
                getattr(m, "MAV_CMD_DO_START_MAG_CAL", CMD_START_MAG_CAL),
                [0, 0, 1, 0, 0, 0, 0], timeout=config.CAL_COMMAND_TIMEOUT_S)
        # Accelerometer: PREFLIGHT_CALIBRATION with param5 = 1 (full 6-position)
        if kind == "cal_accel":
            return PendingCommand(
                kind, "Accelerometer calibration", m.MAV_CMD_PREFLIGHT_CALIBRATION,
                [0, 0, 0, 0, 1, 0, 0], timeout=config.CAL_COMMAND_TIMEOUT_S)
        # Level: PREFLIGHT_CALIBRATION with param5 = 2 (board level)
        if kind == "cal_level":
            return PendingCommand(
                kind, "Level calibration", m.MAV_CMD_PREFLIGHT_CALIBRATION,
                [0, 0, 0, 0, 2, 0, 0], timeout=config.CAL_COMMAND_TIMEOUT_S)
        if kind == "cal_cancel_compass":
            return PendingCommand(
                kind, "Cancel compass calibration",
                getattr(m, "MAV_CMD_DO_CANCEL_MAG_CAL", CMD_CANCEL_MAG_CAL),
                [0, 0, 0, 0, 0, 0, 0])
        if kind == "cal_cancel_accel":      # all zeros = "stop what you are doing"
            return PendingCommand(
                kind, "Cancel accelerometer calibration",
                m.MAV_CMD_PREFLIGHT_CALIBRATION, [0, 0, 0, 0, 0, 0, 0])
        # "Ready" button: tell the drone it is now in the position it asked for
        if kind == "accel_ready":
            return PendingCommand(
                kind, "Position %d confirmed" % int(value),
                getattr(m, "MAV_CMD_ACCELCAL_VEHICLE_POS", CMD_ACCELCAL_POS),
                [value, 0, 0, 0, 0, 0, 0], value=value)
        if kind == "reboot":
            return PendingCommand(
                kind, "Reboot flight controller",
                m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, [1, 0, 0, 0, 0, 0, 0])

        # land / rtl / mode are all "set flight mode" commands
        name = {"land": "LAND", "rtl": "RTL"}.get(kind, mode)
        return PendingCommand("mode", "Mode " + name, m.MAV_CMD_DO_SET_MODE,
                              [m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                               COPTER_MODES[name], 0, 0, 0, 0, 0], mode=name)

    # ------------------------------------------------------------------
    # Commands (run inside the MAVLink thread)
    # ------------------------------------------------------------------
    def _process_commands(self, master, fake=None) -> None:
        """Send queued commands, and give up on ones that never got an ACK."""
        while True:
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            if cmd.cancelled or time.time() - cmd.created > cmd.timeout:
                cmd.finish(False, cmd.label + ": expired before sending.")
                continue
            if fake is not None:                       # simulation mode
                code = fake.handle_command(cmd.kind, cmd.mode, cmd.alt, cmd.value)
                self._finish_with_code(cmd, code, sim=True)
            else:                                      # real drone
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    cmd.mav_command, 0, *cmd.params)
                cmd.sent_at = time.time()
                self._pending.append(cmd)
                if cmd.kind in CAL_START_KINDS:
                    with self._lock:
                        self._telem.calibration = new_calibration(
                            cmd.kind[4:], "starting", "Waiting for the drone to accept...")

        # commands sent but never acknowledged
        now = time.time()
        for cmd in list(self._pending):
            if now - cmd.sent_at < cmd.timeout:
                continue
            self._pending.remove(cmd)
            with self._lock:
                armed, mode = self._telem.armed, self._telem.mode
            worked = ((cmd.kind == "arm" and armed) or
                      (cmd.kind == "disarm" and not armed) or
                      (cmd.kind == "mode" and mode == cmd.mode))
            if cmd.kind == "reboot":
                cmd.finish(True, "Reboot command sent. The link will drop and reconnect.")
            elif worked:
                cmd.finish(True, cmd.label + ": done (no ACK, but the drone reports it).")
            else:
                if cmd.kind in CAL_START_KINDS:
                    with self._lock:
                        self._telem.calibration = new_calibration(
                            cmd.kind[4:], "failed",
                            "The drone did not answer the calibration command.")
                cmd.finish(False, cmd.label + ": no reply from the drone. Check the "
                           "HUD and messages before trying again.")

    def _finish_with_code(self, cmd: PendingCommand, code: int, sim: bool = False) -> None:
        word = MAV_RESULT_TEXT.get(code, "result %s" % code)
        if not sim:
            self._cal_on_result(cmd, code)             # update the calibration status
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
    # Phase 2B: calibration status (the caller holds self._lock where noted)
    # ------------------------------------------------------------------
    def _cal_on_result(self, cmd: PendingCommand, code: int) -> None:
        """The drone answered a calibration-related command."""
        with self._lock:
            t = self._telem
            c = t.calibration
            if cmd.kind in CAL_START_KINDS:
                if code != 0:
                    t.calibration = new_calibration(
                        cmd.kind[4:], "failed",
                        "The drone did not accept the calibration. "
                        "Read 'Drone messages' for the reason.")
                elif cmd.kind == "cal_level":
                    t.calibration = new_calibration(
                        "level", "success",
                        "Level calibration done. The drone now treats this position as level.")
                elif c["state"] == "starting":         # compass or accel accepted
                    c["state"] = "running"
                    c["message"] = (MSG_COMPASS if cmd.kind == "cal_compass" else
                                    "Keep the drone still: calibrating gyros...")
            elif cmd.kind in CAL_CANCEL_KINDS:
                t.calibration = new_calibration("", "idle", "Calibration cancelled.")
            elif cmd.kind == "accel_ready" and code == 0:
                if c["kind"] == "accel" and c["state"] == "waiting_position":
                    c["state"] = "running"
                    c["confirmed"] = int(cmd.value)
                    c["message"] = MSG_SAMPLING

    def _cal_fail_if_active(self, text: str) -> None:
        with self._lock:
            c = self._telem.calibration
            if c["state"] in CAL_ACTIVE:
                self._telem.calibration = new_calibration(c["kind"], "failed", text)
                self._push_message(4, text)

    def _cal_mag_progress(self, msg) -> None:
        """MAG_CAL_PROGRESS: percent done for one compass. (caller holds the lock)"""
        t = self._telem
        c = t.calibration
        status = int(msg.cal_status)
        if c["kind"] != "compass" or c["state"] not in CAL_ACTIVE:
            if status not in (1, 2, 3):
                return                      # stray message after we finished: ignore
            c = t.calibration = new_calibration("compass", "running", MSG_COMPASS)
        ids = [i for i in range(8) if int(msg.cal_mask) & (1 << i)]
        if ids:
            c["expected"] = ids
        for i in c["expected"]:             # show a bar for every compass right away
            c["compasses"].setdefault(str(i), blank_compass())
        comp = c["compasses"].setdefault(str(int(msg.compass_id)), blank_compass())
        comp["pct"] = int(msg.completion_pct)
        comp["status"] = MAG_STATUS_TEXT.get(status, "")
        c["state"] = "running"
        c["message"] = MSG_COMPASS

    def _cal_mag_report(self, msg) -> None:
        """MAG_CAL_REPORT: final result for one compass. (caller holds the lock)"""
        t = self._telem
        c = t.calibration
        if c["kind"] != "compass" or c["state"] not in CAL_ACTIVE:
            c = t.calibration = new_calibration("compass", "running", MSG_COMPASS)
        status = int(msg.cal_status)
        cid = str(int(msg.compass_id))
        comp = c["compasses"].setdefault(cid, blank_compass())
        comp["done"] = True
        comp["ok"] = (status == 4)
        comp["fitness"] = round(float(msg.fitness), 1)
        comp["status"] = MAG_STATUS_TEXT.get(status, "")
        if comp["ok"]:
            comp["pct"] = 100

        expected = [str(i) for i in c["expected"]] or [cid]
        if all(c["compasses"].get(i, {}).get("done") for i in expected):
            bad = [c["compasses"][i] for i in expected if not c["compasses"][i]["ok"]]
            if not bad:
                c["state"] = "success"
                c["message"] = ("Compass calibration finished and saved. "
                                "Reboot the flight controller to apply it.")
                self._push_message(6, "Compass calibration successful")
            else:
                c["state"] = "failed"
                why = bad[0]["status"]
                if why == "Bad orientation":
                    c["message"] = ("Failed: bad orientation. Check that the compass is "
                                    "mounted the way the COMPASS_ORIENT setting says.")
                elif why == "Bad radius":
                    c["message"] = ("Failed: bad radius. There is magnetic interference. "
                                    "Move away from metal, or move the compass away from power wires.")
                else:
                    c["message"] = "Compass calibration failed. Try again outdoors, away from metal."
                self._push_message(4, c["message"])

    def _cal_accel_request(self, position: int) -> None:
        """The drone asks for a position, or reports the end. (caller holds the lock)"""
        t = self._telem
        c = t.calibration
        if c["kind"] != "accel" or c["state"] not in CAL_ACTIVE:
            return                          # not our calibration: ignore
        if position == ACCEL_SUCCESS:
            c["state"] = "success"
            c["step"] = 6
            c["message"] = ("Accelerometer calibration successful. "
                            "Reboot the flight controller to apply it.")
            self._push_message(6, "Accelerometer calibration successful")
        elif position == ACCEL_FAILED:
            t.calibration = new_calibration(
                "accel", "failed", "Accelerometer calibration FAILED. Try again on a flat, steady surface.")
            self._push_message(4, "Accelerometer calibration failed")
        elif position in ACCEL_POSITIONS:
            if c["state"] == "running" and c["confirmed"] == position:
                return                      # the drone repeats its request while sampling
            c["state"] = "waiting_position"
            c["position"] = position
            c["position_text"] = ACCEL_POSITIONS[position]
            c["step"] = position
            c["message"] = accel_message(position)

    def _cal_from_text(self, text: str) -> None:
        """Older firmware only sends text for accelerometer calibration: read it."""
        c = self._telem.calibration
        if c["kind"] != "accel" or c["state"] not in CAL_ACTIVE:
            return
        low = text.lower()
        if "place vehicle" in low:
            pos = (4 if "nose down" in low else 5 if "nose up" in low else
                   6 if "back" in low else 2 if "left" in low else
                   3 if "right" in low else 1 if "level" in low else 0)
            if pos:
                self._cal_accel_request(pos)
        elif "calibration successful" in low:
            self._cal_accel_request(ACCEL_SUCCESS)
        elif "calibration failed" in low:
            self._cal_accel_request(ACCEL_FAILED)

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
                self.missions.abort("simulation stopped.")
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
                self.missions.abort("link closed.")
                self._cal_fail_if_active("Link closed during calibration. Start it again.")
                with self._lock:
                    self._telem.link_ok = False

            stop.wait(config.RECONNECT_DELAY)   # pause, then retry

    def _run_sim(self, stop: threading.Event) -> None:
        fake = FakeDrone()
        self._set_conn(stop, "connected", "Simulation mode: fake telemetry")
        while not stop.is_set():
            self._process_commands(None, fake)
            self.missions.process(None, fake)          # Phase 2C
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
                           (42, 2),    # MISSION_CURRENT (which waypoint is active)
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

            self._process_commands(master)     # send queued commands
            self.missions.process(master)      # Phase 2C: mission upload / download

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
                self._cal_fail_if_active("Link lost during calibration. Start it again.")
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
                self._on_ack(msg)

            # ---- Phase 2B: calibration messages ----
            elif mtype == "MAG_CAL_PROGRESS":
                self._cal_mag_progress(msg)

            elif mtype == "MAG_CAL_REPORT":
                self._cal_mag_report(msg)

            elif mtype == "COMMAND_LONG":
                # During accelerometer calibration the DRONE sends us this to say
                # "put me in position N" (or "finished" / "failed").
                if msg.command == CMD_ACCELCAL_POS:
                    self._cal_accel_request(int(round(msg.param1)))

            # ---- Phase 2C: mission messages ----
            elif mtype in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_COUNT",
                           "MISSION_ACK", "MISSION_ITEM_INT", "MISSION_ITEM"):
                self.missions.on_message(msg, master)

            elif mtype == "MISSION_CURRENT":
                self.missions.on_current(msg)

            elif mtype == "MISSION_ITEM_REACHED":
                self.missions.on_reached(msg)

            elif mtype == "STATUSTEXT":
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode(errors="ignore")
                text = str(text).strip("\x00").strip()
                if text:
                    self._push_message(msg.severity, text)
                    self._cal_from_text(text)