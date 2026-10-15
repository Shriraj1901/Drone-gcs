"""
simulator.py - a fake drone for testing with no hardware and no SITL.

It starts DISARMED on the ground and obeys the same commands as the real drone
(arm, takeoff, modes, land, RTL) and, since Phase 2B, it also pretends to run
the compass, accelerometer and level calibrations. Since Phase 2C it also stores a
mission and flies it when you switch to AUTO.
"""
import copy
import math
import time

import config
from backend.models import (ACCEL_POSITIONS, CAL_ACTIVE, COPTER_MODES, MSG_COMPASS,
                            MSG_SAMPLING, Telemetry, accel_message, blank_compass,
                            new_calibration)

M_PER_DEG_LAT = 111_320.0
RTL_ALT_M = 15.0           # RTL climbs to this height first (like ArduPilot)


class FakeDrone:
    def __init__(self):
        self._last = time.time()
        self._born = self._last
        self.armed = False
        self.mode = "STABILIZE"
        self.north = 0.0          # metres north of home
        self.east = 0.0           # metres east of home
        self.alt = 0.0            # metres above home
        self.speed = 0.0
        self.vz = 0.0             # climb rate
        self.heading = 0.0
        self.target_alt = 0.0
        self.climbing = False     # True while a takeoff is in progress
        self.orbit_angle = math.pi
        self.battery = 100.0
        self.idle_s = 0.0
        self._outbox = [(6, "Simulation running (fake data)")]
        # ---- calibration (Phase 2B) ----
        self.cal = new_calibration()
        self._cal_t0 = 0.0        # when the fake compass calibration started
        self._acc_phase = ""      # "", "gyro", "wait" or "sample"
        self._acc_pos = 0         # which of the 6 positions we are on
        self._acc_until = 0.0     # time when the current phase ends
        # ---- mission (Phase 2C) ----
        self.mission = []         # the items the "drone" has stored (item 0 = home)
        self.seq_to_wp = []
        self.mis_seq = 1          # item being flown
        self.mis_speed = 0.0      # speed set by a "change speed" item (0 = default)
        self.mis_reached_wp = -1  # last WAYPOINT reached

    # ------------------------------------------------------------------
    # messages (like the drone's STATUSTEXT)
    # ------------------------------------------------------------------
    def say(self, severity: int, text: str) -> None:
        self._outbox.append((severity, text))

    def drain(self) -> list:
        out, self._outbox = self._outbox, []
        return out

    # ------------------------------------------------------------------
    # commands - return a MAV_RESULT code (0 = accepted, 4 = failed ...)
    # ------------------------------------------------------------------
    def handle_command(self, kind: str, mode: str = "", alt: float = 0.0,
                       value: float = 0.0) -> int:
        if kind == "arm":
            if self.armed:
                return 0
            if self.cal["state"] in CAL_ACTIVE:
                self.say(4, "PreArm: calibration in progress")
                return 4
            if self.mode in ("AUTO", "RTL", "LAND"):
                self.say(4, "PreArm: cannot arm in " + self.mode + " mode")
                return 4
            self.armed = True
            self.idle_s = 0.0
            self.say(6, "Arming motors")
            return 0

        if kind == "disarm":
            if self.armed and self.alt > 0.5:
                self.say(4, "Disarm refused: vehicle is not landed")
                return 4
            self.armed = False
            self.climbing = False
            self.say(6, "Disarming motors")
            return 0

        if kind == "takeoff":
            if not self.armed:
                self.say(4, "Takeoff failed: motors are not armed")
                return 4
            if self.mode != "GUIDED":
                self.say(4, "Takeoff failed: mode must be GUIDED")
                return 4
            self.target_alt = alt
            self.climbing = True
            self.idle_s = 0.0
            self.say(6, "Takeoff to %.0f m" % alt)
            return 0

        if kind == "mode":
            if mode not in COPTER_MODES:
                return 3                      # unsupported
            self.mode = mode
            if mode != "GUIDED":
                self.climbing = False
            if mode == "AUTO" and self.mission and self.mis_seq >= len(self.mission):
                self.mis_seq = 1                  # the last mission finished: start again
                self.mis_speed = 0.0
            return 0

        # ---------------- Phase 2B ----------------
        if kind in ("cal_compass", "cal_accel", "cal_level"):
            if self.armed or self.alt > 0.5:
                self.say(4, "Calibration refused: vehicle is armed")
                return 4
            if self.cal["state"] in CAL_ACTIVE:
                return 1                      # temporarily rejected
            return self._start_cal(kind)

        if kind in ("cal_cancel_compass", "cal_cancel_accel"):
            self.cal = new_calibration("", "idle", "Calibration cancelled.")
            self._acc_phase = ""
            self.say(6, "Calibration cancelled")
            return 0

        if kind == "accel_ready":
            c = self.cal
            if (c["kind"] != "accel" or c["state"] != "waiting_position"
                    or int(value) != self._acc_pos):
                return 4
            c["state"] = "running"
            c["message"] = MSG_SAMPLING
            c["confirmed"] = self._acc_pos
            self._acc_phase = "sample"
            self._acc_until = time.time() + config.SIM_ACCEL_SAMPLE_S
            return 0

        if kind == "reboot":
            if self.armed:
                self.say(4, "Reboot refused: vehicle is armed")
                return 4
            self.mode = "STABILIZE"
            self.cal = new_calibration()
            self.say(6, "Rebooting flight controller (simulated)")
            return 0

        return 3

    # ------------------------------------------------------------------
    # mission storage (what the real drone keeps in its memory)
    # ------------------------------------------------------------------
    def mission_upload(self, items: list, seq_to_wp: list) -> None:
        self.mission = copy.deepcopy(items)
        self.seq_to_wp = list(seq_to_wp)
        self.mis_seq, self.mis_speed, self.mis_reached_wp = 1, 0.0, -1

    def mission_download(self) -> list:
        return copy.deepcopy(self.mission)

    def mission_clear(self) -> None:
        self.mission, self.seq_to_wp = [], []
        self.mis_seq, self.mis_speed, self.mis_reached_wp = 1, 0.0, -1

    def _to_local(self, lat: float, lon: float):
        """GPS position -> metres north / east of home."""
        home_lat, home_lon = config.SIM_HOME_LAT, config.SIM_HOME_LON
        north = (lat - home_lat) * M_PER_DEG_LAT
        east = (lon - home_lon) * M_PER_DEG_LAT * math.cos(math.radians(home_lat))
        return north, east

    def _mis_next(self) -> None:
        """The current mission item is finished: go on to the next one."""
        wp = self.seq_to_wp[self.mis_seq] if self.mis_seq < len(self.seq_to_wp) else -1
        if wp >= 0:
            self.mis_reached_wp = wp
        self.say(6, "Reached command #%d" % self.mis_seq)
        self.mis_seq += 1
        if self.mis_seq >= len(self.mission):
            self.say(6, "Mission complete")

    def _run_mission(self, dt: float) -> None:
        """Fly the stored mission, one item at a time (used in AUTO mode)."""
        items = self.mission
        if self.mis_seq >= len(items):
            return                                  # finished: just hover
        it = items[self.mis_seq]
        cmd = it["command"]

        if cmd == 178:                              # change speed
            self.mis_speed = it["p2"] if it["p2"] > 0 else 0.0
            self._mis_next()

        elif cmd == 22:                             # takeoff
            if self.alt < it["z"] - 0.2:
                self.alt = min(it["z"], self.alt + config.SIM_CLIMB_MS * dt)
                self.vz = config.SIM_CLIMB_MS
            else:
                self._mis_next()

        elif cmd == 16:                             # waypoint
            tn, te = self._to_local(it["x"], it["y"])
            self._move_toward(tn, te, self.mis_speed or config.SIM_WP_SPEED_MS, dt)
            dz = it["z"] - self.alt                 # also fly to the waypoint's altitude
            if abs(dz) > 0.05:
                step = min(abs(dz), config.SIM_CLIMB_MS * dt)
                self.alt += math.copysign(step, dz)
                self.vz = math.copysign(config.SIM_CLIMB_MS, dz)
            if math.hypot(tn - self.north, te - self.east) < 2.0 and abs(dz) < 1.0:
                self._mis_next()

        elif cmd == 20:                             # return to launch
            self.say(6, "Mission: return to launch")
            self.mode = "RTL"
            self._mis_next()

        elif cmd == 21:                             # land
            self._descend(dt, 1.0)
            if not self.armed:
                self._mis_next()

        else:
            self._mis_next()

    def _start_cal(self, kind: str) -> int:
        now = time.time()
        if kind == "cal_level":
            self.cal = new_calibration(
                "level", "success",
                "Level calibration done. The drone now treats this position as level.")
            self.say(6, "Trim OK (simulated)")
            return 0

        if kind == "cal_compass":
            c = new_calibration("compass", "running", MSG_COMPASS)
            c["expected"] = [0, 1]
            for cid in ("0", "1"):
                c["compasses"][cid] = blank_compass()
            self.cal = c
            self._cal_t0 = now
            self.say(6, "Compass calibration started (simulated)")
            return 0

        # accelerometer: gyros first, then the six positions
        self.cal = new_calibration("accel", "running",
                                   "Keep the drone still: calibrating gyros...")
        self._acc_phase = "gyro"
        self._acc_until = now + 2.0
        self._acc_pos = 0
        self.say(6, "Accelerometer calibration started (simulated)")
        return 0

    def _accel_request(self, pos: int) -> None:
        """The (fake) drone asks you to place it in position 1..6."""
        c = self.cal
        c["state"] = "waiting_position"
        c["position"] = pos
        c["position_text"] = ACCEL_POSITIONS[pos]
        c["step"] = pos
        c["message"] = accel_message(pos)
        self._acc_pos = pos
        self._acc_phase = "wait"
        self.say(6, "Place vehicle %s and press any key." % ACCEL_POSITIONS[pos])

    def _cal_tick(self, now: float) -> None:
        """Move the fake calibration forward in time."""
        c = self.cal
        if c["state"] not in CAL_ACTIVE:
            return

        if c["kind"] == "compass":
            elapsed = now - self._cal_t0
            for cid, rate in zip(("0", "1"), config.SIM_COMPASS_RATES):
                comp = c["compasses"][cid]
                if comp["done"]:
                    continue
                comp["pct"] = min(100, int(elapsed * rate))
                comp["status"] = "Step 1" if comp["pct"] < 50 else "Step 2"
                if comp["pct"] >= 100:
                    comp["done"], comp["ok"] = True, True
                    comp["fitness"] = 8.4 if cid == "0" else 11.9
                    comp["status"] = "Success"
            if all(x["done"] for x in c["compasses"].values()):
                c["state"] = "success"
                c["message"] = ("Compass calibration finished and saved. "
                                "Reboot the flight controller to apply it.")
                self.say(6, "Compass calibration successful (simulated)")

        elif c["kind"] == "accel":
            if self._acc_phase == "gyro" and now >= self._acc_until:
                self._accel_request(1)
            elif self._acc_phase == "sample" and now >= self._acc_until:
                if self._acc_pos >= 6:
                    c["state"] = "success"
                    c["step"] = 6
                    c["message"] = ("Accelerometer calibration successful. "
                                    "Reboot the flight controller to apply it.")
                    self._acc_phase = ""
                    self.say(6, "Calibration successful")
                else:
                    self._accel_request(self._acc_pos + 1)

    # ------------------------------------------------------------------
    # physics
    # ------------------------------------------------------------------
    def step(self, t: Telemetry) -> None:
        """Advance the simulation and write the result into Telemetry."""
        now = time.time()
        dt = min(now - self._last, 0.5)
        self._last = now
        self._physics(dt)
        self._cal_tick(now)
        self._fill(t, now)

    def _physics(self, dt: float) -> None:
        self.speed = 0.0
        self.vz = 0.0
        if not self.armed or dt <= 0:
            return

        # battery drains faster in the air
        self.battery = max(0.0, self.battery - dt * (0.05 if self.alt < 0.5 else 0.12))

        if self.mode == "LAND":
            self._descend(dt, 1.0)
        elif self.mode == "RTL":
            self._rtl(dt)
        elif self.mode == "AUTO" and self.mission:
            self._run_mission(dt)
        elif self.mode == "AUTO" and self.alt > 2.0:
            self._orbit(dt)
        elif self.mode == "GUIDED" and self.climbing:
            self._climb(dt)
        # every other mode: just hover where we are

        # real drones disarm themselves if you arm and never take off
        if self.armed and self.alt <= 0.05 and not self.climbing:
            self.idle_s += dt
            if self.idle_s >= config.SIM_AUTO_DISARM_S:
                self.armed = False
                self.say(6, "Auto disarmed: no takeoff")
        else:
            self.idle_s = 0.0

    def _climb(self, dt: float) -> None:
        if self.alt < self.target_alt:
            self.alt = min(self.target_alt, self.alt + config.SIM_CLIMB_MS * dt)
            self.vz = config.SIM_CLIMB_MS
            if self.alt >= self.target_alt:
                self.climbing = False
                self.say(6, "Reached takeoff altitude")
        else:
            self.climbing = False

    def _descend(self, dt: float, rate: float) -> None:
        self.alt = max(0.0, self.alt - rate * dt)
        self.vz = -rate
        if self.alt <= 0.0:
            self.armed = False
            self.climbing = False
            self.say(6, "Landed - disarmed")

    def _rtl(self, dt: float) -> None:
        dist = math.hypot(self.north, self.east)
        if dist > 2.0:
            if self.alt < RTL_ALT_M:
                self.alt = min(RTL_ALT_M, self.alt + config.SIM_CLIMB_MS * dt)
                self.vz = config.SIM_CLIMB_MS
            else:
                self._move_toward(0.0, 0.0, 6.0, dt)
        else:
            self._descend(dt, 1.0)

    def _orbit(self, dt: float) -> None:
        """Demo flight for AUTO: chase a point that circles (passes through home)."""
        r = config.SIM_RADIUS_M
        self.orbit_angle += (config.SIM_SPEED_MS / r) * dt
        target_n = r + r * math.cos(self.orbit_angle)
        target_e = r * math.sin(self.orbit_angle)
        self._move_toward(target_n, target_e, config.SIM_SPEED_MS * 1.5, dt)

    def _move_toward(self, tn: float, te: float, spd: float, dt: float) -> None:
        dn, de = tn - self.north, te - self.east
        dist = math.hypot(dn, de)
        if dist < 0.01:
            return
        step = min(dist, spd * dt)
        self.north += dn / dist * step
        self.east += de / dist * step
        self.speed = step / dt
        self.heading = math.degrees(math.atan2(de, dn)) % 360.0

    # ------------------------------------------------------------------
    def _fill(self, t: Telemetry, now: float) -> None:
        elapsed = now - self._born
        home_lat, home_lon = config.SIM_HOME_LAT, config.SIM_HOME_LON
        t.has_position = True
        t.lat = home_lat + self.north / M_PER_DEG_LAT
        t.lon = home_lon + self.east / (M_PER_DEG_LAT * math.cos(math.radians(home_lat)))
        t.home_lat, t.home_lon = home_lat, home_lon

        t.heading = self.heading
        t.alt_rel = self.alt
        t.alt_msl = self.alt + 584.0
        t.groundspeed = self.speed
        t.climb = self.vz

        flying = self.armed and self.alt > 0.5
        t.roll = 3.0 * math.sin(elapsed) if flying else 0.0
        t.pitch = 2.0 * math.sin(elapsed / 2.0) if flying else 0.0

        t.battery_percent = int(self.battery)
        t.battery_voltage = 10.8 + 1.8 * self.battery / 100.0

        t.gps_fix = 3
        t.gps_fix_text = "3D Fix"
        t.satellites = 12
        t.hdop = 0.9
        t.mode = self.mode
        t.armed = self.armed
        t.link_ok = True
        t.vehicle_type = "Quadcopter (simulated)"
        t.is_copter = True
        t.calibration = copy.deepcopy(self.cal)     # Phase 2B
        # mission progress (Phase 2C). The upload/download status is kept by MissionManager.
        cur = self.mis_seq if self.mission else -1
        t.mission["current_seq"] = cur
        t.mission["current_wp"] = (self.seq_to_wp[cur]
                                   if 0 <= cur < len(self.seq_to_wp) else -1)
        t.mission["reached_wp"] = self.mis_reached_wp