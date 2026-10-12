"""
simulator.py - a fake drone for testing with no hardware and no SITL.

It is now a tiny "flight simulator": it starts DISARMED on the ground and
obeys the same commands as the real drone (arm, takeoff, modes, land, RTL).
So you can test the Phase 2 buttons completely safely.
"""
import math
import time

import config
from backend.models import COPTER_MODES, Telemetry

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
    def handle_command(self, kind: str, mode: str = "", alt: float = 0.0) -> int:
        if kind == "arm":
            if self.armed:
                return 0
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
            return 0

        return 3

    # ------------------------------------------------------------------
    # physics
    # ------------------------------------------------------------------
    def step(self, t: Telemetry) -> None:
        """Advance the simulation and write the result into Telemetry."""
        now = time.time()
        dt = min(now - self._last, 0.5)
        self._last = now
        self._physics(dt)
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