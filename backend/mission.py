"""
mission.py - turns your waypoint plan into MAVLink mission items, and back.

These are plain functions (no threads, no radio), so they are easy to read and test.

How an ArduPilot mission is laid out (the item number is called "seq"):

    seq 0       HOME           the home position (ArduPilot always keeps it at 0)
    seq 1       TAKEOFF        climb to the takeoff altitude            (optional)
    seq 2...    WAYPOINTS      a "change speed" item is put in front of a waypoint
                               whenever that waypoint asks for a new speed
    last        RTL or LAND    what to do at the end                    (optional)

A "plan" is a plain dict:
    {"takeoff_alt": 10.0,            # 0 = no takeoff item
     "finish": "rtl",                # "rtl" | "land" | "none"
     "waypoints": [{"lat": .., "lon": .., "alt": 20.0, "speed": 0.0}, ...]}
    speed 0 means "keep the current speed".
"""
import math

import config

# MAVLink command numbers
CMD_WAYPOINT = 16
CMD_RTL = 20
CMD_LAND = 21
CMD_TAKEOFF = 22
CMD_CHANGE_SPEED = 178

FRAME_GLOBAL = 0          # altitude above sea level (used for the home item only)
FRAME_REL_ALT = 3         # altitude above the home point (used for everything else)
RELATIVE_FRAMES = (3, 6)  # the frames we know how to read back from the drone


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance between two GPS points in metres (haversine formula)."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def validate_plan(plan: dict, home):
    """Return an error text, or None when the plan is acceptable.
    home is (lat, lon, alt) or None."""
    if plan.get("finish") not in ("rtl", "land", "none"):
        return "Finish must be 'rtl', 'land' or 'none'."
    takeoff = plan.get("takeoff_alt", 0.0)
    if not _num(takeoff):
        return "The takeoff altitude is not a number."
    if takeoff != 0 and not (config.MIN_TAKEOFF_ALT_M <= takeoff <= config.MAX_WP_ALT_M):
        return "The takeoff altitude must be 0 (none) or %.0f to %.0f m." % (
            config.MIN_TAKEOFF_ALT_M, config.MAX_WP_ALT_M)

    wps = plan.get("waypoints") or []
    if not wps:
        return "Add at least one waypoint."
    if len(wps) > config.MAX_WAYPOINTS:
        return "Too many waypoints (%d). The limit is %d." % (len(wps), config.MAX_WAYPOINTS)

    for i, w in enumerate(wps, start=1):
        for key in ("lat", "lon", "alt", "speed"):
            if not _num(w.get(key)):
                return "Waypoint %d: %s is not a number." % (i, key)
        lat, lon, alt, speed = w["lat"], w["lon"], w["alt"], w["speed"]
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return "Waypoint %d: the position is not on Earth." % i
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return "Waypoint %d is at 0,0, which is not a real position." % i
        if not (config.MIN_WP_ALT_M <= alt <= config.MAX_WP_ALT_M):
            return "Waypoint %d: altitude must be %.0f to %.0f m." % (
                i, config.MIN_WP_ALT_M, config.MAX_WP_ALT_M)
        if speed != 0 and not (config.MIN_WP_SPEED_MS <= speed <= config.MAX_WP_SPEED_MS):
            return "Waypoint %d: speed must be 0 (keep) or %.1f to %.0f m/s." % (
                i, config.MIN_WP_SPEED_MS, config.MAX_WP_SPEED_MS)
        if home is not None:
            d = distance_m(home[0], home[1], lat, lon)
            if d > config.MAX_WP_DISTANCE_M:
                return "Waypoint %d is %.0f m from home. The limit is %.0f m." % (
                    i, d, config.MAX_WP_DISTANCE_M)
    return None


def _item(seq, frame, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, x=0.0, y=0.0, z=0.0) -> dict:
    """One mission item. x = latitude and y = longitude in degrees, z = altitude in metres."""
    return {"seq": seq, "frame": frame, "command": command, "current": 0, "autocontinue": 1,
            "p1": float(p1), "p2": float(p2), "p3": float(p3), "p4": float(p4),
            "x": float(x), "y": float(y), "z": float(z)}


def build_items(plan: dict, home_lat: float, home_lon: float, home_alt: float):
    """Plan -> (list of MAVLink items, seq_to_wp).
    seq_to_wp[seq] is the waypoint number (0 = first waypoint) for that item, or -1."""
    items, seq_to_wp = [], []

    def add(wp_index, *args, **kwargs):
        items.append(_item(len(items), *args, **kwargs))
        seq_to_wp.append(wp_index)

    # seq 0: home (ArduPilot expects it, and ignores its contents)
    add(-1, FRAME_GLOBAL, CMD_WAYPOINT, x=home_lat, y=home_lon, z=home_alt)

    if plan.get("takeoff_alt", 0) > 0:
        add(-1, FRAME_REL_ALT, CMD_TAKEOFF, z=plan["takeoff_alt"])

    last_speed = 0.0
    for i, w in enumerate(plan["waypoints"]):
        if w["speed"] > 0 and w["speed"] != last_speed:
            # p1 = 1 (ground speed), p2 = speed in m/s, p3 = -1 (leave throttle alone)
            add(-1, FRAME_REL_ALT, CMD_CHANGE_SPEED, p1=1, p2=w["speed"], p3=-1)
            last_speed = w["speed"]
        add(i, FRAME_REL_ALT, CMD_WAYPOINT, x=w["lat"], y=w["lon"], z=w["alt"])

    if plan.get("finish") == "rtl":
        add(-1, FRAME_REL_ALT, CMD_RTL)
    elif plan.get("finish") == "land":
        add(-1, FRAME_REL_ALT, CMD_LAND)        # position 0,0 = land where the drone is
    return items, seq_to_wp


def parse_items(raw: list):
    """Items read from the drone -> (plan, seq_to_wp, skipped_count).
    Items we do not understand are skipped and counted, never guessed."""
    items = sorted(raw, key=lambda i: i["seq"])
    last_seq = items[-1]["seq"] if items else 0
    plan = {"takeoff_alt": 0.0, "finish": "none", "waypoints": []}
    seq_to_wp, skipped, pending_speed = [], 0, 0.0

    for it in items:
        cmd, wp_index = it["command"], -1
        known_frame = it["frame"] in RELATIVE_FRAMES

        if it["seq"] == 0:                      # home: not part of the plan
            pass
        elif (cmd == CMD_TAKEOFF and known_frame and plan["takeoff_alt"] == 0
              and not plan["waypoints"]):
            plan["takeoff_alt"] = float(it["z"])
        elif cmd == CMD_WAYPOINT and known_frame:
            plan["waypoints"].append({"lat": it["x"], "lon": it["y"],
                                      "alt": float(it["z"]), "speed": pending_speed})
            pending_speed = 0.0
            wp_index = len(plan["waypoints"]) - 1
        elif cmd == CMD_CHANGE_SPEED:
            pending_speed = float(it["p2"]) if it["p2"] > 0 else 0.0
        elif cmd == CMD_RTL and it["seq"] == last_seq:
            plan["finish"] = "rtl"
        elif cmd == CMD_LAND and it["seq"] == last_seq:
            if abs(it["x"]) > 1e-6 or abs(it["y"]) > 1e-6:
                # a landing at its own position = fly there, then land
                prev_alt = plan["waypoints"][-1]["alt"] if plan["waypoints"] else 10.0
                plan["waypoints"].append({"lat": it["x"], "lon": it["y"],
                                          "alt": prev_alt, "speed": 0.0})
                wp_index = len(plan["waypoints"]) - 1
            plan["finish"] = "land"
        else:
            skipped += 1
        seq_to_wp.append(wp_index)
    return plan, seq_to_wp, skipped