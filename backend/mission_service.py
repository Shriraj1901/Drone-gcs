"""
mission_service.py - uploads, downloads and clears the mission stored on the drone.

The MAVLink "mission protocol" is a conversation, one item at a time (the radio link
is slow and loses messages, so every item is confirmed):

  UPLOAD     you: MISSION_COUNT(n)  ->  drone: MISSION_REQUEST_INT(0)  ->  you: item 0
                                    ->  drone: MISSION_REQUEST_INT(1)  ->  you: item 1 ...
                                    ->  drone: MISSION_ACK (accepted)
  DOWNLOAD   you: MISSION_REQUEST_LIST -> drone: MISSION_COUNT(n)
             you: MISSION_REQUEST_INT(0) -> drone: item 0 ... you: MISSION_ACK when done
  CLEAR      you: MISSION_CLEAR_ALL -> drone: MISSION_ACK

The web server never touches the radio. It puts a "job" in a queue; the MAVLink thread
runs the conversation. If the drone is silent, the last message is sent again (retry).
"""
import queue
import threading
import time

import config
from backend import mission as mp
from backend.models import CAL_ACTIVE

MISSION_RESULT_TEXT = {
    1: "error", 2: "unsupported frame", 3: "unsupported command",
    4: "not enough memory on the drone", 5: "invalid item", 6: "invalid parameter 1",
    7: "invalid parameter 2", 8: "invalid parameter 3", 9: "invalid parameter 4",
    10: "invalid latitude", 11: "invalid longitude", 12: "invalid altitude",
    13: "items arrived in the wrong order", 14: "denied (is the drone busy?)",
    15: "cancelled",
}


class MissionJob:
    """One upload, download or clear."""

    def __init__(self, kind: str, items=None, seq_to_wp=None):
        self.kind = kind                      # "upload" | "download" | "clear"
        self.items = items or []              # upload: the items to send
        self.seq_to_wp = seq_to_wp or []
        self.timeout = config.MISSION_TIMEOUT_S + 1.0 * len(self.items)
        self.created = time.time()
        self.last_activity = self.created     # when we last sent or received something
        self.retries = 0
        self.cancelled = False
        self.last_sent = None                 # what to repeat if the drone is silent
        self.use_int = True                   # download: ask for "_INT" items (newer)
        self.max_sent = -1                    # upload: highest item number sent so far
        self.expected = -1                    # download: how many items the drone has
        self.received = {}                    # download: seq -> item
        self.next_seq = 0
        self.done = threading.Event()
        self.ok = False
        self.message = ""
        self.result = None                    # download: the plan

    def finish(self, ok: bool, message: str, result=None) -> None:
        self.ok, self.message, self.result = ok, message, result
        self.done.set()


class MissionManager:
    def __init__(self, svc):
        self.svc = svc                        # the MavlinkService (shares its lock + data)
        self._submit_lock = threading.Lock()  # one mission operation at a time
        self._queue = queue.Queue()           # web thread -> MAVLink thread
        self._job = None                      # the job being run (MAVLink thread only)
        self.seq_to_wp = []                   # item number -> waypoint number

    # ------------------------------------------------------------------
    # Called by the web server thread
    # ------------------------------------------------------------------
    def upload(self, takeoff_alt: float, finish: str, waypoints: list) -> dict:
        error = self._check_allowed("upload")
        if error:
            return {"ok": False, "message": error}
        home = self._home()
        if home is None:
            return {"ok": False, "message": "Home position unknown. Wait for a GPS fix, then try again."}
        plan = {"takeoff_alt": takeoff_alt, "finish": finish, "waypoints": waypoints}
        error = mp.validate_plan(plan, home)
        if error:
            return {"ok": False, "message": error}
        items, seq_to_wp = mp.build_items(plan, *home)
        return self._run(MissionJob("upload", items, seq_to_wp))

    def download(self) -> dict:
        error = self._check_allowed("download")
        if error:
            return {"ok": False, "message": error}
        return self._run(MissionJob("download"))

    def clear(self) -> dict:
        error = self._check_allowed("clear")
        if error:
            return {"ok": False, "message": error}
        return self._run(MissionJob("clear"))

    def check_start(self):
        """SAFETY RULES for starting a mission. Returns an error text or None."""
        svc = self.svc
        with svc._lock:
            t = svc._telem
            m = t.mission
            armed, alt_rel = t.armed, t.alt_rel
            fix, sats, battery = t.gps_fix, t.satellites, t.battery_percent
            count, first_takeoff, busy = m["count"], m["first_takeoff"], m["state"] in (
                "uploading", "downloading", "clearing")
            cal_busy = t.calibration["state"] in CAL_ACTIVE
        if busy or cal_busy:
            return "Wait for the current upload/download/calibration to finish."
        if count < 0:
            return ("I don't know what mission is on the drone. Upload your plan, or "
                    "download the drone's mission, so I can check it first.")
        if count <= 1:
            return "The drone has no mission stored. Upload one first."
        if not armed:
            return "Arm the drone first (set GUIDED, then Arm), then start the mission."
        if fix < 3:
            return "GPS needs a 3D fix before a mission can start."
        if sats < 6:
            return "Only %d GPS satellites. Wait for at least 6." % sats
        if 0 <= battery < config.MIN_MISSION_BATTERY_PCT:
            return "Battery is at %d%%. A mission needs at least %d%%." % (
                battery, config.MIN_MISSION_BATTERY_PCT)
        if alt_rel < 1.0 and not first_takeoff:
            return ("The drone is on the ground but the mission has no takeoff. "
                    "Tick 'Takeoff' in the planner and upload again.")
        return None

    # ------------------------------------------------------------------
    def _check_allowed(self, kind: str):
        svc = self.svc
        with svc._lock:
            t = svc._telem
            state = svc._conn["state"]
            link_ok, armed, alt_rel = t.link_ok, t.armed, t.alt_rel
            is_copter = t.is_copter
            cal_busy = t.calibration["state"] in CAL_ACTIVE
        if state != "connected":
            return "Not connected to a drone."
        if not link_ok:
            return "Link lost - nothing was sent."
        if kind in ("upload", "clear"):
            if not is_copter:
                return "Mission upload is only enabled for multicopters in this version."
            if armed or alt_rel > 1.0:
                return "Disarm first. The mission can only be changed on the ground, disarmed."
            if cal_busy:
                return "A calibration is running. Finish or cancel it first."
        return None

    def _home(self):
        """(lat, lon, altitude above sea level) of home, or None if unknown."""
        svc = self.svc
        with svc._lock:
            t = svc._telem
            if not t.has_position:
                return None
            if t.home_lat is not None and t.home_lon is not None:
                lat, lon = t.home_lat, t.home_lon
            elif t.gps_fix >= 3:
                lat, lon = t.lat, t.lon
            else:
                return None
            return (lat, lon, t.alt_msl - t.alt_rel)

    def _run(self, job: MissionJob) -> dict:
        with self._submit_lock:
            self._queue.put(job)
            if not job.done.wait(job.timeout + 3.0):
                job.cancelled = True
                return {"ok": False, "message": "No answer from the drone. Operation cancelled."}
        out = {"ok": job.ok, "message": job.message}
        if job.result is not None:
            out["plan"] = job.result
        return out

    # ------------------------------------------------------------------
    # Mission status shown in the browser
    # ------------------------------------------------------------------
    def _set(self, **fields) -> None:
        with self.svc._lock:
            self.svc._telem.mission.update(fields)

    def _say(self, severity: int, text: str) -> None:
        with self.svc._lock:
            self.svc._push_message(severity, text)

    # ------------------------------------------------------------------
    # Run inside the MAVLink thread
    # ------------------------------------------------------------------
    def process(self, master, fake=None) -> None:
        """Start the next job, or keep the running one going (retries, time limit)."""
        job = self._job
        if job is None:
            try:
                job = self._queue.get_nowait()
            except queue.Empty:
                return
            if job.cancelled:
                job.finish(False, "Cancelled.")
                return
            self._job = job
            if fake is not None:
                self._run_sim(job, fake)
            else:
                self._start(job, master)
            return
        self._tick(job, master)

    def abort(self, text: str) -> None:
        """Fail everything (link closed, disconnected ...)."""
        while True:
            try:
                self._queue.get_nowait().finish(False, text)
            except queue.Empty:
                break
        if self._job is not None:
            self._fail(self._job, text)

    # ----- simulation: the fake drone stores the items directly -----
    def _run_sim(self, job: MissionJob, fake) -> None:
        if job.kind == "upload":
            fake.mission_upload(job.items, job.seq_to_wp)
            self._upload_ok(job)
        elif job.kind == "download":
            self._download_done(job, fake.mission_download())
        else:
            fake.mission_clear()
            self._clear_ok(job)

    # ----- real drone -----
    def _start(self, job: MissionJob, master) -> None:
        if job.kind == "upload":
            self._set(state="uploading", message="Sending the item count...",
                      done=0, total=len(job.items))
            self._send_count(master, len(job.items))
            job.last_sent = ("count",)
        elif job.kind == "download":
            self._set(state="downloading", message="Asking the drone for its mission...",
                      done=0, total=0)
            self._send_list(master)
            job.last_sent = ("list",)
        else:
            self._set(state="clearing", message="Clearing the mission on the drone...",
                      done=0, total=0)
            self._send_clear(master)
            job.last_sent = ("clear",)
        job.last_activity = time.time()

    def _tick(self, job: MissionJob, master) -> None:
        now = time.time()
        if job.cancelled:
            self._fail(job, "Cancelled.")
        elif now - job.created > job.timeout:
            self._fail(job, "Timed out. The drone did not complete the operation.")
        elif now - job.last_activity > config.MISSION_RETRY_S:
            if job.retries >= config.MISSION_MAX_RETRIES:
                self._fail(job, "The drone stopped answering. Check the link and try again.")
                return
            job.retries += 1
            job.last_activity = now
            if job.kind == "download":
                job.use_int = not job.use_int      # older firmware may only know the plain request
            self._resend(job, master)

    def _resend(self, job: MissionJob, master) -> None:
        what = job.last_sent
        if what is None:
            return
        if what[0] == "count":
            self._send_count(master, len(job.items))
        elif what[0] == "item":
            self._send_item(master, job.items[what[1]], what[2])
        elif what[0] == "list":
            self._send_list(master)
        elif what[0] == "request":
            self._send_request(master, job)
        elif what[0] == "clear":
            self._send_clear(master)

    def on_message(self, msg, master) -> None:
        """MISSION_* messages from the drone (called from the MAVLink thread)."""
        job = self._job
        if job is None or job.cancelled or getattr(msg, "mission_type", 0) != 0:
            return
        kind = msg.get_type()
        now = time.time()

        if job.kind == "upload":
            if kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                seq = int(msg.seq)
                if not 0 <= seq < len(job.items):
                    self._fail(job, "The drone asked for item %d, which does not exist." % seq)
                    return
                as_int = (kind == "MISSION_REQUEST_INT")
                self._send_item(master, job.items[seq], as_int)
                job.last_sent = ("item", seq, as_int)
                job.max_sent = max(job.max_sent, seq)
                job.last_activity, job.retries = now, 0
                self._set(done=seq + 1, total=len(job.items),
                          message="Sending item %d of %d..." % (seq + 1, len(job.items)))
            elif kind == "MISSION_ACK":
                result = int(msg.type)
                if result == 0:
                    if job.max_sent >= len(job.items) - 1:    # ignore an old, late ACK
                        self._upload_ok(job)
                else:
                    self._fail(job, "The drone rejected the mission: %s." %
                               MISSION_RESULT_TEXT.get(result, "code %d" % result))

        elif job.kind == "download":
            if kind == "MISSION_COUNT":
                if job.expected >= 0:
                    return                                      # a repeat: ignore
                job.expected = int(msg.count)
                job.last_activity, job.retries = now, 0
                if job.expected == 0:
                    self._download_done(job, [])
                else:
                    self._set(done=0, total=job.expected,
                              message="Downloading %d items..." % job.expected)
                    self._send_request(master, job)
            elif kind in ("MISSION_ITEM_INT", "MISSION_ITEM") and job.expected > 0:
                seq = int(msg.seq)
                if seq != job.next_seq:
                    return                                      # a repeat or out of order
                if kind == "MISSION_ITEM_INT":
                    x, y = msg.x / 1e7, msg.y / 1e7
                else:
                    x, y = float(msg.x), float(msg.y)
                job.received[seq] = {
                    "seq": seq, "frame": int(msg.frame), "command": int(msg.command),
                    "current": int(msg.current), "autocontinue": int(msg.autocontinue),
                    "p1": float(msg.param1), "p2": float(msg.param2),
                    "p3": float(msg.param3), "p4": float(msg.param4),
                    "x": x, "y": y, "z": float(msg.z)}
                job.next_seq += 1
                job.last_activity, job.retries = now, 0
                self._set(done=job.next_seq, total=job.expected,
                          message="Downloading item %d of %d..." % (job.next_seq, job.expected))
                if job.next_seq < job.expected:
                    self._send_request(master, job)
                else:
                    self._send_ack(master)
                    self._download_done(job, [job.received[i] for i in sorted(job.received)])

        else:   # clear
            if kind == "MISSION_ACK":
                result = int(msg.type)
                if result == 0:
                    self._clear_ok(job)
                else:
                    self._fail(job, "The drone refused to clear the mission: %s." %
                               MISSION_RESULT_TEXT.get(result, "code %d" % result))

    def on_current(self, msg) -> None:
        """MISSION_CURRENT: which item the drone is working on."""
        seq = int(msg.seq)
        wp = self.seq_to_wp[seq] if 0 <= seq < len(self.seq_to_wp) else -1
        self._set(current_seq=seq, current_wp=wp)

    def on_reached(self, msg) -> None:
        """MISSION_ITEM_REACHED: the drone arrived at an item."""
        seq = int(msg.seq)
        wp = self.seq_to_wp[seq] if 0 <= seq < len(self.seq_to_wp) else -1
        if wp >= 0:
            self._set(reached_wp=wp)
            self._say(6, "Reached waypoint %d" % (wp + 1))

    # ----- results -----
    def _fail(self, job: MissionJob, text: str) -> None:
        if self._job is job:
            self._job = None
        if self.svc._telem.mission["state"] in ("uploading", "downloading", "clearing"):
            self._set(state="error", message=text)
        job.finish(False, text)

    def _upload_ok(self, job: MissionJob) -> None:
        self._job = None
        self.seq_to_wp = list(job.seq_to_wp)
        n_wp = sum(1 for w in job.seq_to_wp if w >= 0)
        takeoff = len(job.items) > 1 and job.items[1]["command"] == mp.CMD_TAKEOFF
        text = "Mission uploaded: %d waypoint%s (%d items)." % (
            n_wp, "" if n_wp == 1 else "s", len(job.items))
        self._set(state="ok", message=text, count=len(job.items), first_takeoff=takeoff,
                  done=len(job.items), total=len(job.items), current_wp=-1, reached_wp=-1)
        self._say(6, text)
        job.finish(True, text)

    def _download_done(self, job: MissionJob, items: list) -> None:
        self._job = None
        plan, seq_to_wp, skipped = mp.parse_items(items)
        self.seq_to_wp = seq_to_wp
        takeoff = len(items) > 1 and items[1]["command"] == mp.CMD_TAKEOFF
        n = len(plan["waypoints"])
        if len(items) <= 1:
            text = "The drone has no mission stored."
        else:
            text = "Downloaded %d waypoint%s." % (n, "" if n == 1 else "s")
        if skipped:
            text += (" %d item%s not supported by this planner %s skipped. "
                     "Uploading from here would drop them." % (
                         skipped, "" if skipped == 1 else "s", "was" if skipped == 1 else "were"))
        self._set(state="ok", message=text, count=len(items), first_takeoff=takeoff,
                  done=len(items), total=len(items))
        self._say(6, text)
        job.finish(True, text, result=plan)

    def _clear_ok(self, job: MissionJob) -> None:
        self._job = None
        self.seq_to_wp = []
        self._set(state="ok", message="Mission cleared on the drone.", count=0,
                  first_takeoff=False, current_wp=-1, reached_wp=-1, done=0, total=0)
        self._say(6, "Mission cleared on the drone")
        job.finish(True, "Mission cleared on the drone.")

    # ----- sending (the "mission_type" field only exists in MAVLink 2, so try both) -----
    @staticmethod
    def _send_count(master, n: int) -> None:
        ts, tc = master.target_system, master.target_component
        try:
            master.mav.mission_count_send(ts, tc, n, 0)
        except TypeError:
            master.mav.mission_count_send(ts, tc, n)

    @staticmethod
    def _send_list(master) -> None:
        ts, tc = master.target_system, master.target_component
        try:
            master.mav.mission_request_list_send(ts, tc, 0)
        except TypeError:
            master.mav.mission_request_list_send(ts, tc)

    @staticmethod
    def _send_clear(master) -> None:
        ts, tc = master.target_system, master.target_component
        try:
            master.mav.mission_clear_all_send(ts, tc, 0)
        except TypeError:
            master.mav.mission_clear_all_send(ts, tc)

    @staticmethod
    def _send_ack(master) -> None:
        ts, tc = master.target_system, master.target_component
        try:
            master.mav.mission_ack_send(ts, tc, 0, 0)         # 0 = accepted
        except TypeError:
            master.mav.mission_ack_send(ts, tc, 0)

    def _send_request(self, master, job: MissionJob) -> None:
        ts, tc = master.target_system, master.target_component
        seq = job.next_seq
        try:
            if job.use_int:
                master.mav.mission_request_int_send(ts, tc, seq, 0)
            else:
                master.mav.mission_request_send(ts, tc, seq, 0)
        except TypeError:
            if job.use_int:
                master.mav.mission_request_int_send(ts, tc, seq)
            else:
                master.mav.mission_request_send(ts, tc, seq)
        job.last_sent = ("request",)

    @staticmethod
    def _send_item(master, it: dict, as_int: bool) -> None:
        ts, tc = master.target_system, master.target_component
        if as_int:
            args = (ts, tc, it["seq"], it["frame"], it["command"], it["current"],
                    it["autocontinue"], it["p1"], it["p2"], it["p3"], it["p4"],
                    int(round(it["x"] * 1e7)), int(round(it["y"] * 1e7)), it["z"])
            send = master.mav.mission_item_int_send
        else:
            args = (ts, tc, it["seq"], it["frame"], it["command"], it["current"],
                    it["autocontinue"], it["p1"], it["p2"], it["p3"], it["p4"],
                    it["x"], it["y"], it["z"])
            send = master.mav.mission_item_send
        try:
            send(*args, 0)                    # 0 = mission_type "waypoints"
        except TypeError:
            send(*args)