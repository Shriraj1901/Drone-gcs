"use strict";
/* commands.js - Phase 2A: command buttons with safety checks.
   It reuses $ and api() from app.js, and app.js calls window.onTelemetry(data)
   ten times a second with the newest telemetry. */
(function () {
  const SAFE_MODES = [
    ["STABILIZE", "Stabilize (manual)"],
    ["ALT_HOLD", "Alt Hold"],
    ["LOITER", "Loiter (GPS hold)"],
    ["POSHOLD", "Pos Hold"],
    ["GUIDED", "Guided"],
    ["AUTO", "Auto (mission)"],
  ];

  let latest = null;     // newest telemetry message
  let unlocked = false;  // controls start locked
  let busy = false;      // true while a command is in flight

  /* ---------- confirmation dialog (returns a Promise: true / false) ---------- */
  function ask(info) {
    return new Promise((resolve) => {
      const ok = $("modalOk"), chk = $("modalCheck");
      $("modalTitle").textContent = info.title;
      $("modalText").textContent = info.text;
      $("modalCheckRow").hidden = !info.check;
      $("modalCheckText").textContent = info.check || "";
      chk.checked = false;
      ok.disabled = !!info.check;           // must tick the box first
      ok.textContent = info.okText || "Confirm";
      ok.classList.toggle("danger", !!info.danger);
      $("modal").hidden = false;

      const onKey = (e) => { if (e.key === "Escape") close(false); };
      const close = (value) => {
        $("modal").hidden = true;
        chk.onchange = ok.onclick = $("modalCancel").onclick = null;
        document.removeEventListener("keydown", onKey);
        resolve(value);
      };
      document.addEventListener("keydown", onKey);
      chk.onchange = () => { ok.disabled = !chk.checked; };
      ok.onclick = () => close(true);
      $("modalCancel").onclick = () => close(false);
    });
  }

  /* ---------- the warning text for each command ---------- */
  function describe(cmd) {
    const alt = parseFloat($("takeoffAlt").value);
    const mode = $("modeSel").value;
    switch (cmd) {
      case "arm":
        return { title: "Arm motors?", okText: "Arm", danger: true,
          text: "The motors WILL spin. Keep hands and face away. On the bench, REMOVE THE PROPELLERS first.",
          check: "Propellers are removed, or the area is clear and I am ready." };
      case "disarm":
        return { title: "Disarm motors?", okText: "Disarm",
          text: "The motors will stop. The app refuses this while the drone is in the air." };
      case "takeoff":
        return { title: "Take off to " + alt + " m?", okText: "Take off", danger: true,
          text: "The drone will climb to " + alt + " m and hold. Keep your RC transmitter in your hand, ready to take over.",
          check: "The area is clear and my RC transmitter is ready." };
      case "land":
        return { title: "Land now?", okText: "Land",
          text: "The drone will descend where it is. Make sure the ground below is clear." };
      case "rtl":
        return { title: "Return to launch?", okText: "Return home",
          text: "The drone will climb, fly back to the home point and land. Check that the path home is clear." };
      default:
        if (mode === "AUTO") {
          return { title: "Switch to AUTO?", okText: "Switch to AUTO", danger: true,
            text: "If the drone is armed it will immediately start flying the stored mission.",
            check: "I have checked the mission and the area is clear." };
        }
        return { title: "Change mode to " + mode + "?", okText: "Change mode",
          text: "The flight behaviour changes immediately. Make sure you know how this mode behaves." };
    }
  }

  /* ---------- show the result under the buttons ---------- */
  function showResult(text, kind) {
    const el = $("cmdResult");
    el.textContent = text;
    el.className = "cmd-result " + (kind || "");
  }

  /* ---------- enable / disable buttons from the live telemetry ---------- */
  function refresh() {
    const t = latest ? latest.telemetry : null;
    const c = latest ? latest.connection : null;
    const connected = !!(c && t && c.state === "connected" && t.link_ok);
    const ready = unlocked && connected && t.is_copter && !busy;
    const set = (id, on) => { $(id).disabled = !on; };

    set("armBtn", ready && !t.armed);
    set("disarmBtn", ready && t.armed);
    set("landBtn", ready && t.armed);
    set("rtlBtn", ready && t.armed);
    set("takeoffBtn", ready && t.armed && t.mode === "GUIDED" && t.alt_rel < 2);
    set("modeBtn", ready);
    set("modeSel", ready);
    set("takeoffAlt", ready);

    let hint = "";
    if (!connected) hint = "Connect to a drone first.";
    else if (!t.is_copter) hint = "This vehicle is not a multicopter, so commands are disabled.";
    else if (!unlocked) hint = "Controls are LOCKED. Press the lock button to enable them.";
    else if (!t.armed) hint = "Typical sequence: set mode GUIDED, Arm, then Takeoff within about 10 seconds.";
    else if (t.mode !== "GUIDED" && t.alt_rel < 2) hint = "Armed. Takeoff only works in GUIDED mode.";
    $("cmdNote").textContent = hint;
  }

  /* ---------- run a command ---------- */
  async function run(cmd) {
    if (busy || !unlocked) return;
    const info = describe(cmd);
    if (!(await ask(info))) return;

    const body = { command: cmd };
    if (cmd === "takeoff") body.altitude = parseFloat($("takeoffAlt").value);
    if (cmd === "mode") body.mode = $("modeSel").value;

    busy = true;
    refresh();
    showResult("Sending: " + info.title, "pending");
    try {
      const res = await api("/api/command", "POST", body);
      showResult(res.message, res.ok ? "ok" : "bad");
    } catch (e) {
      showResult("Error: " + e.message, "bad");
    }
    busy = false;
    refresh();
  }

  /* ---------- lock / unlock ---------- */
  function setLocked(lock) {
    unlocked = !lock;
    $("lockBtn").textContent = unlocked ? "🔓 Unlocked" : "🔒 Locked";
    $("lockBtn").classList.toggle("open", unlocked);
    refresh();
  }

  async function toggleLock() {
    if (unlocked) { setLocked(true); return; }
    const ok = await ask({
      title: "Unlock controls?", okText: "Unlock", danger: true,
      text: "These buttons can make the motors spin and the drone move. On the bench, REMOVE THE PROPELLERS. " +
            "Test in the simulator first. Controls lock again automatically if you disconnect.",
      check: "I understand the risks.",
    });
    if (ok) setLocked(false);
  }

  /* ---------- drone message log ---------- */
  let lastKey = "";
  function renderMessages(list) {
    const last = list.length ? list[list.length - 1] : null;
    const key = list.length + (last ? last.time + last.text : "");
    if (key === lastKey) return;            // nothing new
    lastKey = key;
    const box = $("msgLog");
    box.textContent = "";
    if (!last) { box.textContent = "No messages yet"; return; }
    for (const m of list.slice().reverse()) {   // newest first
      const div = document.createElement("div");
      div.className = "msg-line " + (m.sev <= 3 ? "err" : m.sev === 4 ? "warn" : "");
      div.textContent = m.time + "  " + m.text;  // textContent = safe against odd characters
      box.appendChild(div);
    }
  }

  /* ---------- called by app.js for every telemetry update ---------- */
  window.onTelemetry = function (data) {
    latest = data;
    if (unlocked && data.connection.state !== "connected") {
      setLocked(true);
      showResult("Controls re-locked: connection lost.", "bad");
    }
    refresh();
    renderMessages(data.telemetry.messages || []);
  };

  /* ---------- start up ---------- */
  for (const [value, label] of SAFE_MODES) {
    const o = document.createElement("option");
    o.value = value;
    o.textContent = label;
    $("modeSel").appendChild(o);
  }
  $("modeSel").value = "GUIDED";
  document.querySelectorAll("button.cmd").forEach((b) =>
    b.addEventListener("click", () => run(b.dataset.cmd)));
  $("lockBtn").addEventListener("click", toggleLock);
  setLocked(true);
})();