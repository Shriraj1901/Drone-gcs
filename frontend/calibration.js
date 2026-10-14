"use strict";
/* calibration.js - Phase 2B: compass, accelerometer and level calibration.
   It reuses $ and api() from app.js. It also chains onto window.onTelemetry
   (which commands.js already uses) so both files get every telemetry update. */
(function () {
  const ACTIVE = ["starting", "running", "waiting_position"];
  const HINTS = {
    1: "Flat on a table, top side up, nose pointing forward.",
    2: "Tip the drone onto its LEFT side (left arm down), nose still pointing forward.",
    3: "Tip the drone onto its RIGHT side (right arm down), nose still pointing forward.",
    4: "Point the nose straight DOWN (tail up).",
    5: "Point the nose straight UP (tail down).",
    6: "Turn the drone UPSIDE DOWN (on its back).",
  };

  let latest = null;     // newest telemetry message
  let busy = false;      // true while a request is in flight
  let barsKey = "";      // used to skip redrawing when nothing changed
  let dotsKey = "";

  /* ---------- confirmation dialog (same dialog box as commands.js) ---------- */
  function ask(info) {
    return new Promise((resolve) => {
      const ok = $("modalOk"), chk = $("modalCheck");
      $("modalTitle").textContent = info.title;
      $("modalText").textContent = info.text;
      $("modalCheckRow").hidden = !info.check;
      $("modalCheckText").textContent = info.check || "";
      chk.checked = false;
      ok.disabled = !!info.check;
      ok.textContent = info.okText || "Confirm";
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

  const WARN = {
    compass: {
      title: "Start compass calibration?", okText: "Start",
      text: "Go OUTDOORS, away from cars, metal, power lines, speakers and your laptop. " +
            "REMOVE THE PROPELLERS. When it starts, rotate the drone slowly so every side " +
            "points at the ground until all bars reach 100%. This takes 1 to 2 minutes.",
      check: "Propellers are removed and I am away from metal.",
    },
    accel: {
      title: "Start accelerometer calibration?", okText: "Start",
      text: "REMOVE THE PROPELLERS. Put the drone on a flat surface and do not touch it while " +
            "the gyros calibrate. Then the app asks for 6 positions: level, left side, right side, " +
            "nose down, nose up and upside down. Hold still in each one.",
      check: "Propellers are removed and I am ready.",
    },
    level: {
      title: "Start level calibration?", okText: "Calibrate",
      text: "Put the drone on a perfectly flat, level surface and keep it still. This tells the " +
            "drone what 'level' is. A wrong level makes it drift in flight.",
      check: "The drone is on a flat, level surface.",
    },
    reboot: {
      title: "Reboot the flight controller?", okText: "Reboot",
      text: "The flight controller restarts. The link drops for 10 to 20 seconds, then reconnects. " +
            "The app refuses this while the drone is armed.",
    },
  };

  const COMMAND = { compass: "cal_compass", accel: "cal_accel", level: "cal_level", reboot: "reboot" };

  function showResult(text, kind) {
    const el = $("calResult");
    el.textContent = text;
    el.className = "cmd-result " + (kind || "");
  }

  async function post(command, label) {
    if (busy) return;
    busy = true;
    refresh();
    showResult("Sending: " + label, "pending");
    try {
      const res = await api("/api/command", "POST", { command: command });
      showResult(res.message, res.ok ? "ok" : "bad");
    } catch (e) {
      showResult("Error: " + e.message, "bad");
    }
    busy = false;
    refresh();
  }

  async function start(kind) {
    if (busy) return;
    if (!(await ask(WARN[kind]))) return;
    post(COMMAND[kind], WARN[kind].title.replace("Start ", "").replace("?", ""));
  }

  /* ---------- drawing ---------- */
  function renderBars(c) {
    const ids = Object.keys(c.compasses).sort();
    const key = JSON.stringify(c.compasses);
    if (key === barsKey) return;
    barsKey = key;
    const box = $("calBars");
    box.textContent = "";
    for (const id of ids) {
      const x = c.compasses[id];
      const row = document.createElement("div");
      row.className = "cal-row";
      const label = document.createElement("div");
      label.className = "cal-label";
      const left = document.createElement("span");
      left.textContent = "Compass " + (parseInt(id, 10) + 1);
      const right = document.createElement("span");
      right.textContent = x.pct + "%  " + (x.status || "") +
        (x.fitness !== null && x.fitness !== undefined ? "  (fitness " + x.fitness + ")" : "");
      label.appendChild(left);
      label.appendChild(right);
      const track = document.createElement("div");
      track.className = "cal-track";
      const fill = document.createElement("div");
      fill.className = "cal-fill" + (x.done ? (x.ok ? " ok" : " bad") : "");
      fill.style.width = x.pct + "%";
      track.appendChild(fill);
      row.appendChild(label);
      row.appendChild(track);
      box.appendChild(row);
    }
  }

  function renderDots(c) {
    const key = c.state + ":" + c.step;
    if (key === dotsKey) return;
    dotsKey = key;
    const box = $("calDots");
    box.textContent = "";
    for (let i = 1; i <= c.steps; i++) {
      const d = document.createElement("span");
      let cls = "dot";
      if (c.state === "success" || i < c.step) cls += " done";
      else if (i === c.step && ACTIVE.includes(c.state)) cls += " now";
      d.className = cls;
      box.appendChild(d);
    }
  }

  const PILL = {
    idle: ["Idle", "grey"], starting: ["Starting", "amber"], running: ["Running", "amber"],
    waiting_position: ["Your turn", "amber"], success: ["Done", "green"], failed: ["Failed", "red"],
  };

  function refresh() {
    if (!latest) return;
    const t = latest.telemetry, conn = latest.connection;
    const c = t.calibration;
    const active = ACTIVE.includes(c.state);
    const connected = conn.state === "connected" && t.link_ok;

    // status pill
    const [text, cls] = PILL[c.state] || PILL.idle;
    $("calPill").textContent = text;
    $("calPill").className = "pill " + cls;

    // start buttons
    const canStart = connected && !t.armed && t.alt_rel < 1 && !active && !busy;
    for (const id of ["calCompassBtn", "calAccelBtn", "calLevelBtn", "calRebootBtn"]) {
      $(id).disabled = !canStart;
    }
    const needReboot = c.state === "success" && (c.kind === "compass" || c.kind === "accel");
    $("calRebootBtn").classList.toggle("attn", needReboot);

    let hint = "Remove the propellers before any calibration. Do the compass outdoors.";
    if (!connected) hint = "Connect to a drone first.";
    else if (t.armed || t.alt_rel >= 1) hint = "Disarm first. Calibration only runs on the ground while disarmed.";
    else if (active) hint = "Calibration in progress.";
    else if (needReboot) hint = "Saved. Press Reboot FC to apply the new calibration.";
    $("calNote").textContent = hint;

    // result box
    const show = c.state !== "idle" || !!c.message;
    $("calBody").hidden = !show;
    if (!show) return;
    const msg = $("calMsg");
    msg.textContent = c.message;
    msg.className = "cal-msg" + (c.state === "success" ? " ok" : c.state === "failed" ? " bad" : "");

    // compass bars
    const showBars = c.kind === "compass" && Object.keys(c.compasses).length > 0;
    $("calBars").hidden = !showBars;
    if (showBars) renderBars(c);

    // accelerometer positions
    const showPos = c.kind === "accel" && (active || c.state === "success") && c.step > 0;
    $("calPos").hidden = !showPos;
    if (showPos) {
      renderDots(c);
      const waiting = c.state === "waiting_position";
      $("calPosText").textContent = waiting
        ? "Step " + c.step + " of " + c.steps + ": " + c.position_text
        : (c.state === "success" ? "All 6 positions done" : "Step " + c.step + " of " + c.steps + ": sampling");
      $("calPosHint").textContent = waiting ? (HINTS[c.position] || "") : "";
    }

    $("calReadyBtn").hidden = !(c.state === "waiting_position");
    $("calReadyBtn").disabled = busy;
    $("calCancelBtn").hidden = !active;
    $("calCancelBtn").disabled = busy;
  }

  /* ---------- hook into the telemetry stream ---------- */
  const previous = window.onTelemetry;
  window.onTelemetry = function (data) {
    if (previous) previous(data);      // keep commands.js working
    latest = data;
    refresh();
  };

  /* ---------- buttons ---------- */
  $("calCompassBtn").addEventListener("click", () => start("compass"));
  $("calAccelBtn").addEventListener("click", () => start("accel"));
  $("calLevelBtn").addEventListener("click", () => start("level"));
  $("calRebootBtn").addEventListener("click", async () => {
    if (await ask(WARN.reboot)) post("reboot", "Reboot flight controller");
  });
  $("calReadyBtn").addEventListener("click", () => post("accel_ready", "Ready"));
  $("calCancelBtn").addEventListener("click", () => post("cal_cancel", "Cancel calibration"));
})();