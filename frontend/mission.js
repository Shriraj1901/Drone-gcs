"use strict";
/* mission.js - Phase 2C: plan a mission on the map.
   Click the map to add waypoints, edit them in the table, upload them to the drone,
   download the drone's mission, and start it.
   It reuses $, api() and the Leaflet "map" from app.js, and chains onto window.onTelemetry. */
(function () {
  const STORE_KEY = "droneGcsMissionPlan";
  const MAX_WP = 60;
  const BUSY_STATES = ["uploading", "downloading", "clearing"];

  // The plan you are editing in the browser. Speed 0 means "keep the current speed".
  let plan = { takeoffOn: true, takeoffAlt: 10, finish: "rtl", wps: [] };
  let adding = false;      // true while a map click adds a waypoint
  let busy = false;        // true while a request to the server is running
  let latest = null;       // newest telemetry message
  let markers = [];        // Leaflet markers, same order as plan.wps
  let markerCls = [];      // "", "current" or "reached" for each marker
  let line = null;         // the blue dashed route on the map
  let homeKey = "";

  /* ---------- confirmation dialog (same dialog box as the other panels) ---------- */
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

  /* ---------- small helpers ---------- */
  const round7 = (v) => Math.round(v * 1e7) / 1e7;
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

  function metres(a, b) {                       // distance between two {lat, lon} points
    const R = 6371000, rad = Math.PI / 180;
    const dLat = (b.lat - a.lat) * rad, dLon = (b.lon - a.lon) * rad;
    const h = Math.sin(dLat / 2) ** 2 +
              Math.cos(a.lat * rad) * Math.cos(b.lat * rad) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.sqrt(h));
  }

  function homePoint() {
    const t = latest && latest.telemetry;
    return t && t.home_lat !== null && t.home_lat !== undefined
      ? { lat: t.home_lat, lon: t.home_lon } : null;
  }

  function save() {
    try { localStorage.setItem(STORE_KEY, JSON.stringify(plan)); } catch (e) { /* storage blocked */ }
  }

  function load() {
    try {
      const p = JSON.parse(localStorage.getItem(STORE_KEY));
      if (p && Array.isArray(p.wps)) {
        plan = {
          takeoffOn: p.takeoffOn !== false,
          takeoffAlt: Number.isFinite(p.takeoffAlt) ? p.takeoffAlt : 10,
          finish: ["rtl", "land", "none"].includes(p.finish) ? p.finish : "rtl",
          wps: p.wps.filter((w) => Number.isFinite(w.lat) && Number.isFinite(w.lon))
                    .map((w) => ({ lat: w.lat, lon: w.lon,
                                   alt: Number.isFinite(w.alt) ? w.alt : 20,
                                   speed: Number.isFinite(w.speed) ? w.speed : 0 })),
        };
      }
    } catch (e) { /* nothing saved yet */ }
  }

  function showResult(text, kind) {
    const el = $("mResult");
    el.textContent = text;
    el.className = "cmd-result " + (kind || "");
  }

  /* ---------- the plan -> the page ---------- */
  function applyPlanToSettings() {
    $("mTakeoffOn").checked = plan.takeoffOn;
    $("mTakeoffAlt").value = plan.takeoffAlt;
    $("mFinish").value = plan.finish;
  }

  function wpIcon(n, cls) {
    return L.divIcon({
      className: "wp-wrap", iconSize: [28, 28], iconAnchor: [14, 14],
      html: '<div class="wp-marker ' + cls + '">' + n + "</div>",
    });
  }

  function drawLine() {
    if (!line) return;
    const pts = [];
    const home = homePoint();
    if (home && plan.wps.length) pts.push([home.lat, home.lon]);
    plan.wps.forEach((w) => pts.push([w.lat, w.lon]));
    if (home && plan.wps.length && plan.finish === "rtl") pts.push([home.lat, home.lon]);
    line.setLatLngs(pts);
  }

  function updateTooltips() {
    markers.forEach((m, i) => {
      const w = plan.wps[i];
      m.setTooltipContent("Waypoint " + (i + 1) + ": " + w.alt + " m" +
                          (w.speed > 0 ? ", " + w.speed + " m/s" : ""));
    });
  }

  function renderSummary() {
    const el = $("mSummary");
    if (!plan.wps.length) { el.textContent = "No waypoints yet."; return; }
    const home = homePoint();
    let prev = home || plan.wps[0];
    let meters = 0, seconds = 0, speed = 0;
    plan.wps.forEach((w) => {
      if (w.speed > 0) speed = w.speed;
      const d = metres(prev, w);
      meters += d;
      seconds += d / (speed || 5);               // 5 m/s is assumed when no speed is set
      prev = w;
    });
    if (home && plan.finish === "rtl") {
      const d = metres(prev, home);
      meters += d;
      seconds += d / (speed || 5);
    }
    const dist = meters >= 1000 ? (meters / 1000).toFixed(2) + " km" : Math.round(meters) + " m";
    const mins = seconds >= 90 ? Math.round(seconds / 60) + " min" : Math.round(seconds) + " s";
    el.textContent = plan.wps.length + " waypoint" + (plan.wps.length === 1 ? "" : "s") +
      " · " + dist + " · about " + mins + " (estimate)" +
      (home ? "" : " · home not known yet, distance starts at waypoint 1");
  }

  function cell(text, cls) {
    const td = document.createElement("td");
    td.textContent = text;
    if (cls) td.className = cls;
    return td;
  }

  function inputCell(value, lo, hi, step, onChange) {
    const td = document.createElement("td");
    const input = document.createElement("input");
    input.type = "number";
    input.min = lo; input.max = hi; input.step = step;
    input.value = value;
    input.addEventListener("change", () => {
      let v = parseFloat(input.value);
      if (!Number.isFinite(v)) v = value;
      v = clamp(v, lo, hi);
      input.value = v;
      onChange(v);
    });
    td.appendChild(input);
    return td;
  }

  function renderTable() {
    const body = $("mRows");
    body.textContent = "";
    plan.wps.forEach((w, i) => {
      const tr = document.createElement("tr");
      const num = cell(String(i + 1), "num");
      num.addEventListener("click", () => map.panTo([w.lat, w.lon]));   // click the number to find it
      tr.appendChild(num);
      tr.appendChild(cell(w.lat.toFixed(5) + ", " + w.lon.toFixed(5), "mono"));
      tr.appendChild(inputCell(w.alt, 2, 120, 1, (v) => { w.alt = v; save(); updateTooltips(); }));
      tr.appendChild(inputCell(w.speed, 0, 15, 0.5, (v) => { w.speed = v; save(); updateTooltips(); renderSummary(); }));
      const del = document.createElement("td");
      const btn = document.createElement("button");
      btn.textContent = "✕";
      btn.title = "Delete this waypoint";
      btn.addEventListener("click", () => { plan.wps.splice(i, 1); save(); rebuild(); });
      del.appendChild(btn);
      tr.appendChild(del);
      body.appendChild(tr);
    });
    $("mEmpty").hidden = plan.wps.length > 0;
  }

  function rebuild() {                           // redraw markers, route, table and summary
    markers.forEach((m) => map.removeLayer(m));
    markers = [];
    markerCls = [];
    plan.wps.forEach((w, i) => {
      const m = L.marker([w.lat, w.lon], { icon: wpIcon(i + 1, ""), draggable: true, zIndexOffset: 500 }).addTo(map);
      m.bindTooltip("", { direction: "top" });
      m.on("dragend", () => {
        const p = m.getLatLng();
        w.lat = round7(p.lat);
        w.lon = round7(p.lng);
        save(); drawLine(); renderTable(); renderSummary();
      });
      markers.push(m);
      markerCls.push("");
    });
    updateTooltips(); drawLine(); renderTable(); renderSummary();
  }

  function setAdding(on) {
    adding = on;
    $("mAddBtn").textContent = "➕ Add waypoints: " + (on ? "ON" : "OFF");
    $("mAddBtn").classList.toggle("add-on", on);
    $("mapPanel").classList.toggle("adding", on);
    if (on) {
      $("followChk").checked = false;            // do not let the map jump away while you click
      map.doubleClickZoom.disable();
    } else {
      map.doubleClickZoom.enable();
    }
  }

  function onMapClick(e) {
    if (!adding) return;
    if (plan.wps.length >= MAX_WP) { showResult("The limit is " + MAX_WP + " waypoints.", "bad"); return; }
    const alt = clamp(parseFloat($("mDefAlt").value) || 20, 2, 120);
    const speed = clamp(parseFloat($("mDefSpeed").value) || 0, 0, 15);
    plan.wps.push({ lat: round7(e.latlng.lat), lon: round7(e.latlng.lng), alt: alt, speed: speed });
    save();
    rebuild();
  }

  /* ---------- talking to the server ---------- */
  async function call(path, body, label) {
    busy = true;
    refresh();
    showResult(label, "pending");
    let res = null;
    try {
      res = await api(path, "POST", body);
      showResult(res.message, res.ok ? "ok" : "bad");
    } catch (e) {
      showResult("Error: " + e.message, "bad");
    }
    busy = false;
    refresh();
    return res;
  }

  async function onUpload() {
    if (busy) return;
    if (!plan.wps.length) { showResult("Add at least one waypoint first.", "bad"); return; }
    const ok = await ask({
      title: "Upload the mission to the drone?", okText: "Upload",
      text: "This REPLACES any mission stored on the drone with your " + plan.wps.length +
            " waypoint" + (plan.wps.length === 1 ? "" : "s") + ". Check the altitudes and that the " +
            "route is clear of trees, buildings and people. Uploading does not start the flight.",
      check: "I have checked the waypoints, altitudes and the area under the route.",
    });
    if (!ok) return;
    await call("/api/mission/upload", {
      takeoff_alt: plan.takeoffOn ? plan.takeoffAlt : 0,
      finish: plan.finish,
      waypoints: plan.wps.map((w) => ({ lat: w.lat, lon: w.lon, alt: w.alt, speed: w.speed })),
    }, "Uploading...");
  }

  function adopt(p) {                            // replace the plan with one from the drone
    plan.takeoffOn = p.takeoff_alt > 0;
    if (p.takeoff_alt > 0) plan.takeoffAlt = p.takeoff_alt;
    plan.finish = p.finish;
    plan.wps = p.waypoints.map((w) => ({ lat: w.lat, lon: w.lon, alt: w.alt, speed: w.speed }));
    save();
    applyPlanToSettings();
    rebuild();
    if (plan.wps.length) map.fitBounds(plan.wps.map((w) => [w.lat, w.lon]), { padding: [60, 60] });
  }

  async function onDownload() {
    if (busy) return;
    if (plan.wps.length) {
      const ok = await ask({
        title: "Replace your plan?", okText: "Download",
        text: "Downloading replaces the waypoints on the map with the mission stored on the drone.",
      });
      if (!ok) return;
    }
    const res = await call("/api/mission/download", {}, "Downloading...");
    if (res && res.ok && res.plan) adopt(res.plan);
  }

  async function onClearDrone() {
    if (busy) return;
    const ok = await ask({
      title: "Clear the mission on the drone?", okText: "Clear it",
      text: "The mission stored on the drone is deleted. Your plan on the map stays.",
    });
    if (ok) await call("/api/mission/clear", {}, "Clearing...");
  }

  async function onClearPlan() {
    if (!plan.wps.length) return;
    const ok = await ask({
      title: "Clear your plan?", okText: "Clear plan",
      text: "All " + plan.wps.length + " waypoints are removed from the map. The drone is not changed.",
    });
    if (!ok) return;
    plan.wps = [];
    save();
    rebuild();
  }

  async function onStart() {
    if (busy) return;
    const ok = await ask({
      title: "Start the mission now?", okText: "Start mission", danger: true,
      text: "The drone switches to AUTO and flies the mission by itself, including the takeoff. " +
            "Keep your RC transmitter in your hand: switching it to LOITER or RTL takes over. " +
            "On the bench, REMOVE THE PROPELLERS.",
      check: "The area is clear and my RC transmitter is ready.",
    });
    if (ok) await call("/api/command", { command: "mission_start" }, "Starting the mission...");
  }

  /* ---------- status from the drone ---------- */
  const PILL = { ok: ["OK", "green"], error: ["Error", "red"] };

  function refresh() {
    if (!latest) return;
    const t = latest.telemetry, conn = latest.connection, m = t.mission;
    const connected = conn.state === "connected" && t.link_ok;
    const working = busy || BUSY_STATES.includes(m.state);

    let pill = ["Ready", "grey"];
    if (BUSY_STATES.includes(m.state)) {
      pill = [m.state === "uploading" ? "Uploading " + m.done + "/" + m.total
            : m.state === "downloading" ? "Downloading " + m.done + "/" + m.total : "Clearing", "amber"];
    } else if (PILL[m.state]) pill = PILL[m.state];
    $("mPill").textContent = pill[0];
    $("mPill").className = "pill " + pill[1];

    $("mUploadBtn").disabled = !connected || working || t.armed || !plan.wps.length;
    $("mClearDroneBtn").disabled = !connected || working || t.armed;
    $("mDownloadBtn").disabled = !connected || working;
    $("mStartBtn").disabled = !connected || working;
    $("mClearPlanBtn").disabled = !plan.wps.length;

    const flying = t.armed && t.mode === "AUTO";
    let text = "";
    if (!connected) text = "Connect to a drone to upload or fly.";
    else if (flying && m.current_wp >= 0) text = "Flying to waypoint " + (m.current_wp + 1) + " of " + plan.wps.length;
    else if (flying) text = "Mission running (item " + m.current_seq + ")";
    else if (m.count >= 0) text = "On the drone: " + Math.max(0, m.count - 1) + " mission items";
    else text = "Drone mission not checked yet: upload or download first.";
    $("mProgress").textContent = text;

    markers.forEach((mk, i) => {                 // colour the waypoints: green = done, amber = now
      let cls = "";
      if (flying && m.current_wp >= 0) cls = i < m.current_wp ? "reached" : i === m.current_wp ? "current" : "";
      if (cls !== markerCls[i]) { markerCls[i] = cls; mk.setIcon(wpIcon(i + 1, cls)); }
    });

    const h = homePoint();                       // the route starts at home: redraw when it appears
    const key = h ? h.lat + "," + h.lon : "";
    if (key !== homeKey) { homeKey = key; drawLine(); renderSummary(); }
  }

  const previous = window.onTelemetry;
  window.onTelemetry = function (data) {
    if (previous) previous(data);                // keep the other panels working
    latest = data;
    refresh();
  };

  /* ---------- start up (wait until app.js has created the map) ---------- */
  function init() {
    line = L.polyline([], { color: "#3b82f6", weight: 3, dashArray: "6 6" }).addTo(map);
    map.on("click", onMapClick);
    load();
    applyPlanToSettings();
    rebuild();
    if (plan.wps.length) map.fitBounds(plan.wps.map((w) => [w.lat, w.lon]), { padding: [60, 60] });

    $("mAddBtn").addEventListener("click", () => setAdding(!adding));
    $("mToggle").addEventListener("click", () => {
      $("mBody").hidden = !$("mBody").hidden;
      $("mToggle").textContent = $("mBody").hidden ? "+" : "–";
    });
    $("mTakeoffOn").addEventListener("change", () => { plan.takeoffOn = $("mTakeoffOn").checked; save(); });
    $("mTakeoffAlt").addEventListener("change", () => {
      plan.takeoffAlt = clamp(parseFloat($("mTakeoffAlt").value) || 10, 1, 120);
      $("mTakeoffAlt").value = plan.takeoffAlt; save();
    });
    $("mFinish").addEventListener("change", () => { plan.finish = $("mFinish").value; save(); drawLine(); renderSummary(); });
    $("mUploadBtn").addEventListener("click", onUpload);
    $("mDownloadBtn").addEventListener("click", onDownload);
    $("mClearDroneBtn").addEventListener("click", onClearDrone);
    $("mClearPlanBtn").addEventListener("click", onClearPlan);
    $("mStartBtn").addEventListener("click", onStart);
    document.addEventListener("keydown", (e) => { if (e.key === "Escape" && adding) setAdding(false); });
    refresh();
  }

  const wait = setInterval(() => {
    if (typeof map !== "undefined" && map) { clearInterval(wait); init(); }
  }, 200);
})();