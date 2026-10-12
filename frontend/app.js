"use strict";
/* app.js - connects the page to the backend.
   - WebSocket gets live telemetry and updates the HUD and the map
   - Buttons call the REST API (/api/connect, /api/video/start, ...) */

const $ = (id) => document.getElementById(id);

let map, trail = null, droneMarker = null, homeMarker = null;
let ws = null;
let cfg = {};
let lastConnState = "disconnected";
let videoRunning = false;
const MAX_TRAIL_POINTS = 5000;

/* ---------------- small helpers ---------------- */
async function api(path, method = "GET", body = null) {
  const opts = { method, headers: {} };
  if (body) {
    opts.headers["Content-Type"] = "application/json";
    opts.body = JSON.stringify(body);
  }
  const res = await fetch(path, opts);
  let data = null;
  try { data = await res.json(); } catch (e) { /* no body */ }
  if (!res.ok) throw new Error((data && data.detail) || `HTTP ${res.status}`);
  return data;
}

function fmt(v, digits = 1) {
  return (v === null || v === undefined || Number.isNaN(v)) ? "--" : Number(v).toFixed(digits);
}

/* ---------------- map ---------------- */
function initMap() {
  map = L.map("map").setView(cfg.map_center || [0, 0], 15);
  L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);
  trail = L.polyline([], { color: "#ff9800", weight: 3 }).addTo(map);
  // If you drag the map yourself, stop following the drone.
  map.on("dragstart", () => { $("followChk").checked = false; });
}

function droneIcon() {
  return L.divIcon({
    className: "drone-wrap", iconSize: [40, 40], iconAnchor: [20, 20],
    html: '<div class="drone-icon"><svg viewBox="0 0 40 40" width="40" height="40">' +
          '<polygon points="20,3 33,35 20,28 7,35" fill="#ff9800" stroke="#fff" stroke-width="2"/>' +
          '</svg></div>',
  });
}

function resetMap() {
  trail.setLatLngs([]);
  if (droneMarker) { map.removeLayer(droneMarker); droneMarker = null; }
  if (homeMarker) { map.removeLayer(homeMarker); homeMarker = null; }
}

function updateMap(t) {
  if (!t.has_position) return;
  const ll = [t.lat, t.lon];

  if (!droneMarker) {                       // first valid position
    droneMarker = L.marker(ll, { icon: droneIcon(), zIndexOffset: 1000 }).addTo(map);
    map.setView(ll, 17);
  }
  droneMarker.setLatLng(ll);
  const el = droneMarker.getElement();
  if (el) {
    const inner = el.querySelector(".drone-icon");
    if (inner) inner.style.transform = `rotate(${t.heading}deg)`;
  }

  // Flight path trail (only add a point when the drone actually moved)
  const pts = trail.getLatLngs();
  const last = pts.length ? pts[pts.length - 1] : null;
  if (!last || last.lat !== t.lat || last.lng !== t.lon) {
    pts.push(L.latLng(ll));
    if (pts.length > MAX_TRAIL_POINTS) pts.shift();
    trail.setLatLngs(pts);
  }

  // Home marker
  if (t.home_lat !== null && t.home_lon !== null) {
    const h = [t.home_lat, t.home_lon];
    if (!homeMarker) {
      homeMarker = L.marker(h, {
        icon: L.divIcon({ className: "drone-wrap", iconSize: [24, 24], iconAnchor: [12, 12],
                          html: '<div class="home-icon">H</div>' }),
      }).addTo(map).bindTooltip("Home");
    } else {
      homeMarker.setLatLng(h);
    }
  }

  if ($("followChk").checked) map.panTo(ll, { animate: false });
}

/* ---------------- HUD ---------------- */
function updateHud(t, linkOk) {
  $("hudPanel").classList.toggle("stale", !linkOk);

  $("hudAlt").textContent = fmt(t.alt_rel, 1);
  $("hudAltMsl").textContent = fmt(t.alt_msl, 1);
  $("hudSpeed").textContent = fmt(t.groundspeed, 1);
  $("hudSpeedKmh").textContent = fmt(t.groundspeed * 3.6, 0);

  const dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
  $("hudHeading").textContent = Math.round(t.heading) + "°";
  $("hudCompass").textContent = dirs[Math.round(t.heading / 45) % 8];

  $("hudBatt").textContent = t.battery_voltage > 0 ? fmt(t.battery_voltage, 2) + " V" : "--";
  const pct = t.battery_percent;
  $("hudBattPct").textContent = pct >= 0 ? pct + " %" : "-- %";
  const bar = $("hudBattBar");
  bar.style.width = (pct >= 0 ? pct : 0) + "%";
  bar.style.background = pct > 50 ? "var(--green)" : pct > 25 ? "var(--amber)" : "var(--red)";

  $("hudMode").textContent = t.mode;
  const armed = $("hudArmed");
  armed.textContent = t.armed ? "ARMED" : "DISARMED";
  armed.classList.toggle("armed", t.armed);

  $("hudGps").textContent = t.gps_fix_text;
  $("hudSats").textContent = t.satellites;
  $("hudHdop").textContent = t.hdop > 0 ? fmt(t.hdop, 1) : "--";

  $("hudLat").textContent = t.has_position ? fmt(t.lat, 6) : "--";
  $("hudLon").textContent = t.has_position ? fmt(t.lon, 6) : "--";
  $("hudStatusText").textContent = t.status_text || "--";
}

/* ---------------- connection status ---------------- */
function updateConnection(c, t) {
  let text = "DISCONNECTED", cls = "grey";
  if (c.state === "connecting") { text = "CONNECTING"; cls = "amber"; }
  else if (c.state === "error") { text = "ERROR"; cls = "red"; }
  else if (c.state === "connected") {
    if (t.link_ok) { text = "LINK OK"; cls = "green"; }
    else { text = "LINK LOST"; cls = "red"; }
  }
  const pill = $("linkPill");
  pill.textContent = text;
  pill.className = "pill " + cls;
  $("connMsg").textContent = c.message;

  const btn = $("connectBtn");
  const active = c.state !== "disconnected";
  btn.textContent = active ? "Disconnect" : "Connect";
  btn.classList.toggle("danger", active);
  lastConnState = c.state;
}

function handleMessage(data) {
  updateConnection(data.connection, data.telemetry);
  updateHud(data.telemetry, data.telemetry.link_ok);
  updateMap(data.telemetry);
  if (window.onTelemetry) window.onTelemetry(data);   
}

/* ---------------- WebSocket (auto-reconnects) ---------------- */
function connectWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws/telemetry`);
  ws.onmessage = (e) => handleMessage(JSON.parse(e.data));
  ws.onclose = () => {
    $("linkPill").textContent = "SERVER OFFLINE";
    $("linkPill").className = "pill red";
    setTimeout(connectWS, 1000);
  };
  ws.onerror = () => ws.close();
}

/* ---------------- connection controls ---------------- */
function updateConnFields() {
  const type = $("connType").value;
  $("serialFields").style.display = type === "serial" ? "" : "none";
  $("udpAddr").style.display = type === "udp" ? "" : "none";
  $("tcpAddr").style.display = type === "tcp" ? "" : "none";
}

async function loadPorts() {
  const sel = $("serialPort");
  sel.innerHTML = "";
  try {
    const ports = await api("/api/ports");
    if (ports.length === 0) {
      sel.innerHTML = "<option value=''>No ports found</option>";
      return;
    }
    for (const p of ports) {
      const o = document.createElement("option");
      o.value = p.device;
      o.textContent = `${p.device} - ${p.description}`;
      sel.appendChild(o);
    }
    if (ports.some((p) => p.device === cfg.serial_port)) sel.value = cfg.serial_port;
  } catch (e) {
    sel.innerHTML = "<option value=''>Error listing ports</option>";
  }
}

async function onConnectClick() {
  try {
    if (lastConnState !== "disconnected") {
      await api("/api/disconnect", "POST");
      return;
    }
    resetMap();
    await api("/api/connect", "POST", {
      conn_type: $("connType").value,
      serial_port: $("serialPort").value,
      baud: parseInt($("baud").value, 10),
      udp: $("udpAddr").value.trim(),
      tcp: $("tcpAddr").value.trim(),
    });
  } catch (e) {
    $("connMsg").textContent = "Error: " + e.message;
  }
}

/* ---------------- video controls ---------------- */
function updateVideoInput() {
  const type = $("videoType").value;
  const input = $("videoValue");
  const hints = {
    test:   { ph: "(no input needed)", val: "", off: true },
    webcam: { ph: "Camera index, e.g. 0", val: "0", off: false },
    rtsp:   { ph: "rtsp://192.168.1.10:554/stream", val: cfg.rtsp_url || "", off: false },
    udp:    { ph: "UDP port, e.g. 5600", val: String(cfg.udp_video_port || 5600), off: false },
  }[type];
  input.placeholder = hints.ph;
  input.value = hints.val;
  input.disabled = hints.off;
}

function updateVideoUI(s) {
  videoRunning = s.running;
  const pill = $("videoStatus");
  const cls = { streaming: "green", starting: "amber", error: "red" }[s.state] || "grey";
  pill.className = "pill " + cls;
  pill.textContent = s.state === "streaming" ? `Streaming ${s.fps || ""} fps` : s.message;

  $("videoBtn").textContent = s.running ? "Stop video" : "Start video";
  $("videoBtn").classList.toggle("danger", s.running);
  $("videoPlaceholder").textContent = s.running ? s.message : "No video";

  const img = $("videoImg");
  if (s.running) {
    if (!img.getAttribute("src")) {
      img.src = "/video_feed?t=" + Date.now();
      img.hidden = false;
    }
  } else {
    img.removeAttribute("src");
    img.hidden = true;
  }
}

async function refreshVideoStatus() {
  try { updateVideoUI(await api("/api/video/status")); } catch (e) { /* server busy */ }
}

async function onVideoBtn() {
  try {
    if (videoRunning) {
      await api("/api/video/stop", "POST");
    } else {
      $("videoImg").removeAttribute("src");   // force a fresh stream connection
      await api("/api/video/start", "POST", {
        source_type: $("videoType").value,
        value: $("videoValue").value.trim(),
      });
    }
    await refreshVideoStatus();
  } catch (e) {
    $("videoStatus").textContent = e.message;
    $("videoStatus").className = "pill red";
  }
}

/* ---------------- start everything ---------------- */
async function init() {
  try { cfg = await api("/api/config"); } catch (e) { cfg = {}; }

  initMap();

  $("connType").value = cfg.conn_type || "sim";
  $("baud").value = String(cfg.baud || 57600);
  $("udpAddr").value = cfg.udp || "";
  $("tcpAddr").value = cfg.tcp || "";
  $("videoType").value = cfg.video_type || "test";
  updateConnFields();
  updateVideoInput();
  await loadPorts();

  $("connType").addEventListener("change", updateConnFields);
  $("refreshPorts").addEventListener("click", loadPorts);
  $("connectBtn").addEventListener("click", onConnectClick);
  $("videoType").addEventListener("change", updateVideoInput);
  $("videoBtn").addEventListener("click", onVideoBtn);
  $("clearTrailBtn").addEventListener("click", () => trail.setLatLngs([]));
  $("centerBtn").addEventListener("click", () => {
    $("followChk").checked = true;
    if (droneMarker) map.setView(droneMarker.getLatLng(), Math.max(map.getZoom(), 16));
  });

  connectWS();
  refreshVideoStatus();
  setInterval(refreshVideoStatus, 1500);
}

init();