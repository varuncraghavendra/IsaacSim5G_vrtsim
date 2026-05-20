"use strict";

// ============================================================================
// IsaacSim5G dashboard — multi-camera, platform-aware, hold-to-drive.
// ============================================================================
const $ = (id) => document.getElementById(id);

// Top stats
const connDot      = $("connDot");
const modePill     = $("modePill");
const platformPill = $("platformPill");
const sTx          = $("sTx");
const sRx          = $("sRx");
const sPanels      = $("sPanels");
const sCmdState    = $("sCmdState");

// Camera grid (panels rendered dynamically)
const camGrid          = $("camGrid");
const camPanelTemplate = $("camPanelTemplate");

// Control / sliders
const estopBanner  = $("estopBanner");
const linRange     = $("linRange");
const angRange     = $("angRange");
const vertRange    = $("vertRange");
const rateRange    = $("rateRange");
const linVal       = $("linVal");
const angVal       = $("angVal");
const vertVal      = $("vertVal");
const rateVal      = $("rateVal");
const ctrlTitle    = $("ctrlTitle");
const kbdHint      = $("kbdHint");

const padBtns = {
  w: $("btnW"), a: $("btnA"), s: $("btnS"), d: $("btnD"),
  q: $("btnQ"), e: $("btnE"), r: $("btnR"), f: $("btnF"),
};
const btnStop = $("btnStop");
const evtBtns = {
  arm: $("btnArm"), disarm: $("btnDisarm"),
  takeoff: $("btnTakeoff"), land: $("btnLand"), offboard: $("btnOffboard"),
};

const mX = $("mX"), mY = $("mY"), mZ = $("mZ"), mYaw = $("mYaw"), mSpd = $("mSpd");
const mCmdLin = $("mCmdLin"), mCmdAng = $("mCmdAng"), mCmdVert = $("mCmdVert");
const statusBox = $("status"), btnClearLog = $("btnClearLog");

// ============================================================================
// State
// ============================================================================
const KEYS = ["w","a","s","d","q","e","r","f"];
const keyState = Object.fromEntries(KEYS.map(k => [k, false]));
let estop = false;
let currentLin = parseFloat(linRange.value);
let currentAng = parseFloat(angRange.value);
let currentVert = parseFloat(vertRange.value);
let sendHz = parseInt(rateRange.value, 10);
let platform = null;

let ws = null, wsReady = false;
let lastPoseStamp = 0;
let txStamps = [], rxStamps = [];
let zeroSentAt = 0;
let knownTopics = [];   // last topics envelope (for dropdowns)

// Each entry: {id, label, topic, root, img, nofeed, hint, sel, topicLabel,
//              sizeLabel, fpsLabel, lastStamp, frameTimes:[]}.
const panels = new Map();

// ============================================================================
function log(line) {
  const ts = new Date().toLocaleTimeString();
  statusBox.textContent = `[${ts}] ${line}\n` + statusBox.textContent;
  if (statusBox.textContent.length > 4000) {
    statusBox.textContent = statusBox.textContent.slice(0, 4000);
  }
}
btnClearLog.onclick = () => { statusBox.textContent = ""; };

linRange.oninput = () => {
  currentLin = parseFloat(linRange.value);
  linVal.textContent = currentLin.toFixed(2);
};
angRange.oninput = () => {
  currentAng = parseFloat(angRange.value);
  angVal.textContent = currentAng.toFixed(2);
};
vertRange.oninput = () => {
  currentVert = parseFloat(vertRange.value);
  vertVal.textContent = currentVert.toFixed(2);
};
rateRange.oninput = () => {
  sendHz = parseInt(rateRange.value, 10);
  rateVal.textContent = sendHz;
  restartTxLoop();
};

// ============================================================================
// Camera panels
// ============================================================================
const FRESH_THRESHOLD_S = 1.0;
function isFresh(c) {
  return c.last_frame_age_s != null && c.last_frame_age_s <= FRESH_THRESHOLD_S;
}
function freshnessTag(c) {
  if (c.last_frame_age_s == null) return "never";
  if (c.last_frame_age_s <= FRESH_THRESHOLD_S) return "LIVE";
  if (c.last_frame_age_s < 5) return `${c.last_frame_age_s.toFixed(1)}s ago`;
  return `${Math.round(c.last_frame_age_s)}s ago`;
}

function rebuildPanels(panelDefs) {
  // Tear down existing panels.
  camGrid.innerHTML = "";
  panels.clear();

  panelDefs.forEach((def, idx) => {
    const frag = camPanelTemplate.content.cloneNode(true);
    const root = frag.querySelector(".camera-panel");
    const label    = root.querySelector(".js-cam-label");
    const sel      = root.querySelector(".js-cam-select");
    const snap     = root.querySelector(".js-cam-snap");
    const img      = root.querySelector(".js-cam-img");
    const nofeed   = root.querySelector(".js-cam-nofeed");
    const hint     = root.querySelector(".js-cam-nofeed-hint");
    const topicLab = root.querySelector(".js-cam-topic");
    const sizeLab  = root.querySelector(".js-cam-size");
    const fpsLab   = root.querySelector(".js-cam-fps");

    label.textContent = def.label || `Camera ${idx + 1}`;
    const panel = {
      id: idx,
      label: def.label || `Camera ${idx + 1}`,
      topic: def.topic || "",
      root, img, nofeed, hint, sel, topicLab, sizeLab, fpsLab,
      lastStamp: 0,
      frameTimes: [],
    };
    panels.set(idx, panel);

    topicLab.textContent = "topic: " + (panel.topic || "—");
    hint.innerHTML =
      `Waiting for frames on <code>${panel.topic || "(no topic)"}</code>. ` +
      `Pick a different topic from the dropdown if needed.`;

    sel.onchange = () => {
      const newTopic = sel.value;
      if (!newTopic || newTopic === panel.topic) return;
      panel.topic = newTopic;
      panel.lastStamp = 0;
      panel.frameTimes.length = 0;
      panel.img.classList.remove("live");
      panel.nofeed.style.display = "flex";
      panel.topicLab.textContent = "topic: " + newTopic;
      panel.hint.innerHTML =
        `Waiting for frames on <code>${newTopic}</code>.`;
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({type: "ensure_camera", topic: newTopic}));
      }
      log(`panel ${idx} -> ${newTopic}`);
    };

    snap.onclick = () => {
      if (!panel.img.src || !panel.img.classList.contains("live")) {
        log(`panel ${idx}: no live frame to snapshot`);
        return;
      }
      const a = document.createElement("a");
      a.href = panel.img.src;
      const ts = new Date().toISOString().replace(/[:.]/g, "-");
      const safeTopic = panel.topic.replace(/[^a-z0-9]+/gi, "_").slice(0, 60);
      a.download = `${safeTopic || "panel"}_${ts}.jpg`;
      document.body.appendChild(a); a.click(); a.remove();
      log(`panel ${idx}: snapshot saved`);
    };

    camGrid.appendChild(frag);

    // Make sure the backend has a live subscription for this topic.
    if (panel.topic && ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({type: "ensure_camera", topic: panel.topic}));
    }
  });

  sPanels.textContent = String(panels.size);
  populatePanelDropdowns();
}

function populatePanelDropdowns() {
  // Use the cached knownTopics list to refill every panel's dropdown
  // while preserving each panel's current topic selection.
  for (const panel of panels.values()) {
    const sel = panel.sel;
    sel.innerHTML = "";

    // Sorted union of: known topics + this panel's selected topic (in case
    // it isn't yet in the topic list).
    const seen = new Set(knownTopics.map(c => c.topic));
    const list = knownTopics.slice();
    if (panel.topic && !seen.has(panel.topic)) {
      list.push({topic: panel.topic, publishers: 0,
                 frames_received: 0, last_frame_age_s: null});
    }
    list.sort((a, b) => {
      const af = isFresh(a) ? 0 : 1, bf = isFresh(b) ? 0 : 1;
      if (af !== bf) return af - bf;
      const aa = a.last_frame_age_s == null ?  1e9 : a.last_frame_age_s;
      const bb = b.last_frame_age_s == null ?  1e9 : b.last_frame_age_s;
      if (aa !== bb) return aa - bb;
      if ((b.publishers||0) !== (a.publishers||0))
        return (b.publishers||0) - (a.publishers||0);
      return a.topic.localeCompare(b.topic);
    });

    for (const c of list) {
      const opt = document.createElement("option");
      opt.value = c.topic;
      const pub = (c.publishers != null) ? `${c.publishers}p` : "?";
      const fr  = `${c.frames_received||0}fr`;
      const fresh = freshnessTag(c);
      const star = (c.topic === panel.topic) ? "★ " : "  ";
      opt.textContent = `${star}${c.topic}  ${pub}/${fr}/${fresh}`;
      if (c.topic === panel.topic) opt.selected = true;
      sel.appendChild(opt);
    }
  }
}

function deliverFrame(envelope) {
  // The envelope has `base` (canonical topic) and `topic` (may include
  // /compressed). Match each panel by canonical base.
  const base = envelope.base
            || (envelope.topic && envelope.topic.endsWith("/compressed")
                ? envelope.topic.slice(0, -"/compressed".length)
                : envelope.topic);
  for (const panel of panels.values()) {
    if (panel.topic !== base) continue;
    panel.lastStamp = envelope.stamp;
    panel.frameTimes.push(performance.now());
    panel.img.src = `data:image/jpeg;base64,${envelope.data_b64}`;
    panel.img.classList.add("live");
    panel.nofeed.style.display = "none";
    panel.topicLab.textContent = "topic: " + envelope.topic;
    if (envelope.width && envelope.height) {
      panel.sizeLab.textContent = `${envelope.width}×${envelope.height}`;
    }
  }
}

// ============================================================================
// Control intent
// ============================================================================
function computeIntent() {
  if (estop) return { linear_x: 0, linear_y: 0, linear_z: 0, angular_z: 0, emergency_stop: true };
  let lx = 0, ly = 0, lz = 0, az = 0;
  if (!platform || platform.kind === "ground") {
    // Mecanum / holonomic ground: W/S forward-back, A/D strafe,
    // Q/E rotate in place. Nova Carter is mecanum so it can strafe;
    // the dashboard sends both linear_y and angular_z and the broker
    // / bridge / Isaac propagate the full Twist.
    if (keyState.w) lx += currentLin;
    if (keyState.s) lx -= currentLin;
    if (keyState.a) ly += currentLin;
    if (keyState.d) ly -= currentLin;
    if (keyState.q) az += currentAng;
    if (keyState.e) az -= currentAng;
  } else {
    if (keyState.w) lx += currentLin;
    if (keyState.s) lx -= currentLin;
    if (keyState.a) ly += currentLin;
    if (keyState.d) ly -= currentLin;
    if (keyState.q) az += currentAng;
    if (keyState.e) az -= currentAng;
    if (keyState.r) lz += currentVert;
    if (keyState.f) lz -= currentVert;
  }
  return { linear_x: lx, linear_y: ly, linear_z: lz, angular_z: az, emergency_stop: false };
}
function anyKeyHeld() { return KEYS.some(k => keyState[k]); }

let txTimer = null;
function restartTxLoop() {
  if (txTimer) clearInterval(txTimer);
  txTimer = setInterval(txTick, Math.max(10, Math.round(1000 / sendHz)));
}
function txTick() {
  if (!wsReady) return;
  const intent = computeIntent();
  const held = anyKeyHeld() || estop;
  const now = performance.now();
  if (!held && now - zeroSentAt > 600) return;
  try {
    ws.send(JSON.stringify(Object.assign({source: "dashboard"}, intent)));
    txStamps.push(now);
    mCmdLin.textContent  = intent.linear_x.toFixed(2);
    mCmdAng.textContent  = intent.angular_z.toFixed(2);
    if (mCmdVert) mCmdVert.textContent = intent.linear_z.toFixed(2);
    zeroSentAt = held ? 0 : now;
  } catch (e) { log("send failed: " + e.message); }
}

// ============================================================================
// Key handling
// ============================================================================
function setKey(k, on) {
  if (!(k in keyState) || keyState[k] === on) return;
  keyState[k] = on;
  const btn = padBtns[k];
  if (btn) btn.classList.toggle("active", on);
}
document.addEventListener("keydown", (e) => {
  if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT")) return;
  if (e.repeat) return;
  if (e.code === "Space") {
    e.preventDefault();
    if (!estop) {
      estop = true; estopBanner.classList.add("on"); btnStop.classList.add("active");
      log("E-STOP engaged");
    }
    return;
  }
  if (platform && platform.kind === "drone") {
    const map = { "z":"arm", "x":"disarm", "t":"takeoff", "l":"land" };
    const evt = map[e.key.toLowerCase()];
    if (evt) { e.preventDefault(); sendEvent(evt); return; }
  }
  setKey(e.key.toLowerCase(), true);
});
document.addEventListener("keyup", (e) => {
  if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT")) return;
  if (e.code === "Space") {
    e.preventDefault();
    if (estop) {
      estop = false; estopBanner.classList.remove("on"); btnStop.classList.remove("active");
      log("E-STOP released");
    }
    return;
  }
  setKey(e.key.toLowerCase(), false);
});
window.addEventListener("blur", () => { for (const k of KEYS) setKey(k, false); });

function bindMomentary(btn, key) {
  if (!btn) return;
  const down = (e) => { e.preventDefault(); setKey(key, true); };
  const up   = (e) => { e.preventDefault(); setKey(key, false); };
  btn.addEventListener("mousedown", down);
  btn.addEventListener("mouseup",   up);
  btn.addEventListener("mouseleave",up);
  btn.addEventListener("touchstart",down, {passive:false});
  btn.addEventListener("touchend",  up);
  btn.addEventListener("touchcancel",up);
}
for (const k of Object.keys(padBtns)) bindMomentary(padBtns[k], k);
btnStop.addEventListener("mousedown", () => {
  estop = true; estopBanner.classList.add("on"); btnStop.classList.add("active");
});
btnStop.addEventListener("mouseup",   () => {
  estop = false; estopBanner.classList.remove("on"); btnStop.classList.remove("active");
});
btnStop.addEventListener("mouseleave",() => {
  estop = false; estopBanner.classList.remove("on"); btnStop.classList.remove("active");
});

function sendEvent(name) {
  if (!wsReady) return;
  try { ws.send(JSON.stringify({type: "event", name})); log("event: " + name); }
  catch (e) { log("event send failed: " + e.message); }
}
for (const [name, btn] of Object.entries(evtBtns)) {
  if (btn) btn.addEventListener("click", () => sendEvent(name));
}

// ============================================================================
function applyPlatform(p) {
  if (!p) return;
  platform = p;
  platformPill.textContent = "platform: " + (p.name || "—");
  if (p.kind === "drone") {
    document.body.classList.remove("platform-ground");
    document.body.classList.add("platform-drone");
    ctrlTitle.textContent = "Fly";
  } else {
    document.body.classList.add("platform-ground");
    document.body.classList.remove("platform-drone");
    ctrlTitle.textContent = "Drive";
  }
  kbdHint.textContent = p.control_help || "";
  if (p.linear_max_mps) {
    linRange.max = String(p.linear_max_mps);
    if (parseFloat(linRange.value) > p.linear_max_mps) {
      linRange.value = String(p.linear_max_mps); linRange.oninput();
    }
  }
  if (p.angular_max_rps) {
    angRange.max = String(p.angular_max_rps);
    if (parseFloat(angRange.value) > p.angular_max_rps) {
      angRange.value = String(p.angular_max_rps); angRange.oninput();
    }
  }
  if (p.vertical_max_mps) {
    vertRange.max = String(p.vertical_max_mps);
    if (parseFloat(vertRange.value) > p.vertical_max_mps) {
      vertRange.value = String(p.vertical_max_mps); vertRange.oninput();
    }
  }
}

// ============================================================================
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => {
    wsReady = true; connDot.classList.add("ok");
    log("WebSocket open");
  };
  ws.onmessage = (ev) => {
    rxStamps.push(performance.now());
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    switch (m.type) {
      case "config":
        if (m.mode) modePill.textContent = "mode: " + m.mode;
        if (m.platform) applyPlatform(m.platform);
        if (Array.isArray(m.camera_panels) && m.camera_panels.length) {
          rebuildPanels(m.camera_panels);
        }
        if (m.camera_topics) {
          log(`config: platform=${m.platform?.name} cameras=${m.camera_topics.length} panels=${(m.camera_panels||[]).length}`);
        }
        break;
      case "topics":
        knownTopics = m.cameras || [];
        populatePanelDropdowns();
        break;
      case "pose":
        lastPoseStamp = m.stamp;
        mX.textContent   = m.x.toFixed(2);
        mY.textContent   = m.y.toFixed(2);
        if (mZ) mZ.textContent = (m.z || 0).toFixed(2);
        mYaw.textContent = (m.yaw * 180 / Math.PI).toFixed(1);
        const sp = Math.sqrt((m.linear_x||0)**2 + (m.linear_y||0)**2 + (m.linear_z||0)**2);
        mSpd.textContent = sp.toFixed(2);
        break;
      case "camera":
        deliverFrame(m);
        break;
      case "cmd_ack": {
        const p = m.payload || {};
        const state = p.emergency_stop ? "e-stop" :
          (p.linear_x === 0 && p.angular_z === 0 && (p.linear_z||0) === 0 ? "stop" : "driving");
        sCmdState.textContent = state;
        break;
      }
      case "error": log("server error: " + (m.reason || "unknown")); break;
    }
  };
  ws.onclose = () => {
    wsReady = false; connDot.classList.remove("ok");
    log("WebSocket closed, reconnect in 1s");
    setTimeout(connect, 1000);
  };
  ws.onerror = () => {};
}
connect();
restartTxLoop();

// ============================================================================
setInterval(() => {
  const now = performance.now();
  txStamps = txStamps.filter(t => now - t < 2000);
  rxStamps = rxStamps.filter(t => now - t < 2000);
  sTx.textContent = (txStamps.length / 2.0).toFixed(0) + " Hz";
  sRx.textContent = (rxStamps.length / 2.0).toFixed(0) + " Hz";

  // Per-panel FPS + stale handling.
  for (const panel of panels.values()) {
    panel.frameTimes = panel.frameTimes.filter(t => now - t < 2000);
    const fps = panel.frameTimes.length / 2.0;
    panel.fpsLab.textContent = panel.frameTimes.length ? fps.toFixed(1) + " fps" : "— fps";
    if (panel.lastStamp && (Date.now() / 1000) - panel.lastStamp > 2) {
      panel.img.classList.remove("live");
      panel.nofeed.style.display = "flex";
    }
  }
}, 250);
