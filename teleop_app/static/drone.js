"use strict";
// =============================================================================
// IsaacSim5G drone teleop — twin virtual joystick + FPV.
// =============================================================================
const $ = (id) => document.getElementById(id);

const connDot   = $("connDot");
const modePill  = $("modePill");
const armedPill = $("armedPill");
const sTx       = $("sTx");
const sRx       = $("sRx");
const sFps      = $("sFps");
const sMav      = $("sMav");

const camImg    = $("camImg");
const camTopic  = $("camTopic");
const camSize   = $("camSize");
const camFps    = $("camFps");
const nofeed    = $("nofeed");

const estopBanner = $("estopBanner");
const statusBox   = $("status");
const btnClearLog = $("btnClearLog");

const mX = $("mX"), mY = $("mY"), mZ = $("mZ");
const mYaw = $("mYaw"), mSpd = $("mSpd"), mRtf = $("mRtf");
const mThr = $("mThr"), mYawCmd = $("mYawCmd");

const leftStick  = $("leftStick");
const rightStick = $("rightStick");
const leftVal    = $("leftVal");
const rightVal   = $("rightVal");

// ============================================================================
// State
// ============================================================================
let ws = null, wsReady = false;
let estop = false;
let txStamps = [], rxStamps = [], frameStamps = [];

// Stick state — each in [-1, 1] except throttle which we map separately.
const stick = {
  // Left stick: vertical = throttle (down=full down, up=full up),
  //             horizontal = yaw
  left:  { x: 0, y: 0 },
  // Right stick: vertical = pitch (up=forward),
  //              horizontal = roll
  right: { x: 0, y: 0 },
};

// Throttle has memory: we want the drone to stay at altitude when the
// stick is centered, so center == 0.5 throttle (PX4 altitude mode hover).
// When estop is engaged we slam to 0.5 (no-op) and zero everything else.
let lastThrottle = 0.5;

function log(line) {
  const ts = new Date().toLocaleTimeString();
  statusBox.textContent = `[${ts}] ${line}\n` + statusBox.textContent;
  if (statusBox.textContent.length > 4000) {
    statusBox.textContent = statusBox.textContent.slice(0, 4000);
  }
}
btnClearLog.onclick = () => { statusBox.textContent = ""; };

// ============================================================================
// Stick handling
// ============================================================================
function bindStick(el, key) {
  const thumb = el.querySelector(".stick-thumb");
  let active = false;
  let pointerId = null;

  function rect() { return el.getBoundingClientRect(); }

  function setFromPoint(clientX, clientY) {
    const r = rect();
    const cx = r.left + r.width / 2;
    const cy = r.top + r.height / 2;
    let dx = (clientX - cx) / (r.width / 2);
    let dy = (clientY - cy) / (r.height / 2);
    // Clamp to unit circle.
    const m = Math.sqrt(dx*dx + dy*dy);
    if (m > 1) { dx /= m; dy /= m; }
    // y axis: screen-down is +. Flip so up = +1.
    stick[key].x = dx;
    stick[key].y = -dy;
    // Reposition thumb.
    thumb.style.left = `${50 + dx * 50}%`;
    thumb.style.top  = `${50 + dy * 50}%`;
    updateStickLabel();
  }

  function recenter() {
    stick[key].x = 0;
    // Left stick: throttle holds last value; right stick recenters fully.
    if (key === "right") stick[key].y = 0;
    else                 stick[key].y = 0;   // yaw zeroes; throttle handled in computeIntent
    thumb.style.left = "50%";
    thumb.style.top  = "50%";
    updateStickLabel();
  }

  el.addEventListener("pointerdown", (e) => {
    el.setPointerCapture(e.pointerId);
    active = true;
    pointerId = e.pointerId;
    setFromPoint(e.clientX, e.clientY);
  });
  el.addEventListener("pointermove", (e) => {
    if (!active || e.pointerId !== pointerId) return;
    setFromPoint(e.clientX, e.clientY);
  });
  function release(e) {
    if (e && pointerId != null && e.pointerId !== pointerId) return;
    active = false; pointerId = null;
    recenter();
  }
  el.addEventListener("pointerup", release);
  el.addEventListener("pointercancel", release);
  el.addEventListener("pointerleave", (e) => {
    if (active) release(e);
  });
}
bindStick(leftStick, "left");
bindStick(rightStick, "right");

function updateStickLabel() {
  // Throttle in [0,1]: stick.y in [-1,1] maps to 0..1 (down stick = 0, up = 1).
  const thr = (stick.left.y + 1) / 2;
  leftVal.textContent = `thr ${thr.toFixed(2)} · yaw ${stick.left.x.toFixed(2)}`;
  rightVal.textContent = `pitch ${stick.right.y.toFixed(2)} · roll ${stick.right.x.toFixed(2)}`;
}
updateStickLabel();

// ============================================================================
// Send loop
// ============================================================================
function computeIntent() {
  if (estop) {
    return { pitch: 0, roll: 0, yaw: 0, throttle: 0.5, emergency_stop: true };
  }
  // Throttle: left stick vertical, mapped to [0, 1]. Hold the value when
  // the stick releases instead of springing back to 0 — that's how real
  // mode-switched RC sticks behave on a multi-prop. The hover-friendly
  // center value is 0.5, so we *do* spring throttle back to 0.5 when the
  // user lifts off if you'd rather; here we keep the last value so the
  // drone holds altitude in altctl/posctl.
  const thr = (stick.left.y + 1) / 2;
  lastThrottle = thr;
  return {
    pitch: stick.right.y,
    roll: stick.right.x,
    yaw: stick.left.x,
    throttle: thr,
    emergency_stop: false,
  };
}

let txTimer = null;
function startTxLoop() {
  const period = 1000 / 30; // 30 Hz from browser; backend resends at 50 Hz.
  if (txTimer) clearInterval(txTimer);
  txTimer = setInterval(() => {
    if (!wsReady) return;
    const intent = computeIntent();
    try {
      ws.send(JSON.stringify(intent));
      txStamps.push(performance.now());
      mThr.textContent = intent.throttle.toFixed(2);
      mYawCmd.textContent = intent.yaw.toFixed(2);
    } catch (e) {
      log("send failed: " + e.message);
    }
  }, period);
}

// ============================================================================
// Keyboard shortcuts: SPACE = e-stop, arm/disarm/takeoff/land
// ============================================================================
document.addEventListener("keydown", (e) => {
  if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT")) return;
  if (e.repeat) return;
  if (e.code === "Space") {
    e.preventDefault();
    estop = true;
    estopBanner.classList.add("on");
    log("E-STOP engaged");
    return;
  }
  const map = { "z": "arm", "x": "disarm", "t": "takeoff", "l": "land" };
  const ev = map[e.key.toLowerCase()];
  if (ev) { e.preventDefault(); sendEvent(ev); }
});
document.addEventListener("keyup", (e) => {
  if (e.code === "Space") {
    e.preventDefault();
    if (estop) {
      estop = false;
      estopBanner.classList.remove("on");
      log("E-STOP released");
    }
  }
});
window.addEventListener("blur", () => {
  // When focus is lost, recenter both sticks so the drone doesn't drift.
  stick.left.x = stick.left.y = 0;
  stick.right.x = stick.right.y = 0;
  leftStick.querySelector(".stick-thumb").style.left = "50%";
  leftStick.querySelector(".stick-thumb").style.top  = "50%";
  rightStick.querySelector(".stick-thumb").style.left = "50%";
  rightStick.querySelector(".stick-thumb").style.top  = "50%";
  updateStickLabel();
});

document.querySelectorAll(".evt").forEach(btn => {
  btn.addEventListener("click", () => {
    const evt = btn.getAttribute("data-evt");
    sendEvent(evt);
  });
});

function sendEvent(name) {
  if (!wsReady) return;
  try {
    ws.send(JSON.stringify({ type: "event", name }));
    log("event: " + name);
  } catch (e) { log("event failed: " + e.message); }
}

// ============================================================================
// WebSocket
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
        modePill.textContent = "mavlink: " + (m.mavlink_url || "—");
        log(`config: ${m.mavlink_url}`);
        break;
      case "status":
        sMav.textContent = m.connected ? "connected" : "disconnected";
        sMav.style.color = m.connected ? "#5cd35c" : "#ff5e5e";
        modePill.textContent = "mode: " + (m.mode || "—");
        armedPill.textContent = m.armed ? "ARMED" : "disarmed";
        armedPill.classList.toggle("armed", m.armed);
        break;
      case "pose":
        if (typeof m.x === "number") mX.textContent = m.x.toFixed(2);
        if (typeof m.y === "number") mY.textContent = m.y.toFixed(2);
        if (typeof m.z === "number") mZ.textContent = m.z.toFixed(2);
        if (typeof m.heading === "number")
          mYaw.textContent = (m.heading * 180 / Math.PI).toFixed(1);
        const sp = Math.sqrt((m.vx||0)**2 + (m.vy||0)**2 + (m.vz||0)**2);
        mSpd.textContent = sp.toFixed(2);
        if (typeof m.rtf === "number") mRtf.textContent = m.rtf.toFixed(2);
        break;
      case "camera":
        camImg.src = `data:image/jpeg;base64,${m.data_b64}`;
        camImg.classList.add("live");
        nofeed.style.display = "none";
        camTopic.textContent = "topic: " + (m.topic || "—");
        if (m.width && m.height) camSize.textContent = `${m.width}×${m.height}`;
        frameStamps.push(performance.now());
        break;
    }
  };
  ws.onclose = () => {
    wsReady = false; connDot.classList.remove("ok");
    log("WebSocket closed, retry 1s");
    setTimeout(connect, 1000);
  };
  ws.onerror = () => {};
}
connect();
startTxLoop();

// ============================================================================
// Hz / FPS counters
// ============================================================================
setInterval(() => {
  const now = performance.now();
  txStamps    = txStamps.filter(t => now - t < 2000);
  rxStamps    = rxStamps.filter(t => now - t < 2000);
  frameStamps = frameStamps.filter(t => now - t < 2000);
  sTx.textContent  = (txStamps.length / 2.0).toFixed(0) + " Hz";
  sRx.textContent  = (rxStamps.length / 2.0).toFixed(0) + " Hz";
  sFps.textContent = (frameStamps.length / 2.0).toFixed(1);
  // Drop the live class if the camera went dark.
  if (frameStamps.length === 0 && camImg.classList.contains("live")) {
    if (now - (camImg._lastFreshCheck || 0) > 3000) {
      camImg.classList.remove("live");
      nofeed.style.display = "flex";
    }
  } else if (frameStamps.length > 0) {
    camImg._lastFreshCheck = now;
  }
  camFps.textContent = (frameStamps.length / 2.0).toFixed(1) + " fps";
}, 250);
