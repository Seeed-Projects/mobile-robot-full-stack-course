/* M4 Perception Lab client.
 *
 * Browser-only: transport + presentation + metrics. NO ROS, NO algorithm
 * logic — see m4_demo_bringup/web_demo_server.py for the server side.
 *
 * Two transports, chosen by the server and reported on /api/demos:
 *   webrtc (h264_gst | vp8_aiortc) — offer/answer over /signaling, <video>.
 *   mjpeg  (multipart/x-mixed-replace) — plain <img src="/stream?module=...">.
 *
 * Two modes:
 *   single-demo (body[data-hub="0"]) — one module, served by its own server.
 *   hub         (body[data-hub="1"]) — several modules behind ONE server.
 *       Selecting a module sends {"type":"select","demo":"m4_X"} on the
 *       EXISTING websocket (the server swaps its frame source, so WebRTC is
 *       never renegotiated); over MJPEG the <img> src is re-pointed instead,
 *       which is equally cheap.
 *
 * Chapter identity lives in MODULE_REGISTRY rather than in the server: the hub
 * only declares the chapters it can actually run, so 4.3/4.4 (no model runtime
 * yet) would otherwise vanish from the interface entirely. Server readiness
 * always OVERRIDES the registry's blocked flag, so building the missing engine
 * is enough to light a chapter up — no code change here.
 *
 * Metric naming: "Pipeline FPS" (frames arriving from ROS, shown in the status
 * bar) and "Stream FPS" (frames the browser decoded, shown only in the Details
 * drawer) are different numbers and are never both called "FPS".
 */

const body = document.body;
const DEMO_KEY = body.dataset.demo || "m4_1";
const IS_HUB = body.dataset.hub === "1";

const video = document.getElementById("v");
const streamImg = document.getElementById("stream");
const liveEl = document.getElementById("live");
const transportEl = document.getElementById("transport");
const modulesEl = document.getElementById("modules");
const stageMsg = document.getElementById("stageMsg");
const stageMsgText = document.getElementById("stageMsgText");
const spinner = document.getElementById("spinner");
const pipelineModelEl = document.getElementById("pipelineModel");
const pipelineFpsEl = document.getElementById("pipelineFps");
const pipelineLatencyEl = document.getElementById("pipelineLatency");
const pipelineStateEl = document.getElementById("pipelineState");
const fsBtn = document.getElementById("fs");
const shotBtn = document.getElementById("shot");
const drawerEl = document.getElementById("drawer");
const drawerToggle = document.getElementById("drawerToggle");
const drawerClose = document.getElementById("drawerClose");
const settingsEl = document.getElementById("settings");
const settingsBtn = document.getElementById("settingsBtn");
const langBtn = document.getElementById("lang");
const qualityBtn = document.getElementById("quality");
const qualityRow = document.getElementById("qualityRow");
const dcCodecEl = document.getElementById("dcCodec");
const dcTransportEl = document.getElementById("dcTransport");
const dcStreamFpsEl = document.getElementById("dcStreamFps");
const dcFramesEl = document.getElementById("dcFrames");
const dcWebrtcEl = document.getElementById("dcWebrtc");
const dcRttEl = document.getElementById("dcRtt");
const dcTopicEl = document.getElementById("dcTopic");
const dcSizeEl = document.getElementById("dcSize");
const dcAgeEl = document.getElementById("dcAge");
const dcBackendEl = document.getElementById("dcBackend");
const dcStateEl = document.getElementById("dcState");

/* ---- chapter registry ---------------------------------------------------- */

// Stable identity for every M4 chapter: what it is, what runs it, and — for
// the ones whose model runtime is not on the device yet — what is missing.
// `blocked` is a DEFAULT, not a verdict: it only applies while the server does
// not declare the chapter.
const MODULE_REGISTRY = [
  {
    key: "m4_1",
    zh: "4.1 检测", en: "4.1 Detection",
    model: "YOLO11n · TensorRT FP16",
    blocked: false,
  },
  {
    key: "m4_2",
    zh: "4.2 跟踪", en: "4.2 Tracking",
    model: "ByteTrack",
    blocked: false,
  },
  {
    key: "m4_3",
    zh: "4.3 分割", en: "4.3 Segmentation",
    model: "SegFormer-B0 · FP16",
    blocked: true,
    missing: { zh: "TensorRT 引擎", en: "TensorRT engine" },
  },
  {
    key: "m4_4",
    zh: "4.4 6D 位姿", en: "4.4 6D Pose",
    model: "FoundationPose",
    blocked: true,
    missing: { zh: "FoundationPose 运行时", en: "FoundationPose runtime" },
  },
];

const STATE_WORD = {
  running: { zh: "运行中", en: "RUNNING" },
  blocked: { zh: "未就绪", en: "BLOCKED" },
  offline: { zh: "离线", en: "OFFLINE" },
};

let ws = null;
let pc = null;
let framesRendered = 0;
let framesTotal = 0;
let lastReportT = performance.now();
let streamKey = null;    // the chapter the SERVER is feeding
let selectedKey = null;  // the chapter the USER is looking at
let modules = [];
let transport = "webrtc";
// The server always serves an MJPEG crisp view at /stream, even when WebRTC is
// the primary transport. Remember the primary so the toggle can come back.
let primaryTransport = "webrtc";
let mjpegAvailable = false;
// Explicit user choice: "mjpeg" (crisp) or "primary" (server's transport).
// The 2 s /api/demos poll must not silently undo a manual toggle.
let userPref = null;
let mediaSize = "";
let backendName = "";
let rttMs = null;
let iceState = "";
let lang = (navigator.language || "zh").toLowerCase().startsWith("zh") ? "zh" : "en";
let reconnectTimer = null;
let reconnectDelay = 1000;

/* ---- i18n ---------------------------------------------------------------- */

function tr(zh, en) {
  return lang === "zh" ? zh : en;
}

function applyLang() {
  document.documentElement.lang = lang === "zh" ? "zh-CN" : "en";
  for (const el of document.querySelectorAll("[data-i18n-zh][data-i18n-en]")) {
    el.textContent = el.dataset[lang === "zh" ? "i18nZh" : "i18nEn"];
  }
  for (const el of document.querySelectorAll("[data-i18n-title-zh][data-i18n-title-en]")) {
    el.title = el.dataset[lang === "zh" ? "i18nTitleZh" : "i18nTitleEn"];
  }
  langBtn.textContent = lang === "zh" ? "EN" : "中文";
  renderAll();
}

/* ---- small helpers ------------------------------------------------------- */

function setLive(text, cls) {
  const label = liveEl.querySelector("span:last-child");
  if (label) label.textContent = text;
  liveEl.className = "live" + (cls ? " " + cls : "");
}

function moduleTitle(m) {
  if (!m) return "—";
  return lang === "zh" ? m.zh : m.en;
}

function selectedModule() {
  return modules.find((m) => m.key === selectedKey) || null;
}

/* ---- chapter state ------------------------------------------------------- */

// ready     -> the server has frames for it
// running   -> ready AND it is the chapter the server is streaming
// blocked   -> the server does not declare it AND the registry knows it cannot run
// offline   -> declared (or unknown) but no frames yet
function stateOf(m) {
  if (!m) return "offline";
  if (m.ready) return m.key === streamKey ? "running" : "ready";
  if (m.declared) return "offline";
  return m.blocked ? "blocked" : "offline";
}

/* ---- stage message ------------------------------------------------------- */

let stageMsgKind = null;   // {zh, en, spin} | {html}

function renderStageMsg() {
  if (!stageMsgKind) {
    stageMsg.classList.remove("show");
    return;
  }
  if (stageMsgKind.html) {
    spinner.style.display = "none";
    stageMsgText.innerHTML = stageMsgKind.html;
  } else {
    spinner.style.display = stageMsgKind.spin ? "" : "none";
    stageMsgText.textContent = tr(stageMsgKind.zh, stageMsgKind.en);
  }
  stageMsg.classList.add("show");
}

function showStageMsg(zh, en, spin) {
  stageMsgKind = { zh, en, spin: !!spin };
  renderStageMsg();
}

// A message that should not stick around: snapshot errors, not state.
function showTransient(zh, en, ms) {
  showStageMsg(zh, en, false);
  stageMsgKind.transient = true;
  setTimeout(() => {
    if (stageMsgKind && stageMsgKind.transient) clearStageMsg();
  }, ms || 2500);
}

function showEmptyState(m) {
  const state = stateOf(m);
  const missing = m.missing
    ? `<div class="empty-missing">${tr("缺失", "Missing")}: ${tr(m.missing.zh, m.missing.en)}</div>`
    : "";
  const reason = state === "blocked"
    ? tr("模型运行环境尚未就绪。", "Model runtime is not available yet.")
    : tr("该章节尚未产出画面。", "This chapter has no frames yet.");
  stageMsgKind = {
    html:
      `<div class="empty-tag">${m.key.replace("_", ".")}</div>` +
      `<div class="empty-title">${moduleTitle(m)}</div>` +
      `<div class="empty-model">${m.model}</div>` +
      `<div class="empty-reason">${reason}</div>` +
      missing +
      `<div class="empty-note">` +
      tr("其余章节不受影响，画面仍在播放。", "Other chapters are unaffected and keep playing.") +
      `</div>`,
  };
  renderStageMsg();
}

function clearStageMsg() {
  stageMsgKind = null;
  renderStageMsg();
}

/* A negotiated connection is not a picture. aiortc can take many seconds to
 * deliver the first decodable frame (the sender has to start pulling, and
 * H.264 needs an IDR before anything is visible). Clearing the overlay on
 * `ontrack` therefore leaves the viewer staring at a black rectangle with a
 * green "Live" indicator and no idea whether the demo is broken or late.
 *
 * Keep the overlay up until real pixels exist.
 */
let frameWatch = null;

function hasPicture() {
  return transport === "mjpeg"
    ? streamImg.naturalWidth > 0
    : video.videoWidth > 0;
}

function showConnecting() {
  stageMsgKind = {
    zh: "正在建立视频连接…", en: "Establishing video connection…",
    spin: true, conn: true,
  };
  renderStageMsg();
}

function stopFrameWatch() {
  if (frameWatch) { clearInterval(frameWatch); frameWatch = null; }
}

function awaitFirstFrame() {
  stopFrameWatch();
  frameWatch = setInterval(() => {
    if (!hasPicture()) return;
    stopFrameWatch();
    setLive(tr("实时", "Live"), "on");
    // Only retire the *connecting* overlay. A blocked/empty chapter must keep
    // its message even if the shared stream underneath is still running.
    if (stageMsgKind && stageMsgKind.conn) clearStageMsg();
  }, 250);
}

/* ---- module switching ---------------------------------------------------- */

function selectModule(key) {
  const m = modules.find((x) => x.key === key);
  if (!m) return;
  selectedKey = key;
  closeSettings();

  const state = stateOf(m);

  if (state === "blocked") {
    // A chapter whose runtime is missing must never disturb the live stream.
    showEmptyState(m);
    renderAll();
    return;
  }

  if (!IS_HUB) {
    // A single-demo server serves exactly one topic and has no select path.
    if (key === DEMO_KEY) clearStageMsg();
    else {
      showStageMsg(
        `${moduleTitle(m)} 仅在统一 hub 页面中可切换（当前为单模块页面）。`,
        `${moduleTitle(m)} can only be selected from the unified hub page.`,
        false
      );
    }
    renderAll();
    return;
  }

  if (!m.declared) {
    showEmptyState(m);
    renderAll();
    return;
  }

  sendSelect(key);
}

function sendSelect(key) {
  streamKey = key;
  if (transport === "mjpeg") {
    // Over MJPEG a module switch is just a new multipart request.
    streamImg.src = "/stream?module=" + encodeURIComponent(key) + "&t=" + Date.now();
    awaitFirstFrame();
  } else if (ws && ws.readyState === WebSocket.OPEN) {
    // `"select"` is the wire contract asserted by m4_web_hub_regression.py.
    ws.send(JSON.stringify({ type: "select", demo: key }));
  }
  if (hasPicture()) clearStageMsg();
  renderAll();
}

/* ---- rendering ----------------------------------------------------------- */

function renderSidebar() {
  if (!modulesEl) return;
  modulesEl.innerHTML = "";
  for (const m of modules) {
    const state = stateOf(m);
    const tab = document.createElement("button");
    tab.type = "button";
    tab.className = "tab" +
      (m.key === selectedKey ? " active" : "") +
      (state === "blocked" ? " blocked" : "");

    const dot = document.createElement("span");
    dot.className = "dot " +
      (state === "running" ? "running" :
       state === "ready" ? "ready" :
       state === "blocked" ? "blocked" : "offline");

    const bodyEl = document.createElement("div");
    bodyEl.className = "m-body";
    const name = document.createElement("span");
    name.className = "m-name";
    name.textContent = moduleTitle(m);
    bodyEl.appendChild(name);

    // Status text only where it carries information: a missing runtime and the
    // chapter actually being streamed. Everything else is just the dot.
    if (state === "blocked" || state === "running") {
      const word = document.createElement("span");
      word.className = "m-state";
      word.textContent = tr(STATE_WORD[state].zh, STATE_WORD[state].en);
      bodyEl.appendChild(word);
    }

    tab.appendChild(dot);
    tab.appendChild(bodyEl);
    tab.title = (m.topic || "") + (state === "blocked"
      ? tr("（模型运行环境缺失）", " (model runtime missing)") : "");
    tab.addEventListener("click", () => selectModule(m.key));
    modulesEl.appendChild(tab);
  }
}

function renderPipelineInfo() {
  const m = selectedModule();
  if (!m) {
    pipelineModelEl.textContent = "—";
    pipelineFpsEl.textContent = "—";
    pipelineLatencyEl.textContent = "—";
    pipelineStateEl.hidden = true;
    return;
  }
  const state = stateOf(m);
  pipelineModelEl.textContent = m.model;

  if (state === "running" || state === "ready") {
    // Real ROS-side rate, counted by the server on the overlay subscription.
    pipelineFpsEl.textContent = (typeof m.fps === "number")
      ? m.fps.toFixed(1) + " FPS" : "-- FPS";
    pipelineLatencyEl.textContent = (typeof m.age_ms === "number")
      ? Math.round(m.age_ms) + " ms" : "—";
    pipelineStateEl.hidden = true;
  } else {
    pipelineFpsEl.textContent = "-- FPS";
    pipelineLatencyEl.textContent = "—";
    const word = STATE_WORD[state] || STATE_WORD.offline;
    pipelineStateEl.textContent = tr(word.zh, word.en);
    pipelineStateEl.hidden = false;
  }
}

function renderDrawer() {
  const m = selectedModule();
  const state = m ? stateOf(m) : "offline";
  dcCodecEl.textContent = transport === "mjpeg"
    ? "JPEG"
    : (backendName === "h264_gst" ? "H.264" : "VP8");
  dcTransportEl.textContent = backendName || transport;
  dcFramesEl.textContent = String(framesTotal);
  dcRttEl.textContent = rttMs == null ? "—" : rttMs.toFixed(0) + " ms";
  dcWebrtcEl.textContent = iceState || (transport === "mjpeg" ? "MJPEG" : "—");
  dcTopicEl.textContent = (m && m.topic) || "—";
  dcSizeEl.textContent = mediaSize || "—";
  dcAgeEl.textContent = (m && typeof m.age_ms === "number")
    ? Math.round(m.age_ms) + " ms" : "—";
  dcBackendEl.textContent = backendName || "—";
  const word = STATE_WORD[state];
  dcStateEl.textContent = word ? tr(word.zh, word.en) : tr("就绪", "READY");
}

function renderAll() {
  renderSidebar();
  renderPipelineInfo();
  renderDrawer();
}

/* ---- module list polling ------------------------------------------------- */

// Merge the static chapter registry with whatever the server declares, keyed
// by chapter. Registry order wins; a chapter the server declares but the
// registry does not know about is appended so the UI stays forward-compatible.
function mergeModules(serverModules, health) {
  const byKey = new Map((serverModules || []).map((m) => [m.key, m]));
  const merged = [];

  for (const reg of MODULE_REGISTRY) {
    const srv = byKey.get(reg.key);
    if (srv) byKey.delete(reg.key);
    merged.push({
      key: reg.key,
      zh: reg.zh, en: reg.en,
      model: reg.model,
      blocked: !!reg.blocked,
      missing: reg.missing,
      declared: !!srv,
      topic: srv ? srv.topic : "",
      ready: srv ? !!srv.ready : false,
      fps: srv ? srv.fps : undefined,
      age_ms: srv ? srv.age_ms : undefined,
    });
  }
  for (const srv of byKey.values()) {
    merged.push({
      key: srv.key, zh: srv.title, en: srv.title,
      model: "", blocked: false, declared: true,
      topic: srv.topic, ready: !!srv.ready, fps: srv.fps, age_ms: srv.age_ms,
    });
  }

  // A single-demo server declares no module list at all — it serves exactly
  // one topic, so synthesise that chapter and take readiness from /healthz.
  if (!IS_HUB) {
    const me = merged.find((m) => m.key === DEMO_KEY);
    if (me) {
      me.declared = true;
      me.ready = !!(health && health.ros_frame_ready);
      me.topic = me.topic || tr("单模块页面", "single-module page");
    }
  }
  modules = merged;
}

async function refreshModules() {
  try {
    const [demosRes, healthRes] = await Promise.all([
      fetch("/api/demos", { cache: "no-store" }),
      fetch("/healthz", { cache: "no-store" }),
    ]);
    const data = await demosRes.json();
    let health = null;
    try { health = await healthRes.json(); } catch (e) { /* keep going */ }

    mergeModules(data.modules || [], health);
    mjpegAvailable = data.mjpeg === true;
    if (qualityRow) qualityRow.hidden = !mjpegAvailable;

    // Respect a manual choice; otherwise follow the server.
    if (userPref === "mjpeg") {
      if (transport !== "mjpeg") setTransport("mjpeg", { force: true });
    } else if (data.transport) {
      setTransport(data.transport);
    }

    if (health && health.active) streamKey = health.active;
    if (!selectedKey) {
      // First load: prefer a chapter that actually has frames, so opening the
      // page during a partially-ready bring-up never shows a black rectangle.
      const firstReady = modules.find((m) => m.ready);
      selectedKey = (firstReady || modules[0]).key;
      if (IS_HUB && streamKey) selectedKey = streamKey;
    }
    if (selectedKey && streamKey && selectedKey === streamKey && hasPicture()) clearStageMsg();

    renderAll();
  } catch (e) {
    /* keep the last known state */
  }
}

function setTransport(name, opts) {
  const force = !!(opts && opts.force);
  if (!name) return;
  if (name === transport && !force) return;
  const wasMjpeg = transport === "mjpeg";
  transport = name === "mjpeg" ? "mjpeg" : "webrtc";
  if (transport !== "mjpeg") primaryTransport = name;
  backendName = name;
  const label = transport === "mjpeg" ? "MJPEG" : (name === "h264_gst" ? "H.264 HW" : "VP8");
  transportEl.textContent = label;
  transportEl.className = "badge " + (transport === "mjpeg" ? "warn" : "ok");
  applyTransport();
  renderQualityBtn();
  renderDrawer();
  // Coming back from MJPEG needs a fresh WebRTC negotiation.
  if (wasMjpeg && transport !== "mjpeg") connect();
}

function renderQualityBtn() {
  if (!qualityBtn) return;
  const onMjpeg = transport === "mjpeg";
  qualityBtn.textContent = onMjpeg
    ? tr("低延迟模式", "Low-latency mode")
    : tr("清晰模式", "Crisp mode");
  qualityBtn.title = onMjpeg
    ? tr("切回 WebRTC 低延迟码流", "Switch back to the low-latency WebRTC stream")
    : tr("切到 MJPEG 逐帧清晰码流（无编码模糊）",
         "Switch to the per-frame MJPEG stream (no codec blur)");
  qualityBtn.setAttribute("aria-pressed", onMjpeg ? "true" : "false");
}

function applyTransport() {
  if (transport === "mjpeg") {
    video.hidden = true;
    streamImg.hidden = false;
    if (!streamImg.src) {
      streamImg.src = "/stream?module=" + encodeURIComponent(selectedKey || DEMO_KEY) +
        "&t=" + Date.now();
    }
    iceState = "MJPEG";
    showConnecting();
    awaitFirstFrame();
    if (ws) { try { ws.close(); } catch (e) {} ws = null; }
    if (pc) { try { pc.close(); } catch (e) {} pc = null; }
  } else {
    streamImg.hidden = true;
    video.hidden = false;
  }
}

/* ---- WebRTC transport ---------------------------------------------------- */

// Ask the browser to OFFER the codec the server can actually send.
//
// aiortc answers with its own preference order, which puts VP8 first. When the
// server is feeding pre-encoded hardware H.264, a VP8 answer makes aiortc hand
// H.264 NALs to its VP8 packer and the browser decodes nothing (the page looks
// "streaming" but videoWidth stays 0). setCodecPreferences is the standards
// way to control this, and the server additionally falls back to MJPEG if the
// negotiated codec still is not H.264.
function preferCodec(pc, transportName) {
  try {
    const tr = pc.getTransceivers()[0];
    if (!tr || typeof tr.setCodecPreferences !== "function") return;
    const caps = window.RTCRtpReceiver && RTCRtpReceiver.getCapabilities
      ? RTCRtpReceiver.getCapabilities("video") : null;
    if (!caps || !caps.codecs || !caps.codecs.length) return;
    const wantH264 = String(transportName || "").toLowerCase().includes("h264");
    const want = wantH264 ? /H264/i : /VP8/i;
    const preferred = caps.codecs.filter((c) => want.test(c.mimeType));
    if (!preferred.length) return;
    const rest = caps.codecs.filter((c) => !want.test(c.mimeType));
    tr.setCodecPreferences(preferred.concat(rest));
  } catch (e) {
    /* Older browser: the server-side MJPEG fallback still covers us. */
  }
}

async function connect() {
  if (transport === "mjpeg") return;
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  setLive(tr("连接中", "connecting"), "");
  showConnecting();

  ws = new WebSocket(
    (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/signaling"
  );

  ws.onopen = () => {
    reconnectDelay = 1000;
    pc = new RTCPeerConnection();
    pc.ontrack = (ev) => {
      if (ev.streams && ev.streams[0]) {
        video.srcObject = ev.streams[0];
      } else {
        // Some aiortc paths have an empty streams[].
        const ms = new MediaStream();
        ms.addTrack(ev.track);
        video.srcObject = ms;
      }
      iceState = pc.iceConnectionState;
      // NOT clearStageMsg(): the overlay stays until a frame actually lands.
      awaitFirstFrame();
      renderDrawer();
    };
    pc.oniceconnectionstatechange = () => {
      iceState = pc.iceConnectionState;
      if (iceState === "connected" || iceState === "completed") {
        setLive(tr("实时", "Live"), "on");
      } else if (iceState === "failed" || iceState === "disconnected") {
        setLive(iceState, "err");
      }
      renderDrawer();
    };
    pc.addTransceiver("video", { direction: "recvonly" });
    preferCodec(pc, primaryTransport);
    pc.createOffer()
      .then((offer) => pc.setLocalDescription(offer).then(() => offer))
      .then((offer) => {
        ws.send(JSON.stringify({ type: "offer", sdp: offer.sdp, kind: offer.type }));
        // Re-assert the selected module after a (re)connect.
        if (IS_HUB && streamKey) {
          ws.send(JSON.stringify({ type: "select", demo: streamKey }));
        }
      })
      .catch(() => {
        setLive(tr("协商失败", "offer failed"), "err");
        showStageMsg("视频协商失败。", "Video negotiation failed.", false);
      });
  };

  ws.onmessage = async (ev) => {
    if (!pc) return;
    const msg = JSON.parse(ev.data);
    if (msg.type === "answer") {
      await pc.setRemoteDescription({ type: msg.kind, sdp: msg.sdp });
    } else if (msg.type === "selected") {
      streamKey = msg.demo;
      renderAll();
    } else if (msg.type === "transport") {
      // Server-side fallback: e.g. hardware H.264 was NOT negotiated for this
      // peer, so the server is directing us to the MJPEG stream. Without this
      // the client would sit on a stream whose codec mismatch produces garbage.
      // Server-mandated (e.g. H.264 was not the negotiated send codec), so it
      // outranks both the poll and a previous manual choice.
      userPref = msg.transport === "mjpeg" ? "mjpeg" : "primary";
      setTransport(msg.transport, { force: true });
    } else if (msg.type === "error") {
      // A rejected switch must not leave the page claiming a chapter it does
      // not have; fall back to whatever the server is really feeding.
      streamKey = null;
      refreshModules();
    }
  };

  ws.onclose = () => {
    ws = null;
    setLive(tr("已断开", "disconnected"), "err");
    iceState = "closed";
    renderDrawer();
    if (transport !== "mjpeg" && !reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connect();
      }, reconnectDelay);
      reconnectDelay = Math.min(reconnectDelay * 2, 10000);
    }
  };
  ws.onerror = () => setLive(tr("连接错误", "ws error"), "err");
}

/* ---- metrics ------------------------------------------------------------- */

function noteFrame() {
  framesRendered++;
  framesTotal++;
}

function metricsLoop() {
  setInterval(async () => {
    try {
      const now = performance.now();
      const dt = (now - lastReportT) / 1000;
      if (dt > 0) dcStreamFpsEl.textContent = (framesRendered / dt).toFixed(1);
      framesRendered = 0;
      lastReportT = now;
      dcFramesEl.textContent = String(framesTotal);

      const w = transport === "mjpeg" ? streamImg.naturalWidth : video.videoWidth;
      const h = transport === "mjpeg" ? streamImg.naturalHeight : video.videoHeight;
      if (w && h) {
        const s = w + "×" + h;
        if (s !== mediaSize) { mediaSize = s; dcSizeEl.textContent = s; }
      }

      if (pc) {
        const stats = await pc.getStats();
        stats.forEach((s) => {
          if (s.type === "candidate-pair" && s.state === "succeeded" &&
              s.currentRoundTripTime != null) {
            rttMs = s.currentRoundTripTime * 1000;
          }
        });
        dcRttEl.textContent = rttMs == null ? "—" : rttMs.toFixed(0) + " ms";
      }
    } catch (e) {
      /* ignore */
    }
  }, 1000);

  // Modern browsers: per-frame callback. Falls back to nothing in older ones.
  if ("requestVideoFrameCallback" in HTMLVideoElement.prototype) {
    const onFrame = () => {
      if (transport !== "mjpeg") noteFrame();
      video.requestVideoFrameCallback(onFrame);
    };
    video.requestVideoFrameCallback(onFrame);
  }
  // MJPEG has no frame callback; count element loads instead.
  streamImg.addEventListener("load", () => { if (transport === "mjpeg") noteFrame(); });
  streamImg.addEventListener("error", () => {
    if (transport === "mjpeg") setLive(tr("码流中断", "stream error"), "err");
  });
}

/* ---- drawer + settings --------------------------------------------------- */

function setDrawerOpen(open) {
  drawerEl.classList.toggle("open", open);
  drawerToggle.setAttribute("aria-pressed", open ? "true" : "false");
}

function toggleDrawer() {
  setDrawerOpen(!drawerEl.classList.contains("open"));
}

function closeSettings() {
  settingsEl.hidden = true;
  settingsBtn.setAttribute("aria-pressed", "false");
}

function toggleSettings() {
  const open = settingsEl.hidden;
  settingsEl.hidden = !open;
  settingsBtn.setAttribute("aria-pressed", open ? "true" : "false");
}

/* ---- controls ------------------------------------------------------------ */

fsBtn.addEventListener("click", () => {
  if (document.fullscreenElement) document.exitFullscreen();
  else document.documentElement.requestFullscreen?.();
});

shotBtn.addEventListener("click", () => {
  const src = transport === "mjpeg" ? streamImg : video;
  const w = transport === "mjpeg" ? streamImg.naturalWidth : video.videoWidth;
  const h = transport === "mjpeg" ? streamImg.naturalHeight : video.videoHeight;
  if (!w || !h) {
    showTransient("暂无画面可截图。", "No frame to capture yet.");
    return;
  }
  const c = document.createElement("canvas");
  c.width = w; c.height = h;
  c.getContext("2d").drawImage(src, 0, 0, w, h);
  c.toBlob((blob) => {
    if (!blob) { showTransient("截图失败。", "Snapshot failed."); return; }
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = (selectedKey || DEMO_KEY) + "_" + new Date().toISOString().replace(/[:.]/g, "-") + ".png";
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  }, "image/png");
});

langBtn.addEventListener("click", () => {
  lang = lang === "zh" ? "en" : "zh";
  applyLang();
});

qualityBtn.addEventListener("click", () => {
  // Toggle between the primary WebRTC stream and the always-available MJPEG
  // crisp view. Purely client-side: no server round trip, no restart.
  if (transport === "mjpeg") {
    userPref = "primary";
    setTransport(primaryTransport, { force: true });
  } else {
    userPref = "mjpeg";
    setTransport("mjpeg", { force: true });
  }
});

drawerToggle.addEventListener("click", toggleDrawer);
drawerClose.addEventListener("click", () => setDrawerOpen(false));
settingsBtn.addEventListener("click", (e) => {
  e.stopPropagation();
  toggleSettings();
});
settingsEl.addEventListener("click", (e) => e.stopPropagation());
document.addEventListener("click", () => closeSettings());
document.addEventListener("keydown", (e) => {
  if (e.key !== "Escape") return;
  closeSettings();
  setDrawerOpen(false);
});

/* ---- boot ---------------------------------------------------------------- */

applyLang();
mergeModules([], null);
selectedKey = IS_HUB ? null : DEMO_KEY;
streamKey = IS_HUB ? null : DEMO_KEY;
renderAll();
metricsLoop();
// The first offer must already prefer the server's codec, so wait for
// /api/demos (which reports it) before negotiating.
refreshModules().finally(() => {
  if (transport !== "mjpeg") connect();
});
setInterval(refreshModules, 2000);
