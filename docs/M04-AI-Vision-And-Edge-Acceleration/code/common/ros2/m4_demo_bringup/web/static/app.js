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
const videoWrap = document.getElementById("videoWrap");
const liveEl = document.getElementById("live");
const transportEl = document.getElementById("transport");
const modulesEl = document.getElementById("modules");
const stageMsg = document.getElementById("stageMsg");
const stageMsgText = document.getElementById("stageMsgText");
const spinner = document.getElementById("spinner");
const pipelineModelEl = document.getElementById("pipelineModel");
const activeTitleEl = document.getElementById("activeTitle");
const pipelineFpsEl = document.getElementById("pipelineFps");
const pipelineStateEl = document.getElementById("pipelineState");
const activeInputSourceEl = document.getElementById("activeInputSource");
const activeInputFileEl = document.getElementById("activeInputFile");
const segmentationResultsEl = document.getElementById("segmentationResults");
const classResultsEl = document.getElementById("classResults");
const drivableRatioEl = document.getElementById("drivableRatio");
const poseResultsEl = document.getElementById("poseResults");
const poseStatusEl = document.getElementById("poseStatus");
const posePositionEl = document.getElementById("posePosition");
const poseQuatEl = document.getElementById("poseQuat");
const poseNormEl = document.getElementById("poseNorm");
const poseRateEl = document.getElementById("poseRate");
const inputControlBarEl = document.getElementById("inputControlBar");
const poseInputNoteEl = document.getElementById("poseInputNote");
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
const quickCodecEl = document.getElementById("quickCodec");
const quickTransportEl = document.getElementById("quickTransport");
const quickConnectionEl = document.getElementById("quickConnection");
const quickRttEl = document.getElementById("quickRtt");
const undistortToggle = document.getElementById("undistortToggle");
const undistortStateEl = document.getElementById("undistortState");
const undistortHintEl = document.getElementById("undistortHint");
const parameterBtn = document.getElementById("parameterBtn");
const parameterDrawer = document.getElementById("parameterDrawer");
const parameterClose = document.getElementById("parameterClose");
const parameterControls = document.getElementById("parameterControls");
const parameterReset = document.getElementById("parameterReset");
const controlToast = document.getElementById("controlToast");
const cameraSourceBtn = document.getElementById("cameraSourceBtn");
const videoSourceBtn = document.getElementById("videoSourceBtn");
const videoFileInput = document.getElementById("videoFileInput");
const uploadVideoBtn = document.getElementById("uploadVideoBtn");
const deleteVideoBtn = document.getElementById("deleteVideoBtn");
const videoFileSummary = document.getElementById("videoFileSummary");
const uploadProgress = document.getElementById("uploadProgress");
const uploadProgressBar = document.getElementById("uploadProgressBar");

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
    blocked: false,
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
  starting: { zh: "启动中", en: "STARTING" },
  idle: { zh: "未启动", en: "IDLE" },
  unavailable: { zh: "不可用", en: "UNAVAILABLE" },
  blocked: { zh: "未就绪", en: "BLOCKED" },
  offline: { zh: "离线", en: "OFFLINE" },
};

let ws = null;
let pc = null;
let framesRendered = 0;
let framesTotal = 0;
let lastReportT = performance.now();
let lastDecodedFrames = null;
let lastDecodedAt = null;
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
let reconnectSuppressed = false;
let inputState = null;
let visualizationState = { view: "semantic", options: [], results: null };
let poseState = { results: null };
let inputBusy = false;
let visualizationBusy = false;
let lastInputError = "";

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
  if (runtimeSettings) renderParameterControls();
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
  if (m.status === "unavailable") return "unavailable";
  if (m.status === "idle") return "idle";
  if (m.status === "starting") return "starting";
  if (m.status === "offline") return "offline";
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
  const reason = m.error
    ? m.error
    : state === "unavailable" || state === "blocked"
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

  if (state === "blocked" || state === "unavailable") {
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

  if (IS_HUB) history.pushState({}, "", "/m4/" + key.replace("m4_", ""));
  showStageMsg(
    `正在切换到 ${moduleTitle(m)}，上一模块将停止…`,
    `Switching to ${moduleTitle(m)}; stopping the previous module…`,
    true
  );
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
      (state === "blocked" || state === "unavailable" ? " blocked" : "");

    const dot = document.createElement("span");
    dot.className = "dot " +
      (state === "running" ? "running" :
       state === "ready" ? "ready" :
       state === "blocked" || state === "unavailable" ? "blocked" : "offline");

    const bodyEl = document.createElement("div");
    bodyEl.className = "m-body";
    const name = document.createElement("span");
    name.className = "m-name";
    name.textContent = moduleTitle(m);
    bodyEl.appendChild(name);

    // Status text only where it carries information: a missing runtime and the
    // chapter actually being streamed. Everything else is just the dot.
    if (state === "blocked" || state === "unavailable" || state === "idle" || state === "starting" || state === "running") {
      const word = document.createElement("span");
      word.className = "m-state";
      word.textContent = tr(STATE_WORD[state].zh, STATE_WORD[state].en);
      bodyEl.appendChild(word);
    }

    tab.appendChild(dot);
    tab.appendChild(bodyEl);
    tab.title = (m.topic || "") + (m.error ? ` (${m.error})` : "");
    tab.addEventListener("click", () => selectModule(m.key));
    modulesEl.appendChild(tab);
  }
}

function renderPipelineInfo() {
  const m = selectedModule();
  if (!m) {
    activeTitleEl.textContent = tr("M4 感知", "M4 Perception");
    pipelineModelEl.textContent = "—";
    pipelineFpsEl.textContent = "—";
    pipelineStateEl.hidden = true;
    return;
  }
  const state = stateOf(m);
  activeTitleEl.textContent = moduleTitle(m);
  pipelineModelEl.textContent = m.model;

  if (state === "running" || state === "ready") {
    // Real ROS-side rate, counted by the server on the overlay subscription.
    pipelineFpsEl.textContent = (typeof m.fps === "number")
      ? m.fps.toFixed(1) + " FPS" : "-- FPS";
    pipelineStateEl.hidden = true;
  } else {
    pipelineFpsEl.textContent = "-- FPS";
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
  quickCodecEl.textContent = dcCodecEl.textContent;
  quickTransportEl.textContent = dcTransportEl.textContent;
  quickConnectionEl.textContent = dcWebrtcEl.textContent;
  quickRttEl.textContent = dcRttEl.textContent;
}

const CLASS_NAMES_ZH = {
  road: "道路", sidewalk: "人行道", building: "建筑", wall: "墙体",
  fence: "围栏", pole: "杆体", "traffic light": "交通灯",
  "traffic sign": "交通标志", vegetation: "植被", terrain: "地面",
  sky: "天空", person: "行人", rider: "骑行者", car: "汽车",
  truck: "卡车", bus: "公交车", train: "列车", motorcycle: "摩托车",
  bicycle: "自行车",
};

function renderInputState() {
  if (!inputState) return;
  const videoStatus = inputState.video && inputState.video.status;
  const isVideo = inputState.source === "video" || videoStatus === "starting" || videoStatus === "playing";
  const sourceLabel = videoStatus === "starting"
    ? tr("正在启动本地视频", "Starting local video")
    : isVideo
      ? tr("本地视频", "Local video")
      : tr("实时相机", "Live camera");
  activeInputSourceEl.textContent = sourceLabel;
  cameraSourceBtn.setAttribute("aria-pressed", String(!isVideo));
  videoSourceBtn.setAttribute("aria-pressed", String(isVideo));
  cameraSourceBtn.disabled = inputBusy;
  videoSourceBtn.disabled = inputBusy || !inputState.video;
  uploadVideoBtn.disabled = inputBusy;
  deleteVideoBtn.hidden = !inputState.video;
  deleteVideoBtn.disabled = inputBusy;
  activeInputFileEl.hidden = !isVideo || !inputState.video;
  activeInputFileEl.textContent = inputState.video ? inputState.video.filename : "";
  if (inputState.video) {
    const sizeMb = (inputState.video.size_bytes / 1024 / 1024).toFixed(1);
    const loops = inputState.video.loop_count || 0;
    const loopText = videoStatus === "playing"
      ? `${tr("循环", "loops")} ${loops}`
      : videoStatus === "starting"
        ? tr("正在启动", "starting")
        : videoStatus === "error"
          ? tr("视频启动失败", "video start failed")
          : tr("未播放", "idle");
    videoFileSummary.textContent = `${inputState.video.filename} · ${inputState.video.codec.toUpperCase()} · ${sizeMb} MB · ${loopText}`;
  } else {
    videoFileSummary.textContent = tr("尚未上传视频", "No video uploaded");
  }
  const error = inputState.error || (inputState.video && inputState.video.error) || "";
  if (error && error !== lastInputError) {
    showControlToast(error, "err");
  }
  lastInputError = error;
  if (undistortToggle && runtimeSettings) renderUndistortState();
}

function renderVisualization() {
  const visible = selectedKey === "m4_3";
  segmentationResultsEl.hidden = !visible;
  for (const button of segmentationResultsEl.querySelectorAll("[data-view]")) {
    button.setAttribute("aria-pressed", String(button.dataset.view === visualizationState.view));
    button.disabled = visualizationBusy;
  }
  classResultsEl.innerHTML = "";
  const results = visualizationState.results;
  if (!visible || !results) {
    drivableRatioEl.textContent = "—";
    return;
  }
  for (const item of (results.top_classes || []).slice(0, 5)) {
    const chip = document.createElement("span");
    chip.className = "class-chip";
    const swatch = document.createElement("i");
    swatch.style.backgroundColor = item.color;
    const label = document.createElement("span");
    const name = lang === "zh" ? (CLASS_NAMES_ZH[item.name] || item.name) : item.name;
    label.textContent = `${name} ${(Number(item.ratio) * 100).toFixed(0)}%`;
    chip.append(swatch, label);
    classResultsEl.appendChild(chip);
  }
  drivableRatioEl.textContent = `${(Number(results.drivable_ratio || 0) * 100).toFixed(1)}%`;
}

function renderPosePanel() {
  const visible = selectedKey === "m4_4";
  poseResultsEl.hidden = !visible;
  // The shared input bar (camera / local video / undistortion / detection
  // parameters) belongs to 4.1-4.3; M4.4 replays the official example bag
  // inside the Isaac ROS container, so none of those controls apply.
  if (inputControlBarEl) inputControlBarEl.hidden = visible;
  if (poseInputNoteEl) poseInputNoteEl.hidden = !visible;
  const results = poseState.results;
  if (!visible || !results) {
    poseStatusEl.textContent = "—";
    posePositionEl.textContent = "—";
    poseQuatEl.textContent = "—";
    poseNormEl.textContent = "—";
    poseRateEl.textContent = "—";
    return;
  }
  const statusWords = {
    waiting_pose: tr("等待位姿", "waiting for pose"),
    valid_pose: tr("有效位姿", "valid pose"),
    pose_stale: tr("位姿过期", "pose stale"),
  };
  poseStatusEl.textContent = statusWords[results.status] || results.status || "—";
  if (Array.isArray(results.position_m) && results.position_m.length === 3) {
    posePositionEl.textContent = results.position_m.map((v) => Number(v).toFixed(3)).join(", ");
  } else {
    posePositionEl.textContent = "—";
  }
  if (Array.isArray(results.quaternion_xyzw) && results.quaternion_xyzw.length === 4) {
    poseQuatEl.textContent = results.quaternion_xyzw.map((v) => Number(v).toFixed(3)).join(", ");
  } else {
    poseQuatEl.textContent = "—";
  }
  poseNormEl.textContent = Number.isFinite(Number(results.quaternion_norm))
    ? Number(results.quaternion_norm).toFixed(4)
    : "—";
  poseRateEl.textContent = Number.isFinite(Number(results.pose_rate_hz))
    ? `${Number(results.pose_rate_hz).toFixed(1)} Hz`
    : "—";
}

function renderAll() {
  renderSidebar();
  renderPipelineInfo();
  renderDrawer();
  renderInputState();
  renderVisualization();
  renderPosePanel();
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
      status: srv ? srv.status : undefined,
      error: srv ? srv.error : null,
      last_frame_age_ms: srv ? srv.last_frame_age_ms : undefined,
    });
  }
  for (const srv of byKey.values()) {
    merged.push({
      key: srv.key, zh: srv.title, en: srv.title,
      model: "", blocked: false, declared: true,
      topic: srv.topic, ready: !!srv.ready, fps: srv.fps, age_ms: srv.age_ms,
      status: srv.status, error: srv.error, last_frame_age_ms: srv.last_frame_age_ms,
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
      me.status = me.ready ? "ready" : "starting";
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

    if (health && health.active && !streamKey) streamKey = health.active;
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
    lastDecodedFrames = null;
    lastDecodedAt = null;
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
      if (msg.code === "codec_unavailable") {
        // The Jetson intentionally skips a second MJPEG encoder by default.
        // Stop retrying a codec this browser cannot decode and show the
        // actionable recovery path instead of leaving a black video element.
        reconnectSuppressed = true;
        setLive(tr("浏览器不兼容", "browser incompatible"), "err");
        showStageMsg(
          msg.message || "当前浏览器无法协商 H.264。",
          "This browser could not negotiate H.264.",
          false,
        );
        try { await pc.close(); } catch (_) { /* already closed */ }
      } else {
        // A rejected switch must not leave the page claiming a chapter it does
        // not have; fall back to whatever the server is really feeding.
        streamKey = null;
        refreshModules();
      }
    }
  };

  ws.onclose = () => {
    ws = null;
    setLive(tr("已断开", "disconnected"), "err");
    iceState = "closed";
    renderDrawer();
    if (transport !== "mjpeg" && !reconnectTimer && !reconnectSuppressed) {
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
      let browserFps = framesRendered / Math.max(dt, 0.001);
      if (dt > 0) {
        const label = browserFps.toFixed(1) + " FPS";
        dcStreamFpsEl.textContent = label;
      }
      framesRendered = 0;
      lastReportT = now;
      dcFramesEl.textContent = String(framesTotal);

      const w = transport === "mjpeg" ? streamImg.naturalWidth : video.videoWidth;
      const h = transport === "mjpeg" ? streamImg.naturalHeight : video.videoHeight;
      if (w && h) {
        const s = w + "×" + h;
        if (s !== mediaSize) {
          mediaSize = s;
          dcSizeEl.textContent = s;
        }
      }

      if (pc) {
        const stats = await pc.getStats();
        stats.forEach((s) => {
          if (s.type === "inbound-rtp" && s.kind === "video" &&
              Number.isFinite(s.framesDecoded)) {
            if (lastDecodedFrames != null && lastDecodedAt != null && s.timestamp > lastDecodedAt) {
              browserFps = (s.framesDecoded - lastDecodedFrames) /
                ((s.timestamp - lastDecodedAt) / 1000);
              const label = Math.max(0, browserFps).toFixed(1) + " FPS";
              dcStreamFpsEl.textContent = label;
            }
            lastDecodedFrames = s.framesDecoded;
            lastDecodedAt = s.timestamp;
          }
          if (s.type === "candidate-pair" && s.state === "succeeded" &&
              s.currentRoundTripTime != null) {
            rttMs = s.currentRoundTripTime * 1000;
          }
        });
        dcRttEl.textContent = rttMs == null ? "—" : rttMs.toFixed(0) + " ms";
        quickRttEl.textContent = dcRttEl.textContent;
        quickConnectionEl.textContent = iceState || "—";
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

/* ---- runtime controls ---------------------------------------------------- */

const PARAMETER_META = [
  {
    key: "confidence_threshold", group: "detection",
    zh: "检测置信度", en: "Detection confidence",
    helpZh: "提高会保留更可靠的检测框，但可能漏掉较弱目标。",
    helpEn: "Higher keeps more reliable boxes but can miss weaker objects.",
  },
  {
    key: "nms_threshold", group: "detection",
    zh: "NMS 阈值", en: "NMS threshold",
    helpZh: "提高会保留更多重叠检测框；降低会更强地合并重叠框。",
    helpEn: "Higher retains more overlapping boxes; lower suppresses them more strongly.",
  },
  {
    key: "track_activation_threshold", group: "tracking",
    zh: "轨迹激活阈值", en: "Track activation threshold",
    helpZh: "提高会减少新轨迹，通常更稳定；降低会更容易开始追踪。",
    helpEn: "Higher starts fewer, usually steadier tracks; lower starts tracks more easily.",
  },
  {
    key: "minimum_matching_threshold", group: "tracking",
    zh: "轨迹匹配阈值", en: "Track matching threshold",
    helpZh: "控制已有轨迹与新检测框的匹配门槛，观察 ID 漂移和断裂的变化。",
    helpEn: "Controls matching between tracks and detections; observe ID drift and fragmentation.",
  },
  {
    key: "lost_track_buffer", group: "tracking",
    zh: "丢失缓冲帧数", en: "Lost-track buffer",
    helpZh: "提高会在短暂遮挡后保留轨迹更久。",
    helpEn: "Higher retains tracks longer through short occlusions.",
    integer: true,
  },
  {
    key: "minimum_consecutive_frames", group: "tracking",
    zh: "连续命中帧数", en: "Consecutive hits",
    helpZh: "提高可减少偶发误检形成轨迹，但可能错过短暂目标。",
    helpEn: "Higher reduces accidental tracks but can miss short-lived objects.",
    integer: true,
  },
];

let runtimeSettings = null;
let settingsBusy = false;
const settingsDragging = new Set();
let toastTimer = null;

function formatParameter(meta, value) {
  return meta.integer ? String(Math.round(Number(value))) : Number(value).toFixed(2);
}

function showControlToast(message, kind = "ok") {
  if (!controlToast) return;
  controlToast.textContent = message;
  controlToast.className = "control-toast " + kind;
  controlToast.hidden = false;
  if (toastTimer) clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { controlToast.hidden = true; }, 3600);
}

function renderUndistortState() {
  if (!runtimeSettings || !undistortToggle) return;
  const enabled = !!runtimeSettings.values.undistort_enabled;
  const available = !!runtimeSettings.undistort_available;
  const videoActive = !!(inputState && inputState.source === "video");
  undistortToggle.checked = enabled;
  undistortToggle.disabled = settingsBusy || !available || videoActive;
  if (videoActive) {
    undistortStateEl.className = "control-state muted";
    undistortStateEl.textContent = tr("仅实时相机", "Camera only");
    undistortHintEl.textContent = tr(
      "本地视频不会应用鱼眼标定", "Uploaded video does not use camera calibration");
    return;
  }
  undistortStateEl.className = "control-state " +
    (enabled ? "ok" : available ? "muted" : "warn");
  if (enabled) {
    undistortStateEl.textContent = tr("已去畸变", "Corrected");
    undistortHintEl.textContent = tr(
      "检测、追踪与预览正在使用矫正画面", "Detection, tracking and preview use corrected frames");
  } else if (!available) {
    undistortStateEl.textContent = tr("标定不可用", "Calibration unavailable");
    undistortHintEl.textContent = runtimeSettings.undistort_error || tr(
      "当前继续使用原始画面", "The raw image remains active");
  } else {
    undistortStateEl.textContent = tr("原始画面", "Raw input");
    undistortHintEl.textContent = tr(
      "检测、追踪与预览使用同一输入", "Detection, tracking and preview share this input");
  }
}

function parameterGroupTitle(group) {
  if (group === "detection") {
    return { title: tr("目标检测", "Detection"), hint: tr("YOLO 后处理阈值，不会重载 TensorRT 引擎。", "YOLO postprocessing only; TensorRT remains loaded.") };
  }
  return { title: tr("多目标追踪", "Multi-object tracking"), hint: tr("修改后立即重置当前轨迹 ID。", "Changes reset current track IDs immediately.") };
}

function setControlsDisabled(disabled) {
  if (undistortToggle) undistortToggle.disabled = disabled ||
    !(runtimeSettings && runtimeSettings.undistort_available) ||
    !!(inputState && inputState.source === "video");
  if (parameterReset) parameterReset.disabled = disabled;
  for (const input of parameterControls.querySelectorAll("input")) input.disabled = disabled;
}

function syncParameterControls() {
  if (!runtimeSettings) return;
  renderUndistortState();
  for (const meta of PARAMETER_META) {
    if (settingsDragging.has(meta.key)) continue;
    const value = runtimeSettings.values[meta.key];
    const range = parameterControls.querySelector(`.parameter-range[data-key="${meta.key}"]`);
    const number = parameterControls.querySelector(`.parameter-value[data-key="${meta.key}"]`);
    if (range) range.value = String(value);
    if (number) number.value = formatParameter(meta, value);
  }
  setControlsDisabled(settingsBusy);
}

function renderParameterControls() {
  if (!runtimeSettings || !parameterControls) return;
  parameterControls.innerHTML = "";
  for (const group of ["detection", "tracking"]) {
    const groupEl = document.createElement("section");
    groupEl.className = "parameter-group";
    const copy = parameterGroupTitle(group);
    groupEl.innerHTML = `<h3>${copy.title}</h3><p>${copy.hint}</p>`;
    for (const meta of PARAMETER_META.filter((item) => item.group === group)) {
      const limit = runtimeSettings.limits[meta.key];
      const row = document.createElement("div");
      row.className = "parameter-row";
      const id = "param_" + meta.key;
      row.innerHTML =
        `<label for="${id}"><span>${lang === "zh" ? meta.zh : meta.en}</span>` +
        `<input class="parameter-value" data-key="${meta.key}" type="number" ` +
        `min="${limit.min}" max="${limit.max}" step="${limit.step}" /></label>` +
        `<p>${lang === "zh" ? meta.helpZh : meta.helpEn}</p>` +
        `<input class="parameter-range" data-key="${meta.key}" id="${id}" type="range" ` +
        `min="${limit.min}" max="${limit.max}" step="${limit.step}" />`;
      const range = row.querySelector(".parameter-range");
      const number = row.querySelector(".parameter-value");
      const commit = () => {
        const raw = Number(number.value);
        const value = Math.min(limit.max, Math.max(limit.min, raw));
        const normalized = meta.integer ? Math.round(value) : Number(value.toFixed(2));
        range.value = String(normalized);
        number.value = formatParameter(meta, normalized);
        settingsDragging.delete(meta.key);
        applyRuntimeSettings({ [meta.key]: normalized });
      };
      range.addEventListener("pointerdown", () => settingsDragging.add(meta.key));
      range.addEventListener("input", () => { number.value = formatParameter(meta, range.value); });
      range.addEventListener("change", commit);
      number.addEventListener("change", commit);
      number.addEventListener("focus", () => settingsDragging.add(meta.key));
      number.addEventListener("blur", () => settingsDragging.delete(meta.key));
      groupEl.appendChild(row);
    }
    parameterControls.appendChild(groupEl);
  }
  syncParameterControls();
}

async function refreshSettings() {
  try {
    const response = await fetch("/api/settings", { cache: "no-store" });
    if (!response.ok) throw new Error("settings unavailable");
    runtimeSettings = await response.json();
    renderUndistortState();
    if (!parameterControls.children.length) renderParameterControls();
    else syncParameterControls();
  } catch (_) {
    if (undistortStateEl) {
      undistortStateEl.className = "control-state warn";
      undistortStateEl.textContent = tr("控制离线", "Controls offline");
    }
  }
}

async function applyRuntimeSettings(patch) {
  if (settingsBusy) return;
  settingsBusy = true;
  setControlsDisabled(true);
  try {
    const response = await fetch("/api/settings", {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ values: patch }),
    });
    const outcome = await response.json();
    if (!response.ok || !outcome.ok) throw new Error(outcome.error || tr("设置未应用", "Settings were not applied"));
    if (runtimeSettings) runtimeSettings.values = outcome.values;
    const note = outcome.tracker_reset
      ? tr("参数已应用，当前轨迹 ID 已重置。", "Applied; current track IDs were reset.")
      : tr("参数已应用。", "Settings applied.");
    showControlToast(note, "ok");
  } catch (error) {
    showControlToast(error.message || tr("设置应用失败。", "Could not apply settings."), "err");
  } finally {
    settingsBusy = false;
    await refreshSettings();
  }
}

async function resetRuntimeSettings() {
  if (settingsBusy) return;
  settingsBusy = true;
  setControlsDisabled(true);
  try {
    const response = await fetch("/api/settings/reset", { method: "POST" });
    const outcome = await response.json();
    if (!response.ok || !outcome.ok) throw new Error(outcome.error || tr("恢复默认值失败", "Could not restore defaults"));
    if (runtimeSettings) runtimeSettings.values = outcome.values;
    showControlToast(tr("已恢复课程默认值，轨迹 ID 已重置。", "Course defaults restored; track IDs reset."), "ok");
  } catch (error) {
    showControlToast(error.message || tr("恢复默认值失败", "Could not restore defaults"), "err");
  } finally {
    settingsBusy = false;
    await refreshSettings();
  }
}

/* ---- shared camera / uploaded video input ------------------------------- */

async function refreshInputState() {
  try {
    const response = await fetch("/api/input", { cache: "no-store" });
    if (!response.ok) throw new Error("input state unavailable");
    inputState = await response.json();
    renderInputState();
  } catch (_) {
    videoFileSummary.textContent = tr("输入控制离线", "Input control offline");
  }
}

async function selectInputSource(source) {
  if (inputBusy || (inputState && inputState.source === source)) return;
  inputBusy = true;
  renderInputState();
  try {
    const response = await fetch("/api/input", {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ source }),
    });
    const outcome = await response.json();
    if (!response.ok || !outcome.ok) throw new Error(outcome.error || tr("切换失败", "Switch failed"));
    inputState = outcome;
    showControlToast(source === "video"
      ? tr("已切换到循环视频输入。", "Looping video input selected.")
      : tr("已切换到实时相机。", "Live camera selected."), "ok");
  } catch (error) {
    showControlToast(error.message || tr("输入切换失败", "Input switch failed"), "err");
  } finally {
    inputBusy = false;
    await refreshInputState();
  }
}

function uploadVideo(file) {
  if (!file || inputBusy) return;
  if (file.size > 4 * 1024 * 1024 * 1024) {
    showControlToast(tr("视频不能超过 4 GB。", "Video must not exceed 4 GB."), "err");
    return;
  }
  inputBusy = true;
  uploadProgress.hidden = false;
  uploadProgressBar.style.transform = "scaleX(0)";
  renderInputState();
  const data = new FormData();
  data.append("file", file, file.name);
  const xhr = new XMLHttpRequest();
  xhr.open("POST", "/api/input/video");
  xhr.upload.onprogress = (event) => {
    if (!event.lengthComputable) return;
    uploadProgressBar.style.transform = `scaleX(${Math.min(1, event.loaded / event.total)})`;
    videoFileSummary.textContent = `${tr("正在上传", "Uploading")} ${Math.round(event.loaded / event.total * 100)}%`;
  };
  xhr.onload = async () => {
    try {
      const outcome = JSON.parse(xhr.responseText || "{}");
      if (xhr.status < 200 || xhr.status >= 300 || !outcome.ok) {
        throw new Error(outcome.error || tr("上传失败", "Upload failed"));
      }
      inputState = outcome;
      const started = outcome.source === "video";
      showControlToast(
        started
          ? tr("视频已上传并开始循环推理。", "Video uploaded and looping inference started.")
          : (outcome.error || tr("视频已上传，但未能切到视频输入。", "Video uploaded, but video input did not start.")),
        started ? "ok" : "err"
      );
    } catch (error) {
      showControlToast(error.message || tr("上传失败", "Upload failed"), "err");
    } finally {
      inputBusy = false;
      uploadProgress.hidden = true;
      videoFileInput.value = "";
      await refreshInputState();
    }
  };
  xhr.onerror = async () => {
    inputBusy = false;
    uploadProgress.hidden = true;
    videoFileInput.value = "";
    showControlToast(tr("上传连接中断，原视频保持不变。", "Upload interrupted; the previous video was kept."), "err");
    await refreshInputState();
  };
  xhr.send(data);
}

async function deleteUploadedVideo() {
  if (inputBusy || !inputState || !inputState.video) return;
  inputBusy = true;
  renderInputState();
  try {
    const response = await fetch("/api/input/video", { method: "DELETE" });
    const outcome = await response.json();
    if (!response.ok || !outcome.ok) throw new Error(outcome.error || tr("删除失败", "Delete failed"));
    inputState = outcome;
    showControlToast(tr("视频已删除，输入已切回相机。", "Video deleted; live camera restored."), "ok");
  } catch (error) {
    showControlToast(error.message || tr("删除失败", "Delete failed"), "err");
  } finally {
    inputBusy = false;
    await refreshInputState();
  }
}

async function refreshVisualization() {
  try {
    const response = await fetch("/api/visualization/m4_3", { cache: "no-store" });
    if (!response.ok) return;
    visualizationState = await response.json();
    renderVisualization();
  } catch (_) { /* M4.3 controls are optional on a standalone legacy server */ }
}

async function refreshPose() {
  try {
    const response = await fetch("/api/visualization/m4_4", { cache: "no-store" });
    if (!response.ok) return;
    const data = await response.json();
    poseState = { results: data.results || null };
    renderPosePanel();
  } catch (_) { /* the pose endpoint is optional on older servers */ }
}

async function setVisualizationView(view) {
  if (visualizationBusy || visualizationState.view === view) return;
  visualizationBusy = true;
  renderVisualization();
  try {
    const response = await fetch("/api/visualization/m4_3", {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ view }),
    });
    const outcome = await response.json();
    if (!response.ok || !outcome.ok) throw new Error(outcome.error || tr("视图切换失败", "View switch failed"));
    visualizationState.view = outcome.view;
  } catch (error) {
    showControlToast(error.message || tr("视图切换失败", "View switch failed"), "err");
  } finally {
    visualizationBusy = false;
    await refreshVisualization();
  }
}

/* ---- drawer + settings --------------------------------------------------- */

function setDrawerOpen(open) {
  drawerEl.classList.toggle("open", open);
  drawerToggle.setAttribute("aria-pressed", open ? "true" : "false");
}

function toggleDrawer() {
  setDrawerOpen(!drawerEl.classList.contains("open"));
}

function setParameterDrawerOpen(open) {
  parameterDrawer.classList.toggle("open", open);
  parameterBtn.setAttribute("aria-pressed", open ? "true" : "false");
  if (open) {
    setDrawerOpen(false);
    closeSettings();
    refreshSettings();
  }
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
  else videoWrap.requestFullscreen?.();
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
parameterBtn.addEventListener("click", () => setParameterDrawerOpen(
  !parameterDrawer.classList.contains("open")));
parameterClose.addEventListener("click", () => setParameterDrawerOpen(false));
parameterReset.addEventListener("click", resetRuntimeSettings);
undistortToggle.addEventListener("change", () => {
  applyRuntimeSettings({ undistort_enabled: undistortToggle.checked });
});
cameraSourceBtn.addEventListener("click", () => selectInputSource("camera"));
videoSourceBtn.addEventListener("click", () => selectInputSource("video"));
uploadVideoBtn.addEventListener("click", () => videoFileInput.click());
videoFileInput.addEventListener("change", () => uploadVideo(videoFileInput.files[0]));
deleteVideoBtn.addEventListener("click", deleteUploadedVideo);
for (const button of segmentationResultsEl.querySelectorAll("[data-view]")) {
  button.addEventListener("click", () => setVisualizationView(button.dataset.view));
}
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
  setParameterDrawerOpen(false);
});

window.addEventListener("pagehide", () => {
  reconnectSuppressed = true;
  if (ws && ws.readyState === WebSocket.OPEN) {
    try { ws.send(JSON.stringify({ type: "bye" })); } catch (_) { /* leaving */ }
  }
  if (pc) {
    try { pc.close(); } catch (_) { /* leaving */ }
  }
});

/* ---- boot ---------------------------------------------------------------- */

applyLang();
mergeModules([], null);
// In Hub mode data-demo is the requested route (for example /m4/3), not the
// server-wide default.  Keep it until the existing websocket can select it.
selectedKey = DEMO_KEY;
streamKey = DEMO_KEY;
renderAll();
metricsLoop();
// The first offer must already prefer the server's codec, so wait for
// /api/demos (which reports it) before negotiating.
refreshModules().finally(() => {
  if (transport !== "mjpeg") connect();
});
setInterval(refreshModules, 2000);
refreshSettings();
setInterval(refreshSettings, 2000);
refreshInputState();
refreshVisualization();
refreshPose();
setInterval(refreshInputState, 2000);
setInterval(refreshVisualization, 2000);
setInterval(refreshPose, 2000);
