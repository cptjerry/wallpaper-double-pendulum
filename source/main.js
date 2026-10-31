import './we_property_bridge.js';
import {
  canvas, gl, statusEl, hudEl, state, camera, ENABLE_POTENTIAL_SHELL,
  DEFAULT_WEB_CONFIG, CAMERA_PITCH_LIMIT, CAMERA_DRAG_SENSITIVITY, CAMERA_RELEASE_SPEED_LIMIT, PERSIST_INTERVAL_MS,
  trailClear, cameraRotateOrbit, cameraSetPitch, quatNormalize, clamp,
  poseFromConfigState, clearPersistedSimState, estimateCurrentSpeedsFromPoseDelta,
  estimateTraceSpeedScales, syncVisualStateFromConfig, updateViewAndTraceStatsFromSummary,
  makeFallbackSummary, resize, poseFromFrame, parseWallpaperColorValue, estimatePotentialEnvelopeSummary,
  applySpeedValue, cloneConfig
} from './runtime_core.js';
import { createSceneRenderer, pushTrail } from './scene_renderer.js';
import { WorkerStream } from './worker_stream.js';
import { maybePersistSimulationState, loadAndApplyConfigWithStateRestore } from './persist_state.js';
import { updateHudOverlay } from './hud_overlay.js';
import { createThemeState, applyThemeState, updateThemeStateFromProperties } from './theme_state.js';

const sceneRenderer = gl ? createSceneRenderer(gl, canvas) : null;
const worker = new WorkerStream({ state, onStatusNote: reportWorkerIssue });

let latestFrame = null;
let staticPose = null;
let playbackTime = 0.0;
let lastFrameTime = performance.now();
let simTime = 0;
let lastPoseForSpeed = null;
let lastSpeedSimTime = null;
let stepWarningText = '';
let latestDiag = null;
let latestPersistSnapshot = null;
let appliedConfig = cloneConfig(DEFAULT_WEB_CONFIG);
const trail1 = { items: [], start: 0, version: 0 };
const trail2 = { items: [], start: 0, version: 0 };
let wallpaperFpsLimit = 0;
let lastRenderSubmitMs = 0;
let lastPersistSaveMs = 0;
let fpsFrameCount = 0;
let fpsSampleStartMs = performance.now();
let hudRefreshPending = true;

function requestHudRefresh() {
  hudRefreshPending = true;
}

const CONTROL_MODE = {
  MOUSE: 0,
  GAMEPAD: 1,
  NONE: 2,
};
const HUD_POSITION = {
  NONE: 'none',
  TOP_LEFT: 'top_left',
  TOP_RIGHT: 'top_right',
  BOTTOM_LEFT: 'bottom_left',
  BOTTOM_RIGHT: 'bottom_right',
};
const GAMEPAD_DEADZONE = 0.16;
const GAMEPAD_YAW_ACCEL = 1.8;
const GAMEPAD_PITCH_SPEED = 1.15;

function isHudVisible() {
  return state.hudPosition !== HUD_POSITION.NONE;
}

function updateHudLayout() {
  if (!hudEl) return;
  const visible = isHudVisible();
  hudEl.classList.toggle('hidden', !visible);
  if (!visible) return;
  const vw = Math.max(window.innerWidth || 1, 1);
  const vh = Math.max(window.innerHeight || 1, 1);
  const base = Math.min(vw, vh);
  const margin = Math.round(base * 0.018);
  const scale = Math.max(Number(state.hudScale) || 1, 0.5);
  const fontSize = Math.max(11, Math.min(28, base * 0.0135 * scale));
  const paddingY = Math.round(fontSize * 0.8);
  const paddingX = Math.round(fontSize * 0.95);
  hudEl.style.fontSize = `${fontSize}px`;
  hudEl.style.lineHeight = `${(fontSize * 1.42).toFixed(1)}px`;
  hudEl.style.padding = `${paddingY}px ${paddingX}px`;
  hudEl.style.width = 'max-content';
  hudEl.style.maxWidth = `calc(100vw - ${margin * 2}px)`;
  hudEl.style.left = '';
  hudEl.style.right = '';
  hudEl.style.top = '';
  hudEl.style.bottom = '';
  switch (state.hudPosition) {
    case HUD_POSITION.TOP_RIGHT:
      hudEl.style.right = `${margin}px`;
      hudEl.style.top = `${margin}px`;
      break;
    case HUD_POSITION.BOTTOM_LEFT:
      hudEl.style.left = `${margin}px`;
      hudEl.style.bottom = `${margin}px`;
      break;
    case HUD_POSITION.BOTTOM_RIGHT:
      hudEl.style.right = `${margin}px`;
      hudEl.style.bottom = `${margin}px`;
      break;
    default:
      hudEl.style.left = `${margin}px`;
      hudEl.style.top = `${margin}px`;
      break;
  }
}

function setHudPositionFromValue(value) {
  const map = {
    0: HUD_POSITION.NONE,
    1: HUD_POSITION.TOP_LEFT,
    2: HUD_POSITION.TOP_RIGHT,
    3: HUD_POSITION.BOTTOM_LEFT,
    4: HUD_POSITION.BOTTOM_RIGHT,
    none: HUD_POSITION.NONE,
    top_left: HUD_POSITION.TOP_LEFT,
    top_right: HUD_POSITION.TOP_RIGHT,
    bottom_left: HUD_POSITION.BOTTOM_LEFT,
    bottom_right: HUD_POSITION.BOTTOM_RIGHT,
  };
  const next = map[value] ?? map[String(value)] ?? HUD_POSITION.TOP_LEFT;
  state.hudPosition = next;
  state.showHud = next !== HUD_POSITION.NONE;
  updateHudLayout();
  if (!state.showHud && statusEl) statusEl.textContent = '';
  requestHudRefresh();
}

function normalizeControlModeValue(value) {
  const n = Number(value);
  if (!Number.isFinite(n)) return CONTROL_MODE.MOUSE;
  switch (n) {
    case 4:
      return CONTROL_MODE.NONE;
    case 2:
    case 3:
      return CONTROL_MODE.GAMEPAD;
    case 1:
      return CONTROL_MODE.MOUSE;
    case 0:
      return CONTROL_MODE.MOUSE;
    default:
      return n;
  }
}

function isMouseControlMode() {
  return state.controlMode === CONTROL_MODE.MOUSE;
}

function normalizeAxis(v) {
  const n = Number(v) || 0;
  return Math.abs(n) >= GAMEPAD_DEADZONE ? n : 0;
}

function getActiveGamepadAxes() {
  if (!navigator.getGamepads) return null;
  const pads = navigator.getGamepads();
  const pad = pads && Array.from(pads).find((g) => g && g.connected);
  if (!pad || state.controlMode !== CONTROL_MODE.GAMEPAD) return null;
  const axes = Array.isArray(pad.axes) ? pad.axes : [];
  const left = [normalizeAxis(axes[0]), normalizeAxis(axes[1])];
  const right = [normalizeAxis(axes[2]), normalizeAxis(axes[3])];
  const leftMag = Math.abs(left[0]) + Math.abs(left[1]);
  const rightMag = Math.abs(right[0]) + Math.abs(right[1]);
  if (rightMag > leftMag) return right;
  if (leftMag > 0) return left;
  if (rightMag > 0) return right;
  return [0, 0];
}

function adjustAutoRotateSpeed(delta) {
  const next = clamp((Number(state.autoRotateZ) || 0) + delta, -CAMERA_RELEASE_SPEED_LIMIT, CAMERA_RELEASE_SPEED_LIMIT);
  state.autoRotateZ = next;
  camera.releaseYawSpeed = next;
}

function applyControlState(dt) {
  if (state.controlMode === CONTROL_MODE.NONE) {
    const targetPitch = clamp(Number(state.controlPitch) || 0, -CAMERA_PITCH_LIMIT, CAMERA_PITCH_LIMIT);
    cameraSetPitch(targetPitch);
    state.autoRotateZ = clamp(Number(state.controlRotateSpeed) || 0, -CAMERA_RELEASE_SPEED_LIMIT, CAMERA_RELEASE_SPEED_LIMIT);
    camera.releaseYawSpeed = state.autoRotateZ;
    return;
  }
  if (state.controlMode === CONTROL_MODE.GAMEPAD) {
    const axes = getActiveGamepadAxes();
    if (axes) {
      const [ax, ay] = axes;
      if (ax) {
        state.autoRotateZ = clamp((Number(state.autoRotateZ) || 0) + ax * GAMEPAD_YAW_ACCEL * dt, -CAMERA_RELEASE_SPEED_LIMIT, CAMERA_RELEASE_SPEED_LIMIT);
        camera.releaseYawSpeed = state.autoRotateZ;
      }
      if (ay) {
        cameraRotateOrbit(0.0, -ay * GAMEPAD_PITCH_SPEED * dt);
      }
    }
  }
}

// Same status/HUD forwarding as fix11h visible_errors. No separate panel.
function reportWorkerIssue(message) {
  stepWarningText = message ? String(message) : '';
  if (statusEl) statusEl.textContent = stepWarningText;
  requestHudRefresh();
}

function noteMessage(_) {}

function setBootVisibility(ready, failed = false) {
  const body = document.body;
  if (!body) return;
  body.classList.toggle('boot-hidden', !ready);
  body.classList.toggle('boot-failed', !!failed);
  if (hudEl && isHudVisible()) hudEl.classList.toggle('hidden', !ready);
}


function restoreCameraFromSavedState(savedState) {
  const savedCamera = savedState?.camera;
  if (!savedCamera) return;
  if (Array.isArray(savedCamera.orbitQuat) && savedCamera.orbitQuat.length === 4) {
    camera.orbitQuat = quatNormalize(savedCamera.orbitQuat);
  }
  if (Array.isArray(savedCamera.target) && savedCamera.target.length === 3) {
    camera.target = savedCamera.target.slice();
  }
  if (Number.isFinite(savedCamera.distance) && savedCamera.distance > 0) {
    camera.distance = savedCamera.distance;
  }
  if (Number.isFinite(savedCamera.autoRotateZ)) {
    state.autoRotateZ = clamp(savedCamera.autoRotateZ, -CAMERA_RELEASE_SPEED_LIMIT, CAMERA_RELEASE_SPEED_LIMIT);
  }
  camera.releaseYawSpeed = state.autoRotateZ || 0.0;
}

function syncVisualStateFromConfigState(cfg) {
  syncVisualStateFromConfig(cfg.params, cfg.rods, cfg.balls);
}

function applyVisualSummaryForConfig(cfg) {
  const summary = ENABLE_POTENTIAL_SHELL
    ? estimatePotentialEnvelopeSummary(cfg)
    : makeFallbackSummary(cfg);
  state.latestPotentialSummary = summary;
  state.latestShellCfg = cfg;
  updateViewAndTraceStatsFromSummary(summary);
  if (sceneRenderer?.setShellConfig) sceneRenderer.setShellConfig(cfg, summary);
}

function resetSimulationTrails() {
  trailClear(trail1);
  trailClear(trail2);
  lastPoseForSpeed = null;
  lastSpeedSimTime = null;
}

function clearDisplayedTrajectory() {
  resetSimulationTrails();
  latestDiag = null;
  latestPersistSnapshot = null;
  stepWarningText = '';
  requestHudRefresh();
  worker.queueClear();
  latestFrame = null;
  playbackTime = 0;
  simTime = 0;
}

function handleWorkerFrame(frame) {
  latestFrame = frame;
  const rawT = Number(frame.t) || 0;
  simTime = state.simTimeOffset + rawT;
  const tol = Math.max(Number(state.activeTol) || 0, 0);
  if (Number.isFinite(frame.residual) && tol > 0 && frame.residual > tol) state.overThresholdCount += 1;

  const pose = poseFromFrame(frame);
  let s1 = Number.isFinite(frame.v1) ? frame.v1 : 0.0;
  let s2 = Number.isFinite(frame.v2) ? frame.v2 : 0.0;
  if ((!Number.isFinite(frame.v1) || !Number.isFinite(frame.v2)) && lastPoseForSpeed && lastSpeedSimTime != null) {
    const dtSpeed = Math.max(simTime - lastSpeedSimTime, state.simDt, 1e-6);
    const speedParams = (appliedConfig && Array.isArray(appliedConfig.params)) ? appliedConfig.params : DEFAULT_WEB_CONFIG.params;
    const est = estimateCurrentSpeedsFromPoseDelta(lastPoseForSpeed, pose, dtSpeed, speedParams);
    s1 = est.s1;
    s2 = est.s2;
  }

  pushTrail(trail1, pose.joint, s1, simTime);
  pushTrail(trail2, pose.tip, s2, simTime);
  lastPoseForSpeed = {
    n1: [...pose.n1], u1: [...pose.u1], v1: [...pose.v1],
    n2: [...pose.n2], u2: [...pose.u2], v2: [...pose.v2],
  };
  lastSpeedSimTime = simTime;
  if (frame.persistSnapshot) latestPersistSnapshot = frame.persistSnapshot;
  if (frame.diag) {
    latestDiag = frame.diag;
    requestHudRefresh();
  }
  stepWarningText = frame.libRc === 0
    ? `step warning rc=${frame.rc}, lib_rc=${frame.libRc}, res=${Number(frame.residual).toExponential(2)}, it=${frame.niters}`
    : '';
}

async function applyInitFromConfig(cfg, successMessage = 'Applied wallpaper configuration.', savedState = null) {
  const prevPlaying = state.playing;
  state.initializing = true;
  state.playing = false;
  try {
    clearDisplayedTrajectory();
    staticPose = savedState ? poseFromConfigState({ ...cfg, yCur: savedState.q.slice() }) : poseFromConfigState(cfg);
    state.simDt = cfg.simDt;
    syncVisualStateFromConfigState(cfg)

    const speedEstimate = estimateTraceSpeedScales(cfg);
    state.traceSpeedMax1 = speedEstimate.maxSpeed1;
    state.traceSpeedMax2 = speedEstimate.maxSpeed2;
    applyVisualSummaryForConfig(cfg);

    if (!worker.hasClient() || !worker.isReady()) {
      latestFrame = null;
      latestPersistSnapshot = null;
      playbackTime = 0;
      simTime = savedState ? (Number(savedState.t) || 0) : state.simTimeOffset;
      state.activeTol = cfg.tol;
      state.overThresholdCount = 0;
      appliedConfig = cloneConfig(cfg);
      noteMessage(`${successMessage} Worker unavailable, static preview only.`);
      state.playing = false;
      requestHudRefresh();
      return;
    }

    const data = await worker.applyInit(cfg, savedState, state.l1Vis, state.l2Vis);
    if (!data || data.ok !== true) throw new Error(data && data.message ? data.message : 'apply init failed');

    resetSimulationTrails();
    latestDiag = null;
    latestPersistSnapshot = null;
    worker.queueClear();
    playbackTime = data.simTime || 0;
    simTime = playbackTime;

    if (data.initialFrame) {
      latestFrame = data.initialFrame;
      if (data.initialFrame.persistSnapshot) latestPersistSnapshot = data.initialFrame.persistSnapshot;
      const pose = poseFromFrame(data.initialFrame);
      lastPoseForSpeed = {
        n1: [...pose.n1], u1: [...pose.u1], v1: [...pose.v1],
        n2: [...pose.n2], u2: [...pose.u2], v2: [...pose.v2],
      };
      lastSpeedSimTime = simTime;
    }

    worker.queuePushMany(data.frames);
    state.workerGeneration = Number.isFinite(data.generation) ? data.generation : state.workerGeneration;
    state.activeTol = cfg.tol;
    state.overThresholdCount = 0;
    appliedConfig = cloneConfig(cfg);
    worker.requestFillIfNeeded();
    stepWarningText = '';
    state.playing = prevPlaying;
  } catch (err) {
    clearDisplayedTrajectory();
    state.overThresholdCount = 0;
    noteMessage(`Apply failed: ${err && err.message ? err.message : err}`);
    state.playing = prevPlaying;
  } finally {
    state.initializing = false;
    requestHudRefresh();
  }
}

async function loadAndApplyWallpaperConfig(reason = 'reload') {
  const result = await loadAndApplyConfigWithStateRestore({
    state,
    applyInitFromConfig,
    reason,
    trail1,
    trail2,
    onConfigLoaded: (cfg, signature) => {
      appliedConfig = cloneConfig(cfg);
      state.configSignature = signature;
    },
  });
  staticPose = result.saved ? poseFromConfigState({ ...result.cfg, yCur: result.saved.q.slice() }) : poseFromConfigState(result.cfg);
  if (result.saved) restoreCameraFromSavedState(result.saved);
  return result.cfg;
}

function installRuntimeDefaults() {
  syncVisualStateFromConfigState(DEFAULT_WEB_CONFIG);
  staticPose = poseFromConfigState(DEFAULT_WEB_CONFIG);
  appliedConfig = cloneConfig(DEFAULT_WEB_CONFIG);
  state.activeTol = DEFAULT_WEB_CONFIG.tol;
  state.showPotentialShell = false;
  state.showHud = true;
  state.persistStateEnabled = false;
  state.themeAuto = false;
  state.viewDistanceScale = 1.0;
  state.screenOffsetX = 0.0;
  state.screenOffsetY = 0.0;
  state.controlMode = CONTROL_MODE.MOUSE;
  state.controlPitch = 0.24;
  state.controlRotateSpeed = 0.0;
  state.hudScale = 1.0;
  state.hudPosition = HUD_POSITION.TOP_LEFT;
  if (!state.themeState) state.themeState = createThemeState(state);
  applyThemeState(state);
  state.simTimeOffset = 0.0;
  camera.orbitQuat = [0, 0, 0, 1];
  cameraRotateOrbit(0.9, 0.24);
  updateViewAndTraceStatsFromSummary(makeFallbackSummary(DEFAULT_WEB_CONFIG));
  updateHudLayout();
  requestHudRefresh();
}


function updateFps(nowMs) {
  let sampled = false;
  fpsFrameCount += 1;
  const elapsed = nowMs - fpsSampleStartMs;
  if (elapsed >= 500) {
    const measured = (fpsFrameCount * 1000) / Math.max(elapsed, 1);
    state.actualFps = state.actualFps > 0 ? (0.65 * state.actualFps + 0.35 * measured) : measured;
    fpsFrameCount = 0;
    fpsSampleStartMs = nowMs;
    sampled = true;
  }
  const targetFps = wallpaperFpsLimit > 0 ? wallpaperFpsLimit : 0;
  if (state.targetFps !== targetFps) {
    state.targetFps = targetFps;
    sampled = true;
  }
  return sampled;
}

function render(nowMs) {
  const fpsLimit = wallpaperFpsLimit > 0 ? wallpaperFpsLimit : 0;
  if (fpsLimit > 0) {
    const minFrameMs = 1000 / fpsLimit;
    if (!lastRenderSubmitMs) {
      lastRenderSubmitMs = nowMs;
    } else {
      const elapsed = nowMs - lastRenderSubmitMs;
      if (elapsed + 0.25 < minFrameMs) {
        requestAnimationFrame(render);
        return;
      }
      const steps = Math.max(1, Math.floor((elapsed + 0.25) / minFrameMs));
      lastRenderSubmitMs += steps * minFrameMs;
      if (nowMs - lastRenderSubmitMs > minFrameMs * 2) lastRenderSubmitMs = nowMs;
    }
  } else {
    lastRenderSubmitMs = nowMs;
  }

  resize();
  updateHudLayout();
  const fpsHudSync = updateFps(nowMs);
  const dt = Math.min(0.05, (nowMs - lastFrameTime) * 0.001);
  lastFrameTime = nowMs;
  const renderDt = dt * state.renderSpeed;

  applyControlState(dt);
  if (!camera.dragging && Math.abs(state.autoRotateZ) > 1e-9) {
    cameraRotateOrbit(state.autoRotateZ * dt, 0.0);
  }
  if (state.playing && !state.initializing) playbackTime += renderDt;

  let pose = latestFrame ? poseFromFrame(latestFrame) : null;
  if (worker.isReady() && state.playing && !state.initializing) {
    let consumed = 0;
    while (worker.queueLength() && worker.queuePeek().t <= playbackTime + 1e-9) {
      const frame = worker.queueShift();
      consumed += 1;
      handleWorkerFrame(frame);
      pose = poseFromFrame(frame);
    }
    worker.reportDrain(consumed);
  }

  if (!pose) pose = staticPose || poseFromConfigState(appliedConfig);

  if (sceneRenderer) {
    try { sceneRenderer.renderScene(pose, simTime, { trail1, trail2 }); }
    catch (_) {}
  } else if (gl) {
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clearColor(state.backgroundColor[0] / 255, state.backgroundColor[1] / 255, state.backgroundColor[2] / 255, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT);
  }

  if (!lastPersistSaveMs || (nowMs - lastPersistSaveMs) >= PERSIST_INTERVAL_MS) {
    maybePersistSimulationState({ state, camera, latestFrame, latestDiag, latestPersistSnapshot, simTime, trail1, trail2 });
    lastPersistSaveMs = nowMs;
  }

  if (isHudVisible() && (hudRefreshPending || fpsHudSync)) {
    updateHudOverlay(statusEl, {
      state,
      workerReady: worker.isReady(),
      simTime,
      queueLength: worker.queueLength(),
      latestDiag,
      stepWarningText,
    });
    hudRefreshPending = false;
  }

  requestAnimationFrame(render);
}


canvas.addEventListener('pointerdown', (e) => {
  if (e.pointerType !== 'mouse') return;
  if (!isMouseControlMode()) return;
  if (e.button !== 0) return;
  camera.dragging = true;
  camera.pointerId = e.pointerId;
  camera.lastX = e.clientX;
  camera.lastY = e.clientY;
  camera.lastMoveTs = e.timeStamp || performance.now();
  camera.releaseYawSpeed = 0.0;
  state.autoRotateZ = 0.0;
  try { canvas.setPointerCapture(e.pointerId); } catch (_) {}
  e.preventDefault();
});

window.addEventListener('pointerup', (e) => {
  if (!camera.dragging) return;
  if (camera.pointerId != null && e.pointerId !== camera.pointerId) return;
  if (!isMouseControlMode()) { camera.dragging = false; camera.pointerId = null; return; }
  if (e.button !== 0) return;
  camera.dragging = false;
  camera.pointerId = null;
  state.autoRotateZ = clamp(camera.releaseYawSpeed || 0.0, -CAMERA_RELEASE_SPEED_LIMIT, CAMERA_RELEASE_SPEED_LIMIT);
});

window.addEventListener('pointercancel', (e) => {
  if (camera.pointerId != null && e.pointerId !== camera.pointerId) return;
  camera.dragging = false;
  camera.pointerId = null;
});

window.addEventListener('pointermove', (e) => {
  if (!camera.dragging) return;
  if (camera.pointerId != null && e.pointerId !== camera.pointerId) return;
  const dx = e.clientX - camera.lastX;
  const dy = e.clientY - camera.lastY;
  const nowTs = e.timeStamp || performance.now();
  const dtSec = Math.max((nowTs - camera.lastMoveTs) * 0.001, 1e-3);
  camera.lastX = e.clientX;
  camera.lastY = e.clientY;
  camera.lastMoveTs = nowTs;
  cameraRotateOrbit(-dx * CAMERA_DRAG_SENSITIVITY, -dy * CAMERA_DRAG_SENSITIVITY);
  camera.releaseYawSpeed = clamp((-dx * CAMERA_DRAG_SENSITIVITY) / dtSec, -CAMERA_RELEASE_SPEED_LIMIT, CAMERA_RELEASE_SPEED_LIMIT);
});

canvas.addEventListener('contextmenu', (e) => e.preventDefault());

function applyGeneralPropertiesImpl(properties) {
  if (!properties || typeof properties !== 'object') return;
  if (properties && properties.fps != null) {
    const fps = Number(properties.fps);
    wallpaperFpsLimit = Number.isFinite(fps) && fps > 0 ? fps : 0;
    state.targetFps = wallpaperFpsLimit;
    requestHudRefresh();
  }
}

function applyUserPropertiesImpl(properties) {
  if (!properties || typeof properties !== 'object') return;
  if (properties.playbackspeed) applySpeedValue(Number(properties.playbackspeed.value));
  if (properties.showshell) {
    state.showPotentialShell = !!properties.showshell.value;
    if (state.showPotentialShell && sceneRenderer?.setShellConfig && state.latestPotentialSummary) {
      sceneRenderer.setShellConfig(state.latestShellCfg || appliedConfig, state.latestPotentialSummary);
    }
  }
  if (properties.hudposition) setHudPositionFromValue(properties.hudposition.value);
  if (properties.hudscale) { state.hudScale = Math.max(Number(properties.hudscale.value) || 1, 0.5); updateHudLayout(); requestHudRefresh(); }
  updateThemeStateFromProperties(state, properties, parseWallpaperColorValue);
  state.themeAuto = !!(state.themeState && state.themeState.auto);
  if (properties.trailseconds) state.trailSeconds = Number(properties.trailseconds.value);
  if (properties.size) {
    const size = Math.max(Number(properties.size.value) || 1, 0.05);
    state.viewDistanceScale = 1 / size;
  }
  if (properties.screenoffsetx) state.screenOffsetX = -(Number(properties.screenoffsetx.value) || 0);
  if (properties.screenoffsety) state.screenOffsetY = -(Number(properties.screenoffsety.value) || 0);
  if (properties.controlmode) state.controlMode = normalizeControlModeValue(properties.controlmode.value);
  if (properties.controlpitch) state.controlPitch = clamp((Number(properties.controlpitch.value) || 0) * Math.PI / 180, -CAMERA_PITCH_LIMIT, CAMERA_PITCH_LIMIT);
  if (properties.rotationspeed) {
    state.controlRotateSpeed = clamp(Number(properties.rotationspeed.value) || 0, -CAMERA_RELEASE_SPEED_LIMIT, CAMERA_RELEASE_SPEED_LIMIT);
    if (!isMouseControlMode()) camera.dragging = false;
    if (state.controlMode === CONTROL_MODE.NONE) {
      state.autoRotateZ = state.controlRotateSpeed;
      camera.releaseYawSpeed = state.autoRotateZ;
    }
  }
  if (properties.persiststate) {
    const nextEnabled = !!properties.persiststate.value;
    const changed = nextEnabled !== state.persistStateEnabled;
    state.persistStateEnabled = nextEnabled;
    if (!nextEnabled) {
      state.simTimeOffset = 0.0;
      clearPersistedSimState();
      latestPersistSnapshot = null;
      requestHudRefresh();
    } else if (changed) {
      lastPersistSaveMs = 0;
    }
  }
  state.showBar = true;
  if (!isMouseControlMode()) camera.dragging = false;
  if (state.controlMode === CONTROL_MODE.NONE) {
    cameraSetPitch(state.controlPitch);
    state.autoRotateZ = state.controlRotateSpeed;
    camera.releaseYawSpeed = state.autoRotateZ;
  }
  updateHudLayout();
  requestHudRefresh();
}

(async function boot() {
  installRuntimeDefaults();
  window.__MCLD_WE_BRIDGE__.connect({
    applyGeneralProperties: applyGeneralPropertiesImpl,
    applyUserProperties: applyUserPropertiesImpl,
  });
  setBootVisibility(false, false);
  requestAnimationFrame(render);
  worker.start(state.simDt);
  state.useWasm = worker.hasClient();
  await worker.waitUntilReady().catch(() => false);
  try {
    await loadAndApplyWallpaperConfig('startup');
    setBootVisibility(true, false);
  } catch (err) {
    if (statusEl) statusEl.textContent = `Startup failed: ${err && err.message ? err.message : err}`;
    setBootVisibility(true, true);
  }
})();
