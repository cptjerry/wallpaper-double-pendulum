import { PERSIST_STORAGE_KEY, readPersistedSimState, clearPersistedSimState, fetchWallpaperConfigRecord } from './runtime_core.js';

const TRAIL1_SAVE_COUNT = 60;
const TRAIL2_SAVE_COUNT = 120;

function getActiveTrailPoints(trail) {
  if (!trail || !Array.isArray(trail.items)) return [];
  return trail.items.slice(trail.start || 0);
}

function trailPointMatches(a, b) {
  return !!a && !!b && a.t === b.t && a.speed === b.speed && a.pos[0] === b.pos[0] && a.pos[1] === b.pos[1] && a.pos[2] === b.pos[2];
}

function serializeTrailTail(trail, maxPoints, previous = []) {
  const tail = getActiveTrailPoints(trail).slice(-maxPoints);
  const output = new Array(tail.length);
  const offset = Math.max(0, previous.length - tail.length);
  for (let i = 0; i < tail.length; i += 1) {
    const point = tail[i];
    const prevPoint = previous[offset + i];
    output[i] = trailPointMatches(point, prevPoint)
      ? prevPoint
      : { pos: [point.pos[0], point.pos[1], point.pos[2]], speed: point.speed ?? 0, t: point.t ?? 0 };
  }
  return output;
}

function restoreTrailPoints(trail, savedPoints) {
  if (!trail) return;
  trail.items = Array.isArray(savedPoints)
    ? savedPoints.map((point) => ({ pos: [point.pos[0], point.pos[1], point.pos[2]], speed: point.speed, t: point.t }))
    : [];
  trail.start = 0;
}

let lastSerializedState = null;

function serializeCameraState(camera, state) {
  if (!camera || !Array.isArray(camera.orbitQuat) || camera.orbitQuat.length !== 4) return null;
  if (!Array.isArray(camera.target) || camera.target.length !== 3) return null;
  const orbitQuat = camera.orbitQuat.map(Number);
  const target = camera.target.map(Number);
  const distance = Number(camera.distance);
  const autoRotateZ = Number(state?.autoRotateZ);
  if (!orbitQuat.every(Number.isFinite) || !target.every(Number.isFinite) || !Number.isFinite(distance) || distance <= 0) return null;
  return {
    orbitQuat,
    target,
    distance,
    autoRotateZ: Number.isFinite(autoRotateZ) ? autoRotateZ : 0,
  };
}

function maybePersistSimulationState({ state, camera, latestFrame, latestDiag, latestPersistSnapshot, simTime, trail1, trail2 }) {
  if (!state.persistStateEnabled || state.initializing || !latestFrame || !state.configSignature) return;
  if (!Array.isArray(latestFrame.y) || latestFrame.y.length !== 8 || !Array.isArray(latestFrame.p) || latestFrame.p.length !== 6) return;
  const currentEnergy = Number(latestDiag?.H_cur);
  if (!Number.isFinite(currentEnergy)) return;
  const persistHex = latestPersistSnapshot
    && Array.isArray(latestPersistSnapshot.qHex) && latestPersistSnapshot.qHex.length === 8
    && Array.isArray(latestPersistSnapshot.pHex) && latestPersistSnapshot.pHex.length === 6
    && typeof latestPersistSnapshot.tHex === 'string'
    && typeof latestPersistSnapshot.HHex === 'string'
    ? latestPersistSnapshot
    : null;
  const nextState = {
    q: latestFrame.y.slice(),
    p: latestFrame.p.slice(),
    t: Number(simTime),
    H: currentEnergy,
    qHex: persistHex ? persistHex.qHex.slice() : null,
    pHex: persistHex ? persistHex.pHex.slice() : null,
    tHex: persistHex ? persistHex.tHex : null,
    HHex: persistHex ? persistHex.HHex : null,
    signature: state.configSignature,
    trail1: serializeTrailTail(trail1, TRAIL1_SAVE_COUNT, lastSerializedState?.trail1 || []),
    trail2: serializeTrailTail(trail2, TRAIL2_SAVE_COUNT, lastSerializedState?.trail2 || []),
    camera: serializeCameraState(camera, state),
    savedAt: Date.now(),
  };
  lastSerializedState = nextState;
  try {
    localStorage.setItem(PERSIST_STORAGE_KEY, JSON.stringify(nextState));
  } catch (error) {
    console.warn('[persist] failed to save current state', error);
  }
}

async function loadAndApplyConfigWithStateRestore({ state, applyInitFromConfig, reason = 'reload', trail1, trail2, onConfigLoaded }) {
  const record = await fetchWallpaperConfigRecord();
  const config = record.cfg;
  const signature = record.signature;
  const savedState = state.persistStateEnabled ? readPersistedSimState() : null;
  const canRestore = !!(savedState && savedState.signature && savedState.signature === signature);
  state.configSignature = signature;
  state.simTimeOffset = 0.0;
  if (!canRestore && savedState && savedState.signature && savedState.signature !== signature) clearPersistedSimState();
  if (typeof onConfigLoaded === 'function') onConfigLoaded(config, signature);
  await applyInitFromConfig(config, canRestore ? `Restored saved state (${reason}).` : `Loaded wallpaper_config.json (${reason}).`, canRestore ? savedState : null);
  if (canRestore) {
    restoreTrailPoints(trail1, savedState.trail1 || []);
    restoreTrailPoints(trail2, savedState.trail2 || []);
  }
  return { cfg: config, saved: canRestore ? savedState : null, signature };
}

export { maybePersistSimulationState, loadAndApplyConfigWithStateRestore };
