let moduleInstance = null;
let api = null;
let booting = null;
let generation = 0;
let queueCount = 0;
let filling = false;
let currentCfg = null;
let currentRenderLengths = { l1Vis: 0.5, l2Vis: 1.0 };
let frameCounter = 0;
let prevFrameZ = null;

const DEFAULT_BUFFER_CONFIG = Object.freeze({ lowWater: 96, highWater: 192, chunkSize: 12 });
let bufferConfig = { ...DEFAULT_BUFFER_CONFIG };
const SOLVER_MODE_IDS = { lagrange: 0, hamilton_left: 1, hamilton_right: 2 };
const INIT_MODE_IDS = { pair: 0, left: 1, right: 2 };
const CWRAP_SPECS = {
  initDefault: ['mcld_wallpaper_init_default', 'number', ['number']],
  configure: ['mcld_wallpaper_configure', null, ['number', 'number', 'number', 'number', 'number', 'number']],
  configSetReal: ['mcld_wallpaper_config_set_real', null, ['number', 'number', 'number']],
  configSetInt: ['mcld_wallpaper_config_set_int', null, ['number', 'number']],
  paramSet: ['mcld_wallpaper_param_set', null, ['number', 'number']],
  applyInit: ['mcld_wallpaper_apply_init', 'number', []],
  restoreMeta: ['mcld_wallpaper_restore_meta', null, ['number', 'number']],
  restoreSetHex: ['mcld_wallpaper_restore_set_hex', 'number', ['number', 'number', 'string']],
  restoreMetaHex: ['mcld_wallpaper_restore_meta_hex', 'number', ['string', 'string']],
  persistGetHex: ['mcld_wallpaper_persist_get_hex', 'string', ['number', 'number']],
  persistRealBytes: ['mcld_wallpaper_real_byte_size', 'number', []],
  applyRestore: ['mcld_wallpaper_apply_restore', 'number', []],
  stepN: ['mcld_wallpaper_step_n', 'number', ['number']],
  setRenderLengths: ['mcld_wallpaper_set_visual_lengths', null, ['number', 'number']],
  geometryAt: ['mcld_wallpaper_geometry_at', 'number', ['number']],
  getTime: ['mcld_wallpaper_get_time', 'number', []],
  getLastLibRc: ['mcld_wallpaper_get_last_lib_rc', 'number', []],
  getLastResidual: ['mcld_wallpaper_get_last_residual', 'number', []],
  getLastNiters: ['mcld_wallpaper_get_last_niters', 'number', []],
  yAt: ['mcld_wallpaper_y_at', 'number', ['number']],
  zAt: ['mcld_wallpaper_z_at', 'number', ['number']],
  pAt: ['mcld_wallpaper_p_at', 'number', ['number']],
  getPotentialInitial: ['mcld_wallpaper_get_potential_initial', 'number', []],
  getEnergyInitial: ['mcld_wallpaper_get_energy_initial', 'number', []],
  getEnergyCurrent: ['mcld_wallpaper_get_energy_current', 'number', []],
  getQErr: ['mcld_wallpaper_get_qerr', 'number', []],
  getQ1Err: ['mcld_wallpaper_get_q1err', 'number', []],
  getQ2Err: ['mcld_wallpaper_get_q2err', 'number', []],
};

function post(type, extra = {}) { self.postMessage({ type, ...extra }); }
function sanitizeBufferConfig(raw) {
  const lowWaterRaw = Math.max(1, Number(raw?.lowWater) || DEFAULT_BUFFER_CONFIG.lowWater);
  const highWaterRaw = Math.max(lowWaterRaw + 1, Number(raw?.highWater) || DEFAULT_BUFFER_CONFIG.highWater);
  const chunkSizeRaw = Math.max(1, Number(raw?.chunkSize) || DEFAULT_BUFFER_CONFIG.chunkSize);
  const lowWater = Math.max(24, Math.min(4096, Math.round(lowWaterRaw)));
  const highWater = Math.max(lowWater + 8, Math.min(8192, Math.round(highWaterRaw)));
  const chunkSize = Math.max(4, Math.min(48, Math.round(chunkSizeRaw), highWater));
  return { lowWater, highWater, chunkSize };
}

function readArray(length, getter) {
  const out = new Array(length);
  for (let i = 0; i < length; i += 1) out[i] = getter(i);
  return out;
}
function hypot3(a) { return Math.hypot(a[0], a[1], a[2]); }
function sub3(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function add3(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function scale3(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
function cross3(a, b) { return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }
function qNormalize4(q) {
  const n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  if (!(n2 > 1e-20)) return [1, 0, 0, 0];
  const inv = 1 / Math.sqrt(n2);
  return [q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv];
}
function qRow3(q) {
  const [w, x, y, z] = qNormalize4(q);
  return [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)];
}

async function ensureApi(simDt = 1 / 240) {
  if (api) return api;
  if (booting) return booting;
  booting = (async () => {
    try {
      importScripts('./mcld_solver.js');
      const factory = self.createMcldModule;
      if (typeof factory !== 'function') throw new Error('No global createMcldModule found in worker');
      moduleInstance = await factory();
      if (!moduleInstance || !moduleInstance.cwrap) throw new Error('Loaded module has no cwrap');
      api = {};
      for (const [key, spec] of Object.entries(CWRAP_SPECS)) {
        api[key] = moduleInstance.cwrap(spec[0], spec[1], spec[2]);
      }
      const rc = api.initDefault(simDt);
      if (rc !== 0) throw new Error(`solver init failed: ${rc}`);
      post('ready');
      return api;
    } catch (err) {
      post('unavailable', { message: err && err.message ? err.message : String(err) });
      throw err;
    }
  })();
  return booting;
}

function pushConfigArray(slot, values) {
  for (let i = 0; i < values.length; i += 1) api.configSetReal(slot, i, Number(values[i]) || 0);
}

function readPersistSnapshot() {
  const qHex = new Array(8);
  const pHex = new Array(6);
  for (let i = 0; i < 8; i += 1) qHex[i] = api.persistGetHex(0, i);
  for (let i = 0; i < 6; i += 1) pHex[i] = api.persistGetHex(1, i);
  return {
    qHex,
    pHex,
    tHex: api.persistGetHex(2, 0),
    HHex: api.persistGetHex(3, 0),
  };
}
function configureSolver(cfg) {
  api.configure(
    SOLVER_MODE_IDS[cfg.solverMode],
    INIT_MODE_IDS[cfg.initMode],
    Number(cfg.simDt),
    Number(cfg.maxIter),
    Number(cfg.tol),
    Number(cfg.beta),
  );
  for (let i = 0; i < cfg.params.length; i += 1) api.paramSet(i, Number(cfg.params[i]) || 0);
  pushConfigArray(0, cfg.yPrev);
  pushConfigArray(1, cfg.yCur);
  pushConfigArray(2, cfg.pLeft);
  pushConfigArray(3, cfg.pRight);
  pushConfigArray(6, cfg.angularVelocity || [0, 0, 0, 0, 0, 0]);
  api.configSetInt(0, cfg.angularVelocity ? 1 : 0);
}
function pushRestoreState(saved) {
  if (!saved) return;
  const hasHex = Array.isArray(saved.qHex) && saved.qHex.length === 8
    && Array.isArray(saved.pHex) && saved.pHex.length === 6
    && typeof saved.tHex === 'string'
    && typeof saved.HHex === 'string';
  if (hasHex) {
    for (let i = 0; i < 8; i += 1) api.restoreSetHex(0, i, saved.qHex[i]);
    for (let i = 0; i < 6; i += 1) api.restoreSetHex(1, i, saved.pHex[i]);
    api.restoreMetaHex(saved.tHex, saved.HHex);
    return;
  }
  pushConfigArray(4, saved.q);
  pushConfigArray(5, saved.p);
  api.restoreMeta(Number(saved.t) || 0, Number(saved.H) || 0);
}
function pointFromGeom(g, index) {
  const base = index === 1 ? 3 : index === 2 ? 6 : 0;
  return [g[base], g[base + 1], g[base + 2]];
}
function readGeom() { return readArray(27, api.geometryAt); }
function readY() { return readArray(8, api.yAt); }
function readZ() { return readArray(6, api.zAt); }
function readP() { return readArray(6, api.pAt); }

function speedFromZAvg(zPrev, zCur, yCur, dt, params) {
  const h = Math.max(dt, 1e-6);
  const l1 = Math.max(Number(params[0]) || 0, 1e-9);
  const l2 = Math.max(Number(params[1]) || 0, 1e-9);
  const n1 = qRow3(yCur.slice(0, 4));
  const n2 = qRow3(yCur.slice(4, 8));
  const z1 = [0.5 * (zPrev[0] + zCur[0]), 0.5 * (zPrev[1] + zCur[1]), 0.5 * (zPrev[2] + zCur[2])];
  const z2 = [0.5 * (zPrev[3] + zCur[3]), 0.5 * (zPrev[4] + zCur[4]), 0.5 * (zPrev[5] + zCur[5])];
  const v1 = scale3(cross3(scale3(z1, 1 / h), n1), l1);
  const v2 = add3(v1, scale3(cross3(scale3(z2, 1 / h), n2), l2));
  return { v1: hypot3(v1), v2: hypot3(v2) };
}

function estimateSummary(cfg) {
  const totalLen = Math.max(0.1, Number(cfg.params[0]) + Number(cfg.params[1]));
  const g = Math.max(1e-6, Math.abs(Number(cfg.params[10])));
  const horizon = Math.min(7.5, Math.max(2.5, 5.0 * Math.sqrt(totalLen / g)));
  const steps = Math.min(900, Math.max(180, Math.ceil(horizon / Math.max(Number(cfg.simDt), 1e-5))));
  let geom = readGeom();
  let prev1 = pointFromGeom(geom, 1);
  let prev2 = pointFromGeom(geom, 2);
  let maxSpeed1 = 0;
  let maxSpeed2 = 0;
  let maxZ = -1e9;
  let minZ = 1e9;
  let maxRadius = 0;
  const dt = Math.max(Number(cfg.simDt), 1e-6);
  for (let i = 0; i < steps; i += 1) {
    api.stepN(1);
    geom = readGeom();
    const p1 = pointFromGeom(geom, 1);
    const p2 = pointFromGeom(geom, 2);
    const s1 = hypot3(sub3(p1, prev1)) / dt;
    const s2 = hypot3(sub3(p2, prev2)) / dt;
    if (Number.isFinite(s1)) maxSpeed1 = Math.max(maxSpeed1, s1);
    if (Number.isFinite(s2)) maxSpeed2 = Math.max(maxSpeed2, s2);
    maxZ = Math.max(maxZ, geom[2], geom[5], geom[8]);
    minZ = Math.min(minZ, geom[2], geom[5], geom[8]);
    maxRadius = Math.max(maxRadius, hypot3(p1), hypot3(p2));
    prev1 = p1;
    prev2 = p2;
  }
  return { maxSpeed1, maxSpeed2, maxZ, minZ, maxRadius };
}

function makeFrame(rc, includePersistSnapshot = false) {
  const t = api.getTime();
  const g = readGeom();
  const y = readY();
  const z = readZ();
  let v1 = 0.0;
  let v2 = 0.0;
  if (prevFrameZ) {
    const sp = speedFromZAvg(prevFrameZ, z, y, Number(currentCfg?.simDt) || 1 / 240, currentCfg?.params || [0.5, 1.0]);
    v1 = sp.v1;
    v2 = sp.v2;
  }
  prevFrameZ = z;
  frameCounter += 1;
  return {
    t,
    g,
    y,
    p: readP(),
    v1,
    v2,
    rc,
    libRc: api.getLastLibRc(),
    residual: api.getLastResidual(),
    niters: api.getLastNiters(),
    diag: (frameCounter % 15 === 0) ? {
      H_cur: api.getEnergyCurrent(),
      q_err: api.getQErr(),
      q1_err: api.getQ1Err(),
      q2_err: api.getQ2Err(),
    } : null,
    persistSnapshot: includePersistSnapshot ? readPersistSnapshot() : null,
  };
}

function scheduleFill(localGen) {
  if (filling) return;
  filling = true;
  const pump = () => {
    if (generation !== localGen || !api) { filling = false; return; }
    if (queueCount >= bufferConfig.highWater) { filling = false; return; }
    const budget = Math.min(bufferConfig.chunkSize, bufferConfig.highWater - queueCount);
    const frames = new Array(budget);
    for (let i = 0; i < budget; i += 1) {
      frames[i] = makeFrame(api.stepN(1), ((frameCounter + 1) % 15 === 0));
    }
    queueCount += frames.length;
    if (frames.length) post('frames', { generation: localGen, frames });
    if (queueCount < bufferConfig.highWater && generation === localGen) setTimeout(pump, 0);
    else filling = false;
  };
  setTimeout(pump, 0);
}

self.onmessage = async (ev) => {
  const data = ev.data || {};
  try {
    if (data.type === 'boot') {
      bufferConfig = sanitizeBufferConfig(data.bufferConfig);
      await ensureApi(data.simDt || 1 / 240);
      return;
    }
    await ensureApi();
    if (data.type === 'setRenderLengths') {
      currentRenderLengths = {
        l1Vis: Number(data.l1Vis) || currentRenderLengths.l1Vis,
        l2Vis: Number(data.l2Vis) || currentRenderLengths.l2Vis,
      };
      api.setRenderLengths(currentRenderLengths.l1Vis, currentRenderLengths.l2Vis);
      return;
    }
    if (data.type === 'applyInit') {
      generation += 1;
      queueCount = 0;
      filling = false;
      frameCounter = 0;
      prevFrameZ = null;
      const localGen = generation;
      bufferConfig = sanitizeBufferConfig(data.bufferConfig);
      currentCfg = data.cfg;
      currentRenderLengths = {
        l1Vis: Number(data.l1Vis) || 0.5,
        l2Vis: Number(data.l2Vis) || 1.0,
      };

      configureSolver(currentCfg);
      let rc = api.applyInit();
      if (rc !== 0) throw new Error(`apply_init returned ${rc}`);
      api.setRenderLengths(currentRenderLengths.l1Vis, currentRenderLengths.l2Vis);
      const summary = estimateSummary(currentCfg);
      summary.H0 = api.getEnergyInitial();
      summary.U0 = api.getPotentialInitial();

      configureSolver(currentCfg);
      if (data.savedState) {
        pushRestoreState(data.savedState);
        rc = api.applyRestore();
        if (rc !== 0) throw new Error(`apply_restore returned ${rc}`);
      } else {
        rc = api.applyInit();
        if (rc !== 0) throw new Error(`reapply_init returned ${rc}`);
      }
      api.setRenderLengths(currentRenderLengths.l1Vis, currentRenderLengths.l2Vis);
      const initialFrame = makeFrame(0, true);
      const prefillCount = Math.min(Math.max(bufferConfig.lowWater, bufferConfig.chunkSize), bufferConfig.highWater);
      const frames = new Array(prefillCount);
      for (let i = 0; i < prefillCount && generation === localGen; i += 1) {
        frames[i] = makeFrame(api.stepN(1), ((frameCounter + 1) % 15 === 0));
      }
      queueCount = frames.length;
      post('applied', { ok: true, generation: localGen, simTime: initialFrame.t, summary, initialFrame, frames });
      scheduleFill(localGen);
      return;
    }
    if (data.type === 'drain') {
      if (data.bufferConfig) bufferConfig = sanitizeBufferConfig(data.bufferConfig);
      queueCount = Math.max(0, queueCount - (Number(data.consumed) || 0));
      if (data.requestFill && queueCount < bufferConfig.highWater) scheduleFill(generation);
      return;
    }
  } catch (err) {
    const message = err && err.message ? err.message : String(err);
    if (data.type === 'applyInit') post('apply_error', { message });
    else post('status', { message });
  }
};
