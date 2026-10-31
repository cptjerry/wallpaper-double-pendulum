const canvas = document.getElementById('stage');
const gl = canvas.getContext('webgl2', { alpha: true, antialias: true, premultipliedAlpha: false, preserveDrawingBuffer: false }) || canvas.getContext('webgl', { alpha: true, antialias: true, premultipliedAlpha: false, preserveDrawingBuffer: false }) || canvas.getContext('experimental-webgl');
const statusEl = document.getElementById('status');
const hudEl = document.getElementById('hud');
if (statusEl) { statusEl.style.whiteSpace = 'pre'; statusEl.style.fontVariantNumeric = 'tabular-nums'; }

const speedSliderEl = null;
const speedValueEl = null;
const rotateSliderEl = null;
const rotateValueEl = null;
const togglePlayEl = null;
const rodFit1DisplayEl = null;
const rodFit2DisplayEl = null;
const rodFitWarningEl = null;

const ENABLE_POTENTIAL_SHELL = true;
const ShellModule = window.PotentialShell || null;

function cloneConfig(cfg){ return JSON.parse(JSON.stringify(cfg)); }


const state = {
  playing: true,
  renderSpeed: 0.5,
  autoRotateZ: 0.0,
  fadeMode: 'time',
  trailSeconds: 7.0,
  trailMaxLength: 10.0,
  simDt: 1 / 240,
  substepsPerFrame: 2,
  showBar: true,
  barLength: 1.8,
  barStripeAngle: 0.78,
  barStripeWidthScale: 0.18,
  rodRadius1: 0.040,
  rodRadius2: 0.032,
  l1Vis: 0.5,
  l2Vis: 1.0,
  lcRatio1: 1.0,
  lcRatio2: 1.0,
  rodFit1: null,
  rodFit2: null,
  rodSpec1: { mode: 'auto' },
  rodSpec2: { mode: 'auto' },
  traceSpeedMax1: 1.0,
  traceSpeedMax2: 1.0,
  peakHeight: 0.0,
  lowHeight: 0.0,
  viewCenterZ: 0.0,
  viewRadius: 1.5,
  backgroundStrength: 1.0,
  useWasm: false,
  initializing: false,
  showPotentialShell: false,
  showHud: true,
  latestPotentialSummary: null,
  latestShellCfg: null,
  overThresholdCount: 0,
  activeTol: 1.5e-11,
  workerGeneration: 0,
  rodColor1: [12, 12, 16],
  rodColor2: [12, 12, 16],
  barColor1: [182, 156, 62],
  barColor2: [38, 74, 162],
  ballColorRoot: [255, 214, 60],
  ballColorJoint: [170, 177, 186],
  ballColorTip: [220, 228, 236],
  ballRadiusRoot: 0.018,
  ballRadiusJoint: 0.022,
  ballRadiusTip: 0.020,
  trace1Slow: [108, 246, 255],
  trace1Mid: [118, 170, 255],
  trace1Fast: [204, 84, 255],
  trace2Slow: [123, 255, 124],
  trace2Mid: [255, 214, 96],
  trace2Fast: [255, 82, 72],
  backgroundColor: [9, 11, 13],
  schemeColor: [0, 0, 0],
  themeAuto: false,
  themeState: null,
  keyLightColor: [255, 248, 240],
  hemiSkyColor: [46, 54, 70],
  hemiGroundColor: [20, 20, 22],
  rimLightColor: [255, 255, 255],
  persistStateEnabled: false,
  simTimeOffset: 0.0,
  configSignature: '',
  actualFps: 0.0,
  targetFps: 0.0,
  viewDistanceScale: 1.0,
  screenOffsetX: 0.0,
  screenOffsetY: 0.0,
  hudPosition: 'top_left',
  hudScale: 1.0,
  controlMode: 0,
  controlPitch: 0.24,
  controlRotateSpeed: 0.0,
};

const camera = {
  orbitQuat: [0.0, 0.0, 0.0, 1.0],
  distance: 3.2,
  target: [0.0, 0.0, 0.0],
  dragging: false,
  lastX: 0,
  lastY: 0,
  lastMoveTs: 0,
  releaseYawSpeed: 0.0,
  pointerId: null,
};

const WALLPAPER_CONFIG_URL = './wallpaper_config.json';
const PERSIST_STORAGE_KEY = 'mcld_wallpaper_saved_state';
const PERSIST_INTERVAL_MS = 5000;
const CAMERA_PITCH_LIMIT = Math.PI * 0.5;
const CAMERA_DRAG_SENSITIVITY = 0.0016;
const CAMERA_RELEASE_SPEED_LIMIT = 0.8;

const SOLVER_MODE_IDS = {
  lagrange: 0,
  hamilton_left: 1,
  hamilton_right: 2,
};

const INIT_MODE_IDS = {
  pair: 0,
  left: 1,
  right: 2,
};

const PARAM_LABELS = ['l1','l2','m1','m2','lc1','lc2','Iperp1','Iperp2','Ispin1','Ispin2','g'];
const ROD_RADIUS_STOP = 2.0;
const ROD_FIT_LOSS_STOP = 0.12;

const DEFAULT_WEB_CONFIG = {
  solverMode: 'hamilton_left',
  initMode: 'left',
  simDt: 1 / 240,
  maxIter: 6,
  tol: 1.5e-11,
  beta: 1.0,
  params: [0.5, 1.0, 3.0, 1.0, 0.25, 0.5, 0.04, 0.05, 0.003, 0.001, 9.81],
  yPrev: [1, 0, 0, 0, 1, 0, 0, 0],
  yCur: [0.70710678, 0.0, 0.70710678, 0.0, 0.70710678, 0.70710678, 0.0, 0.0],
  pLeft: [0, 0, 0, 0, 0, 0],
  pRight: [0, 0, 0, 0, 0, 0],
  angularVelocity: null,
  rods: {
    rod1: { mode: 'auto' },
    rod2: { mode: 'auto' },
  },
  balls: {
    rootRadius: 0.018,
    jointRadius: 0.022,
    tipRadius: 0.020,
  },
};

function trailCount(trail){ return trail.items.length - trail.start; }
function trailAt(trail, i){ return trail.items[trail.start + i]; }
function trailClear(trail){ trail.items.length = 0; trail.start = 0; }
function trailTrimPrefix(trail, removeCount){
  if (!(removeCount > 0)) return;
  trail.start = Math.min(trail.start + removeCount, trail.items.length);
  if (trail.start > 64 && trail.start * 2 >= trail.items.length) {
    trail.items = trail.items.slice(trail.start);
    trail.start = 0;
  }
}

function v3(x=0,y=0,z=0){ return [x,y,z]; }
function add(a,b){ return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
function sub(a,b){ return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function mul(a,s){ return [a[0]*s, a[1]*s, a[2]*s]; }
function dot(a,b){ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function cross(a,b){ return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }
function len(a){ return Math.hypot(a[0], a[1], a[2]); }
function norm(a){ const l = len(a) || 1e-12; return [a[0]/l, a[1]/l, a[2]/l]; }
function clamp(x,a,b){ return Math.max(a, Math.min(b, x)); }
function quatNormalize(q){
  const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1e-12;
  return [q[0]/l, q[1]/l, q[2]/l, q[3]/l];
}
function quatMul(a,b){
  return [
    a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
    a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
    a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
    a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2],
  ];
}
function quatFromAxisAngle(axis, angle){
  const n = norm(axis);
  const h = 0.5 * angle;
  const s = Math.sin(h);
  return quatNormalize([n[0]*s, n[1]*s, n[2]*s, Math.cos(h)]);
}
function quatConjugate(q){ return [-q[0], -q[1], -q[2], q[3]]; }
function quatRotateVec(q, v){
  const qv = [v[0], v[1], v[2], 0.0];
  const t = quatMul(quatMul(q, qv), quatConjugate(q));
  return [t[0], t[1], t[2]];
}
// Roll-free orbit: rotated up.Z >= 0 is exactly the +/-90 degree limit.
// For q=[x,y,z,w], up.Z = w*w+z*z-x*x-y*y. No Euler extraction.
function cameraRotateOrbit(horizontalDelta = 0.0, verticalDelta = 0.0){
  let q = quatNormalize(camera.orbitQuat);
  if (Number.isFinite(horizontalDelta) && horizontalDelta !== 0.0) {
    q = quatNormalize(quatMul(quatFromAxisAngle([0, 0, 1], horizontalDelta), q));
  }
  if (Number.isFinite(verticalDelta) && verticalDelta !== 0.0) {
    // At most two quarter turns; larger same-direction input stops at a pole.
    let remaining = clamp(verticalDelta, -Math.PI, Math.PI);
    for (let i = 0; i < 2 && Math.abs(remaining) > 0; i++) {
      const direction = Math.sign(remaining);
      const forward = quatRotateVec(q, [-1, 0, 0]);
      const upZ = q[3]*q[3] + q[2]*q[2] - q[0]*q[0] - q[1]*q[1];
      if (upZ <= 1e-12 && direction * forward[2] < 0) break;
      const step = direction * Math.min(Math.abs(remaining), Math.PI / 2);
      const right = quatRotateVec(q, [0, 1, 0]);
      const candidate = quatNormalize(quatMul(quatFromAxisAngle(right, -step), q));
      const candidateUpZ = candidate[3]*candidate[3] + candidate[2]*candidate[2]
        - candidate[0]*candidate[0] - candidate[1]*candidate[1];
      if (candidateUpZ < -1e-14) {
        // Algebraic half-angle quaternion taking the forward vector to the
        // requested pole, without acos/asin/atan2 or a recovered pitch.
        const cosine = clamp(-direction * forward[2], -1, 1);
        const c = Math.sqrt(Math.max(0, (1 + cosine) / 2));
        const h = -direction * Math.sqrt(Math.max(0, (1 - cosine) / 2));
        q = quatNormalize(quatMul([right[0]*h, right[1]*h, right[2]*h, c], q));
        break;
      }
      q = candidate;
      remaining -= step;
    }
  }
  camera.orbitQuat = q;
}
function cameraSetPitch(inclination){
  if (!Number.isFinite(inclination)) return;
  // A user-supplied angle is an input only; never extract/store Euler angles.
  const target = clamp(inclination, -CAMERA_PITCH_LIMIT, CAMERA_PITCH_LIMIT);
  const q = quatNormalize(camera.orbitQuat);
  const forward = quatRotateVec(q, [-1, 0, 0]);
  const up = quatRotateVec(q, [0, 0, 1]);
  const sine = Math.sin(target), cosine = Math.cos(target);
  const deltaCos = clamp(cosine * up[2] - sine * forward[2], -1, 1);
  const deltaSin = sine * up[2] + cosine * forward[2];
  const sign = Math.abs(deltaSin) > 1e-14 ? Math.sign(deltaSin) : Math.sign(target);
  // Use the sine for small turns to avoid losing their magnitude in 1-cos.
  let c, h;
  if (deltaCos >= 0) {
    c = Math.sqrt((1 + deltaCos) / 2);
    h = -deltaSin / (2*c);
  } else {
    h = -sign * Math.sqrt((1 - deltaCos) / 2);
    c = Math.abs(h) > 0 ? Math.abs(deltaSin / (2*h)) : 1;
  }
  const right = quatRotateVec(q, [0, 1, 0]);
  camera.orbitQuat = quatNormalize(quatMul([right[0]*h, right[1]*h, right[2]*h, c], q));
}
function mix(a,b,t){ return a + (b-a)*t; }
function mix3(a,b,t){ return [mix(a[0],b[0],t), mix(a[1],b[1],t), mix(a[2],b[2],t)]; }
function rgba(rgb,a){ return `rgba(${rgb[0]|0},${rgb[1]|0},${rgb[2]|0},${a})`; }
function rotateAroundAxis(v, axis, angle){
  const c = Math.cos(angle), s = Math.sin(angle);
  return add(add(mul(v, c), mul(cross(axis, v), s)), mul(axis, dot(axis, v) * (1 - c)));
}
function makeFrameFromAxis(axis, spinPhase){
  const n = norm(axis);
  const ref = Math.abs(n[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
  const u0 = norm(cross(ref, n));
  const v0 = norm(cross(n, u0));
  const u = rotateAroundAxis(u0, n, spinPhase);
  const v = rotateAroundAxis(v0, n, spinPhase);
  return { n, u, v };
}

function colorRamp(speed, vmax, slow, mid, fast){
  const vmaxSafe = Math.max(vmax, 1e-6);
  const t = clamp((speed - 0.04 * vmaxSafe) / (0.92 * vmaxSafe), 0, 1);
  return t < 0.55 ? mix3(slow, mid, t / 0.55) : mix3(mid, fast, (t - 0.55) / 0.45);
}
function colorMapSpeed1(speed){
  return colorRamp(speed, state.traceSpeedMax1, state.trace1Slow, state.trace1Mid, state.trace1Fast);
}
function colorMapSpeed2(speed){
  return colorRamp(speed, state.traceSpeedMax2, state.trace2Slow, state.trace2Mid, state.trace2Fast);
}
function parseWallpaperColorValue(value, fallback){
  if (typeof value !== 'string') return [...fallback];
  const s = value.trim();
  const hex = s.match(/^#([0-9a-f]{6})$/i);
  if (hex) {
    const n = hex[1];
    return [parseInt(n.slice(0,2), 16), parseInt(n.slice(2,4), 16), parseInt(n.slice(4,6), 16)];
  }
  const hexShort = s.match(/^#([0-9a-f]{3})$/i);
  if (hexShort) {
    const n = hexShort[1];
    return [parseInt(n[0] + n[0], 16), parseInt(n[1] + n[1], 16), parseInt(n[2] + n[2], 16)];
  }
  const parts = s.split(/\s+/).map(Number).filter(Number.isFinite);
  if (parts.length >= 3) return parts.slice(0, 3).map(c => clamp(Math.round(c * 255), 0, 255));
  return [...fallback];
}

function qNormalize4(q){
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1e-12;
  return [q[0]/n, q[1]/n, q[2]/n, q[3]/n];
}

function qRow3(q){
  const [w,x,y,z] = qNormalize4(q);
  return [
    2 * (x * z - w * y),
    2 * (y * z + w * x),
    1 - 2 * (x * x + y * y),
  ];
}

function qDisplayFrame(q){
  const [w, x, y, z] = qNormalize4(q);
  const u = [
    1 - 2 * (y * y + z * z),
    2 * (x * y - w * z),
    -2 * (x * z + w * y),
  ];
  const v = [
    2 * (x * y + w * z),
    1 - 2 * (x * x + z * z),
    -2 * (y * z - w * x),
  ];
  const n = [
    2 * (x * z - w * y),
    2 * (y * z + w * x),
    -(1 - 2 * (x * x + y * y)),
  ];
  return { n: norm(n), u: norm(u), v: norm(v) };
}

function poseFromConfigState(cfg){
  const q1 = (cfg && cfg.yCur ? cfg.yCur.slice(0,4) : DEFAULT_WEB_CONFIG.yCur.slice(0,4));
  const q2 = (cfg && cfg.yCur ? cfg.yCur.slice(4,8) : DEFAULT_WEB_CONFIG.yCur.slice(4,8));
  const f1 = qDisplayFrame(q1);
  const f2 = qDisplayFrame(q2);
  const root = [0,0,0];
  const joint = mul(f1.n, state.l1Vis);
  const tip = add(joint, mul(f2.n, state.l2Vis));
  return { root, joint, tip, n1: f1.n, u1: f1.u, v1: f1.v, n2: f2.n, u2: f2.u, v2: f2.v };
}

function quatFromRotationMatrixRows(R){
  const m00 = R[0][0], m01 = R[0][1], m02 = R[0][2];
  const m10 = R[1][0], m11 = R[1][1], m12 = R[1][2];
  const m20 = R[2][0], m21 = R[2][1], m22 = R[2][2];
  const trace = m00 + m11 + m22;
  let w, x, y, z;
  if (trace > 0){
    const s = Math.sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m21 - m12) / s;
    y = (m02 - m20) / s;
    z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22){
    const s = Math.sqrt(1.0 + m00 - m11 - m22) * 2.0;
    w = (m21 - m12) / s;
    x = 0.25 * s;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
  } else if (m11 > m22){
    const s = Math.sqrt(1.0 + m11 - m00 - m22) * 2.0;
    w = (m02 - m20) / s;
    x = (m01 + m10) / s;
    y = 0.25 * s;
    z = (m12 + m21) / s;
  } else {
    const s = Math.sqrt(1.0 + m22 - m00 - m11) * 2.0;
    w = (m10 - m01) / s;
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = 0.25 * s;
  }
  return qNormalize4([w, x, y, z]);
}

function quatFromDisplayFrame(frame){
  const { u, v, n } = frame;
  const R = [
    [u[0], u[1], -u[2]],
    [v[0], v[1], -v[2]],
    [n[0], n[1], -n[2]],
  ];
  return quatFromRotationMatrixRows(R);
}

function normalizeAngleTriple(raw, unit = 'rad'){
  let triple;
  if (Array.isArray(raw)) {
    const [yaw = 0, pitch = 0, spin = 0] = raw;
    triple = { yaw: Number(yaw) || 0, pitch: Number(pitch) || 0, spin: Number(spin) || 0 };
  } else {
    triple = {
      yaw: Number(raw?.yaw) || 0,
      pitch: Number(raw?.pitch) || 0,
      spin: Number(raw?.spin) || 0,
    };
  }
  const unitText = String(unit || 'rad').toLowerCase();
  if (unitText.startsWith('deg')) {
    const k = Math.PI / 180.0;
    triple.yaw *= k;
    triple.pitch *= k;
    triple.spin *= k;
  } else if (!unitText.startsWith('rad')) {
    throw new Error(`angles.unit must be "deg" or "rad"`);
  }
  return triple;
}

function rodFrameFromAngles(angleTriple){
  const yaw = angleTriple?.yaw || 0;
  const pitch = clamp(angleTriple?.pitch || 0, -Math.PI * 0.5, Math.PI * 0.5);
  const spin = angleTriple?.spin || 0;
  const n = norm([
    -Math.cos(pitch) * Math.cos(yaw),
    -Math.cos(pitch) * Math.sin(yaw),
    -Math.sin(pitch),
  ]);
  const ref = Math.abs(dot(n, [0, 1, 0])) < 0.95 ? [0, 1, 0] : [0, 0, 1];
  const u0 = norm(cross(n, ref));
  const v0 = norm(cross(u0, n));
  const u = rotateAroundAxis(u0, n, spin);
  const v = rotateAroundAxis(v0, n, spin);
  return { n, u, v };
}

function quatFromRodAngles(rawTriple, unit){
  return quatFromDisplayFrame(rodFrameFromAngles(normalizeAngleTriple(rawTriple, unit)));
}

function paramsArrayFromJson(rawParams){
  if (Array.isArray(rawParams)) {
    if (rawParams.length !== 11) throw new Error(`params must contain 11 entries, got ${rawParams.length}`);
    const vals = rawParams.map(Number);
    vals.forEach((v, i) => { if (!Number.isFinite(v)) throw new Error(`params[${i}] must be finite`); });
    return vals;
  }
  const obj = rawParams || {};
  const vals = [obj.l1, obj.l2, obj.m1, obj.m2, obj.lc1, obj.lc2, obj.Iperp1, obj.Iperp2, obj.Ispin1, obj.Ispin2, obj.g].map(Number);
  vals.forEach((v, i) => { if (!Number.isFinite(v)) throw new Error(`params.${PARAM_LABELS[i]} must be finite`); });
  return vals;
}

function vector6OrZeros(raw, label){
  if (raw == null) return [0, 0, 0, 0, 0, 0];
  if (!Array.isArray(raw) || raw.length !== 6) throw new Error(`${label} must contain 6 numbers`);
  const vals = raw.map(Number);
  vals.forEach((v, i) => { if (!Number.isFinite(v)) throw new Error(`${label}[${i}] must be finite`); });
  return vals;
}

function vector8OrNull(raw, label){
  if (raw == null) return null;
  if (!Array.isArray(raw) || raw.length !== 8) throw new Error(`${label} must contain 8 numbers`);
  const vals = raw.map(Number);
  vals.forEach((v, i) => { if (!Number.isFinite(v)) throw new Error(`${label}[${i}] must be finite`); });
  return vals;
}

function normalizeOmegaTriple(raw, unit = 'rad/s', label = 'angularVelocity'){
  let vals;
  if (Array.isArray(raw)) {
    const [x = 0, y = 0, z = 0] = raw;
    vals = [Number(x) || 0, Number(y) || 0, Number(z) || 0];
  } else if (raw && typeof raw === 'object') {
    vals = [Number(raw.x) || 0, Number(raw.y) || 0, Number(raw.z) || 0];
  } else {
    throw new Error(`${label} must be a 3-vector`);
  }
  const unitText = String(unit || 'rad/s').toLowerCase();
  if (unitText.startsWith('deg')) {
    const k = Math.PI / 180.0;
    vals = vals.map(v => v * k);
  } else if (!unitText.startsWith('rad')) {
    throw new Error(`${label}.unit must be "rad/s" or "deg/s"`);
  }
  vals.forEach((v, i) => { if (!Number.isFinite(v)) throw new Error(`${label}[${i}] must be finite`); });
  return vals;
}

function angularVelocityFromJson(raw){
  if (raw == null) return null;
  if (Array.isArray(raw)) return vector6OrZeros(raw, 'angularVelocity');
  if (typeof raw !== 'object') throw new Error('angularVelocity must be an object, array, or null');
  const unit = String(raw.unit || raw.angleUnit || 'rad/s').toLowerCase();
  const frame = String(raw.frame || 'body').toLowerCase();
  if (frame !== 'body') throw new Error('angularVelocity.frame currently only supports "body"');
  const current = (raw.current && typeof raw.current === 'object') ? raw.current : raw;
  if (!current.rod1 || !current.rod2) throw new Error('angularVelocity.current requires rod1 and rod2 3-vectors');
  return [
    ...normalizeOmegaTriple(current.rod1, unit, 'angularVelocity.current.rod1'),
    ...normalizeOmegaTriple(current.rod2, unit, 'angularVelocity.current.rod2'),
  ];
}

function yStateFromAnglesPair(rawPair, unit, label){
  if (!rawPair || !rawPair.rod1 || !rawPair.rod2) throw new Error(`${label} requires rod1 and rod2 angle triples`);
  return [
    ...quatFromRodAngles(rawPair.rod1, unit),
    ...quatFromRodAngles(rawPair.rod2, unit),
  ];
}

function normalizeRodConfig(rawRod, label){
  if (rawRod == null) return { mode: 'auto' };
  const rawObj = (typeof rawRod === 'object' && rawRod !== null) ? rawRod : {};
  const mode = String(rawObj.mode || (rawObj.auto === false ? 'manual' : 'auto')).toLowerCase();
  if (mode === 'auto') return { mode: 'auto' };
  if (mode !== 'manual') throw new Error(`${label}.mode must be "auto" or "manual"`);
  const shapeSrc = (rawObj.shape && typeof rawObj.shape === 'object') ? rawObj.shape : rawObj;
  const out = {
    mode: 'manual',
    shape: {
      a: Number(shapeSrc.a),
      b: Number(shapeSrc.b),
      c: shapeSrc.c == null ? null : Number(shapeSrc.c),
      R: Number(shapeSrc.R),
    },
  };
  if (!Number.isFinite(out.shape.a) || !Number.isFinite(out.shape.b) || !Number.isFinite(out.shape.R)) {
    throw new Error(`${label}.shape requires finite a, b, R`);
  }
  if (out.shape.c != null && !Number.isFinite(out.shape.c)) throw new Error(`${label}.shape.c must be finite when provided`);
  return out;
}


function normalizeBallConfig(rawBalls){
  const src = (rawBalls && typeof rawBalls === 'object') ? rawBalls : {};
  const out = {
    rootRadius: src.rootRadius == null ? DEFAULT_WEB_CONFIG.balls.rootRadius : Number(src.rootRadius),
    jointRadius: src.jointRadius == null ? DEFAULT_WEB_CONFIG.balls.jointRadius : Number(src.jointRadius),
    tipRadius: src.tipRadius == null ? DEFAULT_WEB_CONFIG.balls.tipRadius : Number(src.tipRadius),
  };
  if (!(out.rootRadius > 0) || !(out.jointRadius > 0) || !(out.tipRadius > 0)) {
    throw new Error('balls radii must be > 0');
  }
  return out;
}

function configFromWallpaperJson(raw){
  if (!raw || typeof raw !== 'object') throw new Error('wallpaper_config.json must contain an object');
  const cfg = cloneConfig(DEFAULT_WEB_CONFIG);
  cfg.solverMode = String(raw.solverMode || cfg.solverMode);
  cfg.initMode = String(raw.initMode || cfg.initMode);
  if (raw.simDt != null) cfg.simDt = Number(raw.simDt);
  if (raw.maxIter != null) cfg.maxIter = Number(raw.maxIter);
  if (raw.tol != null) cfg.tol = Number(raw.tol);
  if (raw.beta != null) cfg.beta = Number(raw.beta);
  cfg.params = paramsArrayFromJson(raw.params || cfg.params);

  const angleSection = raw.angles || raw.initial || {};
  const quaternionSection = (raw.quaternion && typeof raw.quaternion === 'object') ? raw.quaternion : {};
  const unit = String(angleSection.unit || raw.angleUnit || 'rad').toLowerCase();
  cfg.yCur = yStateFromAnglesPair(angleSection.current || angleSection.yCur, unit, 'angles.current')
    || vector8OrNull(quaternionSection.current, 'quaternion.current');

  cfg.yPrev = ((angleSection.previous || angleSection.yPrev)
      ? yStateFromAnglesPair(angleSection.previous || angleSection.yPrev, unit, 'angles.previous')
      : cfg.yPrev)
    || vector8OrNull(quaternionSection.previous, 'quaternion.previous');

  const momenta = (raw.momenta && typeof raw.momenta === 'object') ? raw.momenta : {};
  cfg.pLeft = vector6OrZeros(momenta.left, 'momenta.left');
  cfg.pRight = vector6OrZeros(momenta.right, 'momenta.right');
  cfg.angularVelocity = angularVelocityFromJson(raw.angularVelocity ?? raw.omega ?? null);

  const rodsRaw = raw.rods || {};
  cfg.rods = {
    rod1: normalizeRodConfig(rodsRaw.rod1, 'rods.rod1'),
    rod2: normalizeRodConfig(rodsRaw.rod2, 'rods.rod2'),
  };
  cfg.balls = normalizeBallConfig(raw.balls);

  if (!(cfg.simDt > 0)) throw new Error('simDt must be > 0');
  if (!(cfg.maxIter > 0)) throw new Error('maxIter must be > 0');
  if (!(cfg.tol > 0)) throw new Error('tol must be > 0');
  if (!Number.isFinite(cfg.beta)) throw new Error('beta must be finite');
  return cfg;
}

function hashStringFNV1a(str){
  let h = 0x811c9dc5;
  for (let i = 0; i < str.length; i += 1) {
    h ^= str.charCodeAt(i);
    h = Math.imul(h, 0x01000193);
  }
  return `fnv1a:${(h >>> 0).toString(16).padStart(8, '0')}`;
}

async function fetchWallpaperConfigRecord(){
  const response = await fetch(WALLPAPER_CONFIG_URL, { cache: 'no-store' });
  if (!response.ok) throw new Error(`failed to load wallpaper_config.json (${response.status})`);
  const text = await response.text();
  const raw = JSON.parse(text);
  return { cfg: configFromWallpaperJson(raw), signature: hashStringFNV1a(text) };
}

async function fetchWallpaperConfig(){
  return (await fetchWallpaperConfigRecord()).cfg;
}

function normalizePersistTrail(raw, maxPoints){
  if (!Array.isArray(raw)) return [];
  const out = [];
  for (let i = Math.max(0, raw.length - maxPoints); i < raw.length; i += 1) {
    const item = raw[i];
    if (!item || !Array.isArray(item.pos) || item.pos.length !== 3) continue;
    const pos = item.pos.map(Number);
    const speed = Number(item.speed);
    const t = Number(item.t);
    if (pos.every(Number.isFinite) && Number.isFinite(speed) && Number.isFinite(t)) out.push({ pos, speed, t });
  }
  return out;
}

function normalizePersistCamera(raw){
  if (!raw || typeof raw !== 'object') return null;
  const orbitQuat = Array.isArray(raw.orbitQuat) && raw.orbitQuat.length === 4 ? raw.orbitQuat.map(Number) : null;
  const target = Array.isArray(raw.target) && raw.target.length === 3 ? raw.target.map(Number) : null;
  const distance = Number(raw.distance);
  const autoRotateZ = Number(raw.autoRotateZ);
  if (!orbitQuat || !orbitQuat.every(Number.isFinite)) return null;
  if (!target || !target.every(Number.isFinite)) return null;
  if (!Number.isFinite(distance) || distance <= 0) return null;
  return {
    orbitQuat: quatNormalize(orbitQuat),
    target,
    distance,
    autoRotateZ: Number.isFinite(autoRotateZ) ? autoRotateZ : 0,
  };
}

function normalizePersistHexArray(raw, length){
  if (!Array.isArray(raw) || raw.length !== length) return null;
  const out = raw.map((item) => (typeof item === 'string' ? item.trim().toLowerCase() : ''));
  return out.every((item) => /^[0-9a-f]+$/.test(item) && item.length > 0 && item.length % 2 === 0) ? out : null;
}

function normalizePersistHexScalar(raw){
  if (typeof raw !== 'string') return null;
  const value = raw.trim().toLowerCase();
  return /^[0-9a-f]+$/.test(value) && value.length > 0 && value.length % 2 === 0 ? value : null;
}

function readPersistedSimState(){
  try {
    const raw = localStorage.getItem(PERSIST_STORAGE_KEY);
    if (!raw) return null;
    const obj = JSON.parse(raw);
    if (!obj || !Array.isArray(obj.q) || obj.q.length !== 8 || !Array.isArray(obj.p) || obj.p.length !== 6) return null;
    if (!Number.isFinite(obj.t) || !Number.isFinite(obj.H)) return null;
    const qHex = normalizePersistHexArray(obj.qHex, 8);
    const pHex = normalizePersistHexArray(obj.pHex, 6);
    const tHex = normalizePersistHexScalar(obj.tHex);
    const HHex = normalizePersistHexScalar(obj.HHex);
    return {
      q: obj.q.map(Number),
      p: obj.p.map(Number),
      t: Number(obj.t),
      H: Number(obj.H),
      qHex,
      pHex,
      tHex,
      HHex,
      signature: typeof obj.signature === 'string' ? obj.signature : '',
      trail1: normalizePersistTrail(obj.trail1, 60),
      trail2: normalizePersistTrail(obj.trail2, 120),
      camera: normalizePersistCamera(obj.camera),
      savedAt: Number(obj.savedAt) || 0,
    };
  } catch (err) {
    console.warn('[persist] failed to read saved state', err);
    return null;
  }
}

function clearPersistedSimState(){
  try { localStorage.removeItem(PERSIST_STORAGE_KEY); } catch (err) { console.warn('[persist] failed to clear saved state', err); }
}

function mat3FromBasisRows(u, v, n){
  return [
    [u[0], u[1], u[2]],
    [v[0], v[1], v[2]],
    [n[0], n[1], n[2]],
  ];
}

function mat3Mul(a, b){
  return [
    [a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0], a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1], a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2]],
    [a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0], a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1], a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2]],
    [a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0], a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1], a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2]],
  ];
}

function mat3Transpose(a){
  return [
    [a[0][0], a[1][0], a[2][0]],
    [a[0][1], a[1][1], a[2][1]],
    [a[0][2], a[1][2], a[2][2]],
  ];
}

function rotvecFromRotationMatrix(R){
  const tr = clamp((R[0][0] + R[1][1] + R[2][2] - 1) * 0.5, -1, 1);
  const ang = Math.acos(tr);
  if (ang < 1e-9) return [0, 0, 0];
  const s = Math.sin(ang);
  if (Math.abs(s) < 1e-8) {
    return [
      0.5 * (R[2][1] - R[1][2]),
      0.5 * (R[0][2] - R[2][0]),
      0.5 * (R[1][0] - R[0][1]),
    ];
  }
  const k = ang / (2 * s);
  return [
    k * (R[2][1] - R[1][2]),
    k * (R[0][2] - R[2][0]),
    k * (R[1][0] - R[0][1]),
  ];
}

function estimateCurrentSpeedsFromPoseDelta(prevPose, curPose, dt, params){
  const h = Math.max(dt, 1e-6);
  const l1 = Math.max(params[0], 1e-9);
  const l2 = Math.max(params[1], 1e-9);

  const R1prev = mat3FromBasisRows(prevPose.u1, prevPose.v1, prevPose.n1);
  const R1cur  = mat3FromBasisRows(curPose.u1,  curPose.v1,  curPose.n1);
  const R2prev = mat3FromBasisRows(prevPose.u2, prevPose.v2, prevPose.n2);
  const R2cur  = mat3FromBasisRows(curPose.u2,  curPose.v2,  curPose.n2);

  const dR1 = mat3Mul(mat3Transpose(R1prev), R1cur);
  const dR2 = mat3Mul(mat3Transpose(R2prev), R2cur);
  const z1 = rotvecFromRotationMatrix(dR1);
  const z2 = rotvecFromRotationMatrix(dR2);
  const w1 = mul(z1, 1 / h);
  const w2 = mul(z2, 1 / h);

  const v1 = mul(cross(w1, curPose.n1), l1);
  const v2 = add(v1, mul(cross(w2, curPose.n2), l2));
  return { s1: len(v1), s2: len(v2) };
}

function positionsFromY(y, params){
  const l1 = params[0], l2 = params[1];
  const n1 = qRow3(y.slice(0, 4));
  const n2 = qRow3(y.slice(4, 8));
  const joint = [l1 * n1[0], l1 * n1[1], l1 * n1[2]];
  const tip = [joint[0] + l2 * n2[0], joint[1] + l2 * n2[1], joint[2] + l2 * n2[2]];
  return { n1, n2, joint, tip };
}

function estimatePotentialEnergy(y, params){
  const l1 = params[0], m1 = params[2], m2 = params[3], lc1 = params[4], lc2 = params[5], g = Math.abs(params[10]);
  const { n1, n2 } = positionsFromY(y, params);
  return -g * (m1 * lc1 * n1[2] + m2 * (l1 * n1[2] + lc2 * n2[2]));
}

function estimateTraceSpeedScales(cfg){
  const l1 = Math.max(cfg.params[0], 1e-6);
  const l2 = Math.max(cfg.params[1], 1e-6);
  const m1 = Math.max(cfg.params[2], 1e-6);
  const m2 = Math.max(cfg.params[3], 1e-6);
  const lc1 = Math.max(Math.abs(cfg.params[4]), 1e-9);
  const lc2 = Math.max(Math.abs(cfg.params[5]), 1e-9);
  const Iperp1 = Math.max(cfg.params[6], 0.0);
  const Iperp2 = Math.max(cfg.params[7], 0.0);
  const g = Math.abs(cfg.params[10]);

  const U0 = estimatePotentialEnergy(cfg.yCur, cfg.params);
  const Umin = -g * (m1 * lc1 + m2 * (l1 + lc2));
  const deltaU = Math.max(0.0, U0 - Umin);

  const denom1 = Math.max(Iperp1 + m1 * lc1 * lc1, 1e-12);
  const denom2 = Math.max(Iperp2 + m2 * lc2 * lc2, 1e-12);

  const vmax1 = 1.01 * l1 * Math.sqrt((2 * deltaU) / denom1);
  const vmax2 = 1.01 * l2 * Math.sqrt((2 * deltaU) / denom2);

  return {
    maxSpeed1: Math.max(0.15, vmax1),
    maxSpeed2: Math.max(0.15, vmax2),
  };
}

function cccMassMoments(m, l, a, b, c, R){
  const M = a + b + 3 * c;
  if (!(M > 1e-12) || R < 0 || c < 0 || a < 0 || b < 0) return null;
  const m1 = m * a / M;
  const m2 = 3 * m * c / M;
  const m3 = m * b / M;
  const z1 = 0.75 * a;
  const z2 = a + 0.5 * c;
  const z3 = a + c + 0.25 * b;
  const lc = (m1 * z1 + m2 * z2 + m3 * z3) / Math.max(m, 1e-12);
  const I1c = (3 * m1 / 80) * (4 * R * R + a * a);
  const I2c = (m2 / 12) * (3 * R * R + c * c);
  const I3c = (3 * m3 / 80) * (4 * R * R + b * b);
  const Iperp = I1c + m1 * (z1 - lc) ** 2 + I2c + m2 * (z2 - lc) ** 2 + I3c + m3 * (z3 - lc) ** 2;
  const IspinDen = Math.max(a + b + 3 * c, 1e-12);
  const Ispin = (3 * m * R * R / 10) * ((a + b + 5 * c) / IspinDen);
  return { m1, m2, m3, z1, z2, z3, lc, Iperp, Ispin };
}

function rodLossMetrics(length, mass, lcTarget, IperpTarget, IspinTarget, shape){
  const l = Math.max(length, 1e-9);
  const a = Number(shape?.a);
  const b = Number(shape?.b);
  let c = shape?.c == null ? (l - a - b) : Number(shape.c);
  const R = Number(shape?.R);
  if (![a,b,c,R].every(Number.isFinite)) return { ok: false, reason: 'non-finite shape', loss: Infinity };
  if (a < 0 || b < 0 || c < 0 || R < 0) return { ok: false, reason: 'negative shape', loss: Infinity, a, b, c, R };
  const sumErrAbs = Math.abs((a + b + c) - l);
  if (sumErrAbs > Math.max(1e-6, 1e-3 * l)) return { ok: false, reason: 'a+b+c != l', loss: Infinity, a, b, c, R };
  c = Math.max(c, 0);
  const mm = cccMassMoments(Math.max(mass, 1e-9), l, a, b, c, R);
  if (!mm) return { ok: false, reason: 'invalid mass moments', loss: Infinity, a, b, c, R };
  const errLc = (mm.lc - clamp(lcTarget, 1e-6, l - 1e-6)) / l;
  const scaleI = Math.max(IperpTarget, 1e-9, l * l * Math.max(mass, 1e-9) * 0.02);
  const errI = (mm.Iperp - IperpTarget) / scaleI;
  const scaleS = Math.max(IspinTarget, 1e-9, l * l * Math.max(mass, 1e-9) * 0.01);
  const errS = (mm.Ispin - IspinTarget) / scaleS;
  const endPenalty = Math.max(0, 0.02 * l - a) / l + Math.max(0, 0.02 * l - b) / l;
  const cylPenalty = Math.max(0, 0.04 * l - c) / l;
  const sumPenalty = sumErrAbs / l;
  const loss = errLc * errLc * 9 + errI * errI + 0.6 * errS * errS + 0.25 * (endPenalty * endPenalty + cylPenalty * cylPenalty) + 4 * sumPenalty * sumPenalty;
  return { ok: true, a, b, c, R, lc: mm.lc, Iperp: mm.Iperp, Ispin: mm.Ispin, errLc, errI, errS, loss };
}

function solveRodShapeFromParams(length, mass, lc, Iperp, Ispin, fallbackRadius){
  const l = Math.max(length, 1e-9);
  const m = Math.max(mass, 1e-9);
  const lcTarget = clamp(lc, 1e-6, l - 1e-6);
  if (!(Ispin >= 0) || !(Iperp >= 0)) return { ok: false, reason: 'negative inertia', loss: Infinity };

  let best = null;
  const evalCandidate = (a, b) => {
    const c = l - a - b;
    if (a < 0 || b < 0 || c < 0) return;
    const denomSpin = a + b + 5 * c;
    const numerSpin = a + b + 3 * c;
    if (denomSpin <= 1e-12 || numerSpin <= 1e-12) return;
    const R2 = (10 * Ispin / (3 * m)) * (numerSpin / denomSpin);
    if (!(R2 >= 0) || !Number.isFinite(R2)) return;
    const R = Math.sqrt(R2);
    const mm = cccMassMoments(m, l, a, b, c, R);
    if (!mm) return;
    const errLc = (mm.lc - lcTarget) / l;
    const scaleI = Math.max(Iperp, 1e-9, l * l * m * 0.02);
    const errI = (mm.Iperp - Iperp) / scaleI;
    const endPenalty = Math.max(0, 0.02 * l - a) / l + Math.max(0, 0.02 * l - b) / l;
    const cylPenalty = Math.max(0, 0.04 * l - c) / l;
    const loss = errLc * errLc * 9 + errI * errI + 0.25 * (endPenalty * endPenalty + cylPenalty * cylPenalty);
    const cand = { ok: true, a, b, c, R, lc: mm.lc, Iperp: mm.Iperp, loss };
    if (!best || cand.loss < best.loss) best = cand;
  };

  const N = 48;
  for (let ia = 0; ia <= N; ++ia) {
    const a = l * (ia / N) * 0.78;
    for (let ib = 0; ib <= N; ++ib) {
      const b = l * (ib / N) * 0.78;
      if (a + b > l * 0.96) continue;
      evalCandidate(a, b);
    }
  }
  if (!best) return { ok: false, reason: 'no candidate', loss: Infinity };

  for (let pass = 0; pass < 4; ++pass) {
    const da = Math.max(l * 0.12 / (2 ** pass), 0.002 * l);
    const db = Math.max(l * 0.12 / (2 ** pass), 0.002 * l);
    const ca = best.a, cb = best.b;
    for (let i = -8; i <= 8; ++i) {
      for (let j = -8; j <= 8; ++j) {
        evalCandidate(ca + i * da / 8, cb + j * db / 8);
      }
    }
  }

  if (!best) return { ok: false, reason: 'refine failed', loss: Infinity };
  const minVisible = Math.max(0.18 * fallbackRadius, 0.006 * l);
  best.R = Math.max(best.R, minVisible);
  if (!Number.isFinite(best.R) || !Number.isFinite(best.a) || !Number.isFinite(best.b) || !Number.isFinite(best.c)) return { ok: false, reason: 'non-finite fit', loss: Infinity };
  if (best.R >= ROD_RADIUS_STOP) return { ...best, ok: false, reason: `R >= ${ROD_RADIUS_STOP}` };
  if (best.loss > ROD_FIT_LOSS_STOP) return { ...best, ok: false, reason: `loss too high (${best.loss.toFixed(3)})` };
  return best;
}

function formatRodFit(fit){
  if (!fit) return '(a,b,c,R) = --';
  const vals = ['a','b','c','R'].map(k => Number.isFinite(fit[k]) ? fit[k].toFixed(6) : '--');
  const mode = fit.mode ? `[${fit.mode}] ` : '';
  const suffix = Number.isFinite(fit.loss) ? ` · loss=${fit.loss.toFixed(4)}` : '';
  return `${mode}(a,b,c,R) = (${vals.join(', ')})${suffix}`;
}

function updateRodFitPanel(){
  if (!rodFit1DisplayEl || !rodFit2DisplayEl || !rodFitWarningEl) return;
  rodFit1DisplayEl.textContent = formatRodFit(state.rodFit1);
  rodFit2DisplayEl.textContent = formatRodFit(state.rodFit2);
  rodFit1DisplayEl.classList.toggle('rod-fit-bad', !!state.rodFit1 && !state.rodFit1.ok);
  rodFit2DisplayEl.classList.toggle('rod-fit-bad', !!state.rodFit2 && !state.rodFit2.ok);
  const msgs = [];
  if (!state.rodFit1 || !state.rodFit1.ok) msgs.push(`rod1: ${(state.rodFit1 && state.rodFit1.reason) || 'unsolved'}`);
  if (!state.rodFit2 || !state.rodFit2.ok) msgs.push(`rod2: ${(state.rodFit2 && state.rodFit2.reason) || 'unsolved'}`);
  rodFitWarningEl.textContent = msgs.length ? `Rod fit warning — ${msgs.join(' · ')}. Playback paused.` : '';
}

function updateSpeedUi(){
  if (speedSliderEl) speedSliderEl.value = String(clamp(state.renderSpeed, 0.0, 12.0));
  if (speedValueEl) speedValueEl.value = state.renderSpeed.toFixed(2);
  const rotateUi = clamp(-state.autoRotateZ, -1.5, 1.5);
  if (rotateSliderEl) rotateSliderEl.value = String(rotateUi);
  if (rotateValueEl) rotateValueEl.value = rotateUi.toFixed(2);
  if (togglePlayEl) togglePlayEl.textContent = state.playing ? 'Pause' : 'Play';
}

function applySpeedValue(v){
  if (!Number.isFinite(v)) return;
  const next = clamp(v, 0.0, 12.0);
  state.renderSpeed = next;
  state.playing = next > 0;
  updateSpeedUi();
}

function applyRotateValue(v){
  if (!Number.isFinite(v)) return;
  state.autoRotateZ = -clamp(v, -1.5, 1.5);
  updateSpeedUi();
}

function enforceRodFitValidity(){
  const bad1 = !state.rodFit1 || !state.rodFit1.ok;
  const bad2 = !state.rodFit2 || !state.rodFit2.ok;
  updateRodFitPanel();
  if (bad1 || bad2) {
    state.playing = false;
    updateSpeedUi();
    const why = [bad1 ? `rod1 ${(state.rodFit1 && state.rodFit1.reason) || 'unsolved'}` : null, bad2 ? `rod2 ${(state.rodFit2 && state.rodFit2.reason) || 'unsolved'}` : null].filter(Boolean).join(' · ');
    if (formMessageEl) formMessageEl.textContent = `Rod fit warning: ${why}`;
    return false;
  }
  return true;
}

function buildRodFitFromSpec(params, rodIndex, spec, fallbackRadius){
  const off = rodIndex === 0 ? 0 : 1;
  const length = params[off];
  const mass = params[2 + off];
  const lc = params[4 + off];
  const Iperp = params[6 + off];
  const Ispin = params[8 + off];
  const mode = (spec && spec.mode) || 'auto';
  if (mode === 'manual') {
    const metrics = rodLossMetrics(length, mass, lc, Iperp, Ispin, spec.shape || {}, fallbackRadius);
    return { ...metrics, mode: 'manual', source: 'manual' };
  }
  const autoFit = solveRodShapeFromParams(length, mass, lc, Iperp, Ispin, fallbackRadius);
  return { ...autoFit, mode: 'auto', source: 'auto' };
}

function syncVisualStateFromConfig(params, rodsSpec = null, ballSpec = null){
  const r1 = (rodsSpec && rodsSpec.rod1) ? rodsSpec.rod1 : { mode: 'auto' };
  const r2 = (rodsSpec && rodsSpec.rod2) ? rodsSpec.rod2 : { mode: 'auto' };
  state.rodSpec1 = cloneConfig(r1);
  state.rodSpec2 = cloneConfig(r2);
  state.rodFit1 = buildRodFitFromSpec(params, 0, r1, state.rodRadius1);
  state.rodFit2 = buildRodFitFromSpec(params, 1, r2, state.rodRadius2);
  state.lcRatio1 = params[0] !== 0 ? clamp(params[4] / params[0], 0.0, 1.0) : 0.5;
  state.lcRatio2 = params[1] !== 0 ? clamp(params[5] / params[1], 0.0, 1.0) : 0.5;
  const balls = (ballSpec && typeof ballSpec === 'object') ? ballSpec : DEFAULT_WEB_CONFIG.balls;
  state.ballRadiusRoot = Number(balls.rootRadius) || DEFAULT_WEB_CONFIG.balls.rootRadius;
  state.ballRadiusJoint = Number(balls.jointRadius) || DEFAULT_WEB_CONFIG.balls.jointRadius;
  state.ballRadiusTip = Number(balls.tipRadius) || DEFAULT_WEB_CONFIG.balls.tipRadius;
  updateRodFitPanel();
}

function syncRodGeometryFromParams(params, rodsSpec = null, ballSpec = null){
  syncVisualStateFromConfig(params, rodsSpec, ballSpec);
}

function updateViewAndTraceStatsFromSummary(summary){
  state.peakHeight = Number.isFinite(summary.maxZ) ? summary.maxZ : 0.0;
  state.lowHeight = Number.isFinite(summary.minZ) ? summary.minZ : 0.0;
  const verticalSpan = Math.max(0.1, state.peakHeight - state.lowHeight);
  const midHeight = 0.5 * (state.peakHeight + state.lowHeight);
  state.viewCenterZ = midHeight;
  const reachR = Math.max(summary.maxRadius || (state.l1Vis + state.l2Vis), state.l1Vis + state.l2Vis, 0.6);
  state.viewRadius = Math.max(reachR, 0.5 * verticalSpan);
  camera.target = [0, 0, state.viewCenterZ];
  const fittedPitch = clamp(0.15 + 0.12 * (verticalSpan / Math.max(state.viewRadius, 1e-6)), 0.10, 0.36);
  cameraSetPitch(fittedPitch);
  const aspectFit = Math.max((canvas.clientWidth || window.innerWidth || 1280) / Math.max((canvas.clientHeight || window.innerHeight || 720), 1), 1e-6);
  const fovyFit = 0.70;
  const fovxFit = 2.0 * Math.atan(Math.tan(fovyFit * 0.5) * aspectFit);
  const halfH = 0.5 * verticalSpan;
  const halfW = reachR;
  const fitFrac = 0.90;
  const dY = halfH / Math.max(1e-6, fitFrac * Math.tan(fovyFit * 0.5));
  const dX = halfW / Math.max(1e-6, fitFrac * Math.tan(fovxFit * 0.5));
  camera.distance = Math.max(1.4, dX, dY);
}


function makeFallbackSummary(cfg){
  const l1 = Math.max(Math.abs(cfg?.params?.[0] ?? state.l1Vis ?? 0.5), 1e-6);
  const l2 = Math.max(Math.abs(cfg?.params?.[1] ?? state.l2Vis ?? 1.0), 1e-6);
  const maxReach = l1 + l2;
  return {
    ok: false,
    reason: 'fallback',
    minZ: -maxReach,
    maxZ: maxReach,
    midZ: 0,
    maxRadius: maxReach,
    env: null,
  };
}

function estimatePotentialEnvelopeSummary(cfg){
  if (ShellModule && ShellModule.estimatePotentialEnvelopeSummary) {
    try {
      return ShellModule.estimatePotentialEnvelopeSummary(cfg);
    } catch (err) {
      console.error('[shell] estimatePotentialEnvelopeSummary failed', err);
    }
  }
  return makeFallbackSummary(cfg);
}

function resize(){

  const dpr = Math.max(1, Math.min(window.devicePixelRatio || 1, 2));
  const w = Math.floor(innerWidth * dpr);
  const h = Math.floor(innerHeight * dpr);
  if (canvas.width !== w || canvas.height !== h){
    canvas.width = w;
    canvas.height = h;
    canvas.style.width = `${innerWidth}px`;
    canvas.style.height = `${innerHeight}px`;
  }
}

window.addEventListener('resize', resize);
resize();

function getCameraBasis(){
  const q = quatNormalize(camera.orbitQuat);
  camera.orbitQuat = q;
  const forward = norm(quatRotateVec(q, [-1, 0, 0]));
  const right = norm(quatRotateVec(q, [0, 1, 0]));
  const up = norm(quatRotateVec(q, [0, 0, 1]));
  const distance = camera.distance * Math.max(Number(state.viewDistanceScale) || 1.0, 0.05);
  const eye = sub(camera.target, mul(forward, distance));
  return { eye, forward, right, up };
}


function poseFromGeometryArray(g){
  return {
    root: [g[0], g[1], g[2]],
    joint: [g[3], g[4], g[5]],
    tip: [g[6], g[7], g[8]],
    n1: [g[9], g[10], g[11]],
    u1: [g[12], g[13], g[14]],
    v1: [g[15], g[16], g[17]],
    n2: [g[18], g[19], g[20]],
    u2: [g[21], g[22], g[23]],
    v2: [g[24], g[25], g[26]],
  };
}

function poseFromFrame(frame){
  return poseFromGeometryArray(frame.g);
}


export {
  canvas, gl, statusEl, hudEl,
  ENABLE_POTENTIAL_SHELL, ShellModule, state, camera, WALLPAPER_CONFIG_URL, PERSIST_STORAGE_KEY,
  PERSIST_INTERVAL_MS, CAMERA_PITCH_LIMIT, CAMERA_DRAG_SENSITIVITY, CAMERA_RELEASE_SPEED_LIMIT, SOLVER_MODE_IDS,
  INIT_MODE_IDS, PARAM_LABELS, ROD_RADIUS_STOP, ROD_FIT_LOSS_STOP, DEFAULT_WEB_CONFIG,
  trailCount, trailAt, trailClear, trailTrimPrefix,
  v3, add, sub, mul, dot, cross, len, norm, clamp, quatNormalize, quatMul, quatFromAxisAngle, quatConjugate, quatRotateVec,
  cameraRotateOrbit, cameraSetPitch, mix, mix3, rgba, rotateAroundAxis, makeFrameFromAxis, colorRamp, colorMapSpeed1, colorMapSpeed2,
  parseWallpaperColorValue, qNormalize4, qRow3, qDisplayFrame, poseFromConfigState, quatFromRotationMatrixRows, quatFromDisplayFrame,
  normalizeAngleTriple, rodFrameFromAngles, quatFromRodAngles, paramsArrayFromJson, vector6OrZeros, normalizeOmegaTriple, angularVelocityFromJson, yStateFromAnglesPair,
  normalizeRodConfig, normalizeBallConfig, configFromWallpaperJson, fetchWallpaperConfigRecord, fetchWallpaperConfig, readPersistedSimState, clearPersistedSimState, cloneConfig,
  mat3FromBasisRows, mat3Mul, mat3Transpose, rotvecFromRotationMatrix, estimateCurrentSpeedsFromPoseDelta, positionsFromY,
  estimatePotentialEnergy, estimateTraceSpeedScales, cccMassMoments, rodLossMetrics, solveRodShapeFromParams, formatRodFit,
  updateRodFitPanel, updateSpeedUi, applySpeedValue, applyRotateValue, enforceRodFitValidity, buildRodFitFromSpec,
  syncVisualStateFromConfig, syncRodGeometryFromParams, updateViewAndTraceStatsFromSummary, makeFallbackSummary, estimatePotentialEnvelopeSummary,
  resize, getCameraBasis, poseFromGeometryArray, poseFromFrame
};
