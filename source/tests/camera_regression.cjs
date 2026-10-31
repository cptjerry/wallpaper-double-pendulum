// Run with Node.js: node tests/camera_regression.cjs [project-directory]
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(process.argv[2] || path.join(__dirname, '..'));
const canvas = { width: 0, height: 0, style: {}, getContext: () => ({}) };
const sandbox = {
  console, innerWidth: 1920, innerHeight: 1080,
  window: { devicePixelRatio: 1, addEventListener() {} },
  document: { getElementById: id => id === 'stage' ? canvas : { style: {} } },
};
vm.createContext(sandbox);
const evaluate = code => vm.runInContext(code, sandbox);
const runtime = fs.readFileSync(path.join(root, 'runtime_core.js'), 'utf8');
evaluate(runtime.replace(/export\s*\{[\s\S]*?\};?\s*$/, ''));
const main = fs.readFileSync(path.join(root, 'main.js'), 'utf8');
evaluate(main.slice(main.indexOf('function restoreCameraFromSavedState('), main.indexOf('function syncVisualStateFromConfigState(')));
// Angles below are test fixtures/measurements, never stored camera state.
evaluate(`
function fixtureOrientation(heading, inclination) {
  camera.orbitQuat = quatNormalize(quatMul(
    quatFromAxisAngle([0, 0, 1], heading),
    quatFromAxisAngle([0, 1, 0], -inclination)
  ));
}
function rotateForTest(h, v) {
  if (typeof cameraRotateOrbit === 'function') cameraRotateOrbit(h, v);
  else { const o = cameraGetYawPitch(); cameraSetOrbitFromYawPitch(o.yaw + h, clamp(o.pitch + v, -CAMERA_PITCH_LIMIT, CAMERA_PITCH_LIMIT)); }
}
function pitchForTest() {
  const f = getCameraBasis().forward;
  return Math.atan2(-f[2], Math.hypot(f[0], f[1]));
}
`);
const angleError = (a, b) => Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
const heading = () => evaluate(`(() => {const r = getCameraBasis().right; return Math.atan2(-r[0], r[1]);})()`);
let checks = 0;
for (const pitch of [-Math.PI / 2, -Math.PI / 2 + 1e-10, 0, Math.PI / 2 - 1e-10, Math.PI / 2]) {
  for (const delta of [-0.008, 0.008]) {
    evaluate(`fixtureOrientation(0.7, ${pitch})`);
    for (let i = 1; i <= 2000; i++) {
      evaluate(`rotateForTest(${delta}, 0)`);
      assert(angleError(heading(), 0.7 + i * delta) < 1e-10, `heading jump at pitch=${pitch}, frame=${i}`);
      assert(Math.abs(evaluate('pitchForTest()') - pitch) < 1e-10, 'inclination drift');
      checks++;
    }
    const before = heading();
    evaluate(`rotateForTest(0, ${pitch > 0 ? -0.01 : 0.01})`);
    assert(angleError(heading(), before) < 1e-10, 'heading jumped when leaving pole');
  }
}
// Dragging beyond a pole clamps there, then immediately responds inward.
for (const sign of [-1, 1]) {
  evaluate(`fixtureOrientation(1.2, ${sign} * (Math.PI / 2 - 0.02)); rotateForTest(0, ${sign} * 0.2)`);
  assert(Math.abs(evaluate('pitchForTest()') - sign * Math.PI / 2) < 1e-12);
  for (let i = 0; i < 100; i++) evaluate(`rotateForTest(0.02, ${sign} * 0.1)`);
  assert(Math.abs(evaluate('pitchForTest()') - sign * Math.PI / 2) < 1e-12);
  const before = heading();
  evaluate(`rotateForTest(0, ${-sign} * 0.01)`);
  assert(angleError(heading(), before) < 1e-10);
  assert(Math.abs(evaluate('pitchForTest()') - sign * (Math.PI / 2 - 0.01)) < 1e-10);
}
// Quaternion-only saves, both q/-q signs, with or without obsolete angle fields.
for (const pitch of [-Math.PI / 2, -Math.PI / 2 + 1e-10, 0.42, Math.PI / 2 - 1e-10, Math.PI / 2]) {
  for (const yaw of [-2.8, -1.4, 0, 0.7, 2.3]) {
    for (const sign of [-1, 1]) {
      for (const extra of ['', 'orbitYaw: null, orbitPitch: null,', 'orbitYaw: 0, orbitPitch: 0,']) {
        sandbox.savedRecord = evaluate(`(() => {
          fixtureOrientation(${yaw}, ${pitch});
          return {orbitQuat: camera.orbitQuat.map(v => v * ${sign}), target: [0,0,0], distance: 3.2, ${extra} autoRotateZ: 0.2};
        })()`);
        evaluate('fixtureOrientation(-0.5, 0.1); restoreCameraFromSavedState({camera: normalizePersistCamera(savedRecord)})');
        assert(angleError(heading(), yaw) < 1e-10, 'restored heading changed');
        assert(Math.abs(evaluate('pitchForTest()') - pitch) < 1e-10, 'restored inclination changed');
        evaluate('rotateForTest(0.001, 0)');
        assert(angleError(heading(), yaw + 0.001) < 1e-10, 'first frame after restore jumped');
        checks++;
      }
    }
  }
}
// Large input and absolute-angle controls use only quaternion constraints.
for (let i = 0; i < 2000; i++) {
  const p = -Math.PI / 2 + (i % 101) * Math.PI / 100;
  const d = Math.sin(i * 1.23) * 100;
  evaluate(`fixtureOrientation(0.7, ${p}); cameraRotateOrbit(0, ${d})`);
  assert(Math.abs(evaluate('pitchForTest()') - Math.max(-Math.PI / 2, Math.min(Math.PI / 2, p + d))) < 1e-10, 'large input limit failed');
  const target = Math.sin(i * 0.45) * Math.PI / 2;
  evaluate(`fixtureOrientation(1.2, ${p}); cameraSetPitch(${target})`);
  assert(Math.abs(evaluate('pitchForTest()') - target) < 1e-10, 'absolute pitch input failed');
}
for (const sign of [-1, 1]) {
  evaluate(`fixtureOrientation(1.2, ${sign} * (Math.PI / 2 - 1e-8)); cameraSetPitch(${-sign} * (Math.PI / 2 - 2e-8))`);
  assert(Math.abs(evaluate('pitchForTest()') + sign * (Math.PI / 2 - 2e-8)) < 1e-10);
}
const persist = fs.readFileSync(path.join(root, 'persist_state.js'), 'utf8');
evaluate(persist.slice(persist.indexOf('function serializeCameraState('), persist.indexOf('function maybePersistSimulationState(')));
const saved = evaluate('serializeCameraState(camera, state)');
assert(!('orbitYaw' in saved) && !('orbitPitch' in saved), 'new save still stores Euler angles');
assert(!('orbitYaw' in evaluate('camera')) && !('orbitPitch' in evaluate('camera')), 'camera still stores Euler angles');
// Exercise the actual main.js fixed-angle and gamepad control paths.
evaluate(main.slice(main.indexOf('const CONTROL_MODE ='), main.indexOf('function isHudVisible(')));
evaluate('let testAxes = [0, 0]; function getActiveGamepadAxes() { return testAxes; }');
evaluate(main.slice(main.indexOf('function applyControlState('), main.indexOf('function noteMessage(')));
for (const sign of [-1, 1]) {
  evaluate(`fixtureOrientation(0.7, 0.2); state.controlMode = CONTROL_MODE.NONE; state.controlPitch = ${sign} * Math.PI / 2; state.controlRotateSpeed = 0.2`);
  for (let i = 1; i <= 1000; i++) {
    evaluate('applyControlState(0.01); cameraRotateOrbit(state.autoRotateZ * 0.01, 0)');
    assert(angleError(heading(), 0.7 + i * 0.002) < 1e-10, 'fixed-angle control jumped at pole');
    assert(Math.abs(evaluate('pitchForTest()') - sign * Math.PI / 2) < 1e-10);
  }
  const before = heading();
  evaluate(`state.controlMode = CONTROL_MODE.GAMEPAD; testAxes = [0, ${sign}]; applyControlState(0.01)`);
  assert(angleError(heading(), before) < 1e-10, 'gamepad changed heading while leaving pole');
  assert(Math.abs(evaluate('pitchForTest()') - sign * (Math.PI / 2 - 0.0115)) < 1e-10);
}
// Repeated mixed-axis input must retain an orthonormal camera basis.
for (let i = 0; i < 5000; i++) evaluate(`rotateForTest(0.003, ${Math.sin(i * 0.02) * 0.002})`);
const basis = evaluate('getCameraBasis()');
for (const v of [basis.forward, basis.right, basis.up]) assert(Math.abs(Math.hypot(...v) - 1) < 1e-12);
const dot = (a,b) => a.reduce((sum,x,i) => sum + x*b[i],0);
assert(Math.abs(dot(basis.forward,basis.right)) < 1e-12);
assert(Math.abs(dot(basis.forward,basis.up)) < 1e-12);
assert(Math.abs(dot(basis.right,basis.up)) < 1e-12);
console.log(`PASS: ${checks} rotation/restore checks; 2000 fixed-angle frames; gamepad pole departure; quaternion-only saves; 5000 mixed input steps.`);
