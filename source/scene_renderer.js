import {
  state, camera, canvas, ShellModule,
  add, sub, mul, dot, cross, len, norm, clamp,
  colorMapSpeed1, colorMapSpeed2, getCameraBasis, trailAt, trailCount, trailTrimPrefix,
} from './runtime_core.js';

function pushTrail(trail, position, speed, now) {
  trail.items.push({ pos: [position[0], position[1], position[2]], speed, t: now });
  const maxAge = state.trailSeconds * 1.2;
  let dropCount = 0;
  while ((trailCount(trail) - dropCount) > 3) {
    const first = trailAt(trail, dropCount);
    if ((now - first.t) <= maxAge) break;
    dropCount += 1;
  }
  if (state.fadeMode === 'length') {
    let accumulated = 0;
    const count = trailCount(trail);
    let keepFrom = 0;
    for (let i = count - 1; i > 0; i -= 1) {
      const current = trailAt(trail, i);
      const previous = trailAt(trail, i - 1);
      accumulated += len(sub(current.pos, previous.pos));
      if (accumulated > state.trailMaxLength) {
        keepFrom = Math.max(0, i - 1);
        break;
      }
    }
    dropCount = Math.max(dropCount, keepFrom);
  }
  trailTrimPrefix(trail, dropCount);
}

function lightDirection(basis) {
  const topLight = [0, 0, 1];
  const cameraFill = norm(sub(basis.eye, camera.target));
  return norm(add(mul(topLight, 0.7), mul(cameraFill, 0.3)));
}

function mat4Perspective(fovy, aspect, near, far, shiftX = 0, shiftY = 0) {
  const f = 1.0 / Math.tan(fovy * 0.5);
  const nf = 1.0 / (near - far);
  return new Float32Array([
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    shiftX, shiftY, (far + near) * nf, -1,
    0, 0, (2 * far * near) * nf, 0,
  ]);
}

function mat4LookAt(eye, target, up) {
  const z = norm(sub(eye, target));
  let x = norm(cross(up, z));
  if (len(x) < 1e-8) x = [1, 0, 0];
  const y = norm(cross(z, x));
  return new Float32Array([
    x[0], y[0], z[0], 0,
    x[1], y[1], z[1], 0,
    x[2], y[2], z[2], 0,
    -dot(x, eye), -dot(y, eye), -dot(z, eye), 1,
  ]);
}

function mat4Multiply(a, b) {
  const out = new Float32Array(16);
  for (let c = 0; c < 4; c += 1) {
    for (let r = 0; r < 4; r += 1) {
      out[c * 4 + r] =
        a[0 * 4 + r] * b[c * 4 + 0] +
        a[1 * 4 + r] * b[c * 4 + 1] +
        a[2 * 4 + r] * b[c * 4 + 2] +
        a[3 * 4 + r] * b[c * 4 + 3];
    }
  }
  return out;
}

function makeRigidModelMatrix(origin, axisX, axisY, axisZ, scaleX = 1, scaleY = 1, scaleZ = 1) {
  return new Float32Array([
    axisX[0] * scaleX, axisX[1] * scaleX, axisX[2] * scaleX, 0,
    axisY[0] * scaleY, axisY[1] * scaleY, axisY[2] * scaleY, 0,
    axisZ[0] * scaleZ, axisZ[1] * scaleZ, axisZ[2] * scaleZ, 0,
    origin[0],          origin[1],          origin[2],          1,
  ]);
}

function makeRotation3(axisX, axisY, axisZ) {
  return new Float32Array([
    axisX[0], axisX[1], axisX[2],
    axisY[0], axisY[1], axisY[2],
    axisZ[0], axisZ[1], axisZ[2],
  ]);
}

function MeshBuilder() {
  this.positions = [];
  this.normals = [];
  this.colors = [];
  this.indices = [];
  this.count = 0;
}
MeshBuilder.prototype.vertex = function vertex(position, normal, color) {
  this.positions.push(position[0], position[1], position[2]);
  this.normals.push(normal[0], normal[1], normal[2]);
  this.colors.push(color[0], color[1], color[2], color[3]);
  return this.count += 1, this.count - 1;
};
MeshBuilder.prototype.triangle = function triangle(a, b, c) {
  this.indices.push(a, b, c);
};
MeshBuilder.prototype.quad = function quad(a, b, c, d) {
  this.indices.push(a, b, c, a, c, d);
};

function toColor4(rgb, alpha = 1.0) {
  return [rgb[0] / 255, rgb[1] / 255, rgb[2] / 255, alpha];
}

function getRodShape(length, radius, lcRatio, rodFit) {
  if (rodFit && rodFit.ok && rodFit.a >= 0 && rodFit.b >= 0 && rodFit.c >= 0) {
    return { radius: rodFit.R, a: rodFit.a, c: rodFit.c, b: rodFit.b };
  }
  const cylinderRatio = 0.46;
  const coneTotal = 1.0 - cylinderRatio;
  let asymmetry = (lcRatio - 0.5) * 0.34;
  asymmetry = clamp(asymmetry, -0.18, 0.18);
  let aRatio = coneTotal * 0.5 - asymmetry;
  let bRatio = coneTotal - aRatio;
  aRatio = clamp(aRatio, 0.10, 0.32);
  bRatio = clamp(bRatio, 0.10, 0.32);
  const correction = coneTotal / (aRatio + bRatio);
  aRatio *= correction;
  bRatio *= correction;
  const cRatio = 1.0 - aRatio - bRatio;
  return { radius, a: length * aRatio, c: length * cRatio, b: length * bRatio };
}

function buildLocalFrustum(builder, x0, x1, r0, r1, color, phiStart = 0, phiSpan = Math.PI * 2, closed = true) {
  const full = Math.PI * 2;
  const segmentCount = Math.max(10, Math.round(56 * (phiSpan / full)));
  const steps = closed ? segmentCount : segmentCount + 1;
  const ringA = [];
  const ringB = [];
  const slope = r1 - r0;
  const segmentLength = Math.max(Math.abs(x1 - x0), 1e-9);
  function makeNormal(phi) {
    const radial = [0, Math.cos(phi), Math.sin(phi)];
    return norm([-(slope / segmentLength), radial[1], radial[2]]);
  }
  if (r0 <= 1e-8 || r1 <= 1e-8) {
    const useStartApex = r0 <= 1e-8;
    const apex = [useStartApex ? x0 : x1, 0, 0];
    const baseRadius = useStartApex ? r1 : r0;
    const baseX = useStartApex ? x1 : x0;
    const apexIndex = builder.vertex(apex, [useStartApex ? -1 : 1, 0, 0], color);
    const ring = [];
    for (let i = 0; i < steps; i += 1) {
      const t = i / segmentCount;
      const phi = phiStart + t * phiSpan;
      const normal = makeNormal(phi);
      ring.push(builder.vertex([baseX, baseRadius * Math.cos(phi), baseRadius * Math.sin(phi)], normal, color));
    }
    const end = steps - (closed ? 0 : 1);
    for (let i = 0; i < end; i += 1) {
      const j = (i + 1) % steps;
      if (useStartApex) builder.triangle(apexIndex, ring[i], ring[j]);
      else builder.triangle(ring[j], ring[i], apexIndex);
    }
    return;
  }
  for (let i = 0; i < steps; i += 1) {
    const t = i / segmentCount;
    const phi = phiStart + t * phiSpan;
    const normal = makeNormal(phi);
    ringA.push(builder.vertex([x0, r0 * Math.cos(phi), r0 * Math.sin(phi)], normal, color));
    ringB.push(builder.vertex([x1, r1 * Math.cos(phi), r1 * Math.sin(phi)], normal, color));
  }
  const end = steps - (closed ? 0 : 1);
  for (let i = 0; i < end; i += 1) {
    const j = (i + 1) % steps;
    builder.quad(ringA[i], ringA[j], ringB[j], ringB[i]);
  }
}

function buildLocalRodMesh(length, radius, lcRatio, rodFit, baseRgb) {
  const builder = new MeshBuilder();
  const shape = getRodShape(length, radius, lcRatio, rodFit);
  const baseColor = toColor4(baseRgb, 1.0);
  const stripeGold = toColor4(state.barColor1, 1.16);
  const stripeBlue = toColor4(state.barColor2, 1.14);
  const x1 = shape.a;
  const x2 = shape.a + shape.c;
  buildLocalFrustum(builder, 0, x1, 0.0, shape.radius, baseColor);
  buildLocalFrustum(builder, x1, x2, shape.radius, shape.radius, baseColor);
  buildLocalFrustum(builder, x2, length, shape.radius, 0.0, baseColor);
  if (state.showBar) {
    const halfWidth = clamp(state.barStripeWidthScale * 0.18, 0.035, 0.22);
    const phiSpan = 2 * halfWidth;
    const frontPhi = state.barStripeAngle - halfWidth;
    const backPhi = state.barStripeAngle + Math.PI - halfWidth;
    buildLocalFrustum(builder, 0, x1, 0.0, shape.radius, stripeGold, frontPhi, phiSpan, false);
    buildLocalFrustum(builder, x1, x2, shape.radius, shape.radius, stripeGold, frontPhi, phiSpan, false);
    buildLocalFrustum(builder, x2, length, shape.radius, 0.0, stripeGold, frontPhi, phiSpan, false);
    buildLocalFrustum(builder, 0, x1, 0.0, shape.radius, stripeBlue, backPhi, phiSpan, false);
    buildLocalFrustum(builder, x1, x2, shape.radius, shape.radius, stripeBlue, backPhi, phiSpan, false);
    buildLocalFrustum(builder, x2, length, shape.radius, 0.0, stripeBlue, backPhi, phiSpan, false);
  }
  return builder;
}

function buildUnitSphereMesh(latitudeCount = 16, longitudeCount = 24) {
  const builder = new MeshBuilder();
  const white = [1, 1, 1, 1];
  const grid = [];
  for (let iy = 0; iy <= latitudeCount; iy += 1) {
    const v = iy / latitudeCount;
    const theta = v * Math.PI;
    const st = Math.sin(theta);
    const ct = Math.cos(theta);
    const row = [];
    for (let ix = 0; ix <= longitudeCount; ix += 1) {
      const u = ix / longitudeCount;
      const phi = u * Math.PI * 2;
      const sp = Math.sin(phi);
      const cp = Math.cos(phi);
      const normal = [st * cp, st * sp, ct];
      row.push(builder.vertex(normal, normal, white));
    }
    grid.push(row);
  }
  for (let iy = 0; iy < latitudeCount; iy += 1) {
    for (let ix = 0; ix < longitudeCount; ix += 1) {
      const a = grid[iy][ix];
      const b = grid[iy][ix + 1];
      const c = grid[iy + 1][ix + 1];
      const d = grid[iy + 1][ix];
      builder.quad(a, b, c, d);
    }
  }
  return builder;
}

function buildTrailRibbonArrays(trail, now, colorFn, eye) {
  const count = trailCount(trail);
  if (!trail || count < 2) return null;
  const positions = [];
  const colors = [];
  const baseWidth = Math.max(0.0022, state.viewRadius * 0.0038);

  function alphaAt(index) {
    const point = trailAt(trail, index);
    let alpha = 1.0;
    if (state.fadeMode === 'time') alpha = 1.0 - clamp((now - point.t) / state.trailSeconds, 0, 1);
    else alpha = 1.0 - index / Math.max(count - 1, 1);
    return Math.pow(alpha, 1.7);
  }

  for (let i = 0; i < count - 1; i += 1) {
    const p0 = trailAt(trail, i);
    const p1 = trailAt(trail, i + 1);
    const a0 = alphaAt(i);
    const a1 = alphaAt(i + 1);
    const rgb0 = colorFn(p0.speed || 0.0);
    const rgb1 = colorFn(p1.speed || 0.0);
    const t0 = i > 0 ? norm(sub(trailAt(trail, i + 1).pos, trailAt(trail, i - 1).pos)) : norm(sub(p1.pos, p0.pos));
    const t1 = i < count - 2 ? norm(sub(trailAt(trail, i + 2).pos, trailAt(trail, i).pos)) : norm(sub(p1.pos, p0.pos));
    let s0 = cross(norm(sub(eye, p0.pos)), t0);
    let s1 = cross(norm(sub(eye, p1.pos)), t1);
    if (len(s0) < 1e-8) s0 = [0, 0, 1];
    if (len(s1) < 1e-8) s1 = [0, 0, 1];
    s0 = norm(s0);
    s1 = norm(s1);
    const w0 = baseWidth * (0.55 + 0.45 * a0);
    const w1 = baseWidth * (0.55 + 0.45 * a1);
    const l0 = add(p0.pos, mul(s0, w0));
    const r0 = add(p0.pos, mul(s0, -w0));
    const l1 = add(p1.pos, mul(s1, w1));
    const r1 = add(p1.pos, mul(s1, -w1));
    const c0 = [rgb0[0] / 255, rgb0[1] / 255, rgb0[2] / 255, 0.78 * a0];
    const c1 = [rgb1[0] / 255, rgb1[1] / 255, rgb1[2] / 255, 0.78 * a1];
    positions.push(l0[0], l0[1], l0[2], r0[0], r0[1], r0[2], l1[0], l1[1], l1[2]);
    colors.push(c0[0], c0[1], c0[2], c0[3], c0[0], c0[1], c0[2], c0[3], c1[0], c1[1], c1[2], c1[3]);
    positions.push(l1[0], l1[1], l1[2], r0[0], r0[1], r0[2], r1[0], r1[1], r1[2]);
    colors.push(c1[0], c1[1], c1[2], c1[3], c0[0], c0[1], c0[2], c0[3], c1[0], c1[1], c1[2], c1[3]);
  }
  return { positions, colors, count: positions.length / 3 };
}

function createGlProgram(gl, vertexSource, fragmentSource) {
  function compile(type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      const log = gl.getShaderInfoLog(shader);
      gl.deleteShader(shader);
      throw new Error(log || 'shader compile failed');
    }
    return shader;
  }
  const program = gl.createProgram();
  const vertexShader = compile(gl.VERTEX_SHADER, vertexSource);
  const fragmentShader = compile(gl.FRAGMENT_SHADER, fragmentSource);
  gl.attachShader(program, vertexShader);
  gl.attachShader(program, fragmentShader);
  gl.linkProgram(program);
  gl.deleteShader(vertexShader);
  gl.deleteShader(fragmentShader);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    const log = gl.getProgramInfoLog(program);
    gl.deleteProgram(program);
    throw new Error(log || 'program link failed');
  }
  return program;
}

function createStaticMesh(gl, builder) {
  const mesh = {
    position: gl.createBuffer(),
    normal: gl.createBuffer(),
    color: gl.createBuffer(),
    index: gl.createBuffer(),
    count: builder.indices.length,
  };
  gl.bindBuffer(gl.ARRAY_BUFFER, mesh.position);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(builder.positions), gl.STATIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, mesh.normal);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(builder.normals), gl.STATIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, mesh.color);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(builder.colors), gl.STATIC_DRAW);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, mesh.index);
  gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(builder.indices), gl.STATIC_DRAW);
  return mesh;
}

function createSceneRenderer(gl, canvasElement) {
  const isGl2 = (typeof WebGL2RenderingContext !== 'undefined') && (gl instanceof WebGL2RenderingContext);
  let shellRenderer = null;
  if (isGl2 && ShellModule && ShellModule.createRenderer) {
    try {
      shellRenderer = ShellModule.createRenderer(gl);
    } catch (error) {
      console.error('[renderer] create shell renderer failed', error);
      shellRenderer = null;
    }
  }

  const meshProgram = createGlProgram(gl, `
    attribute vec3 aPosition;
    attribute vec3 aNormal;
    attribute vec4 aColor;
    uniform mat4 uViewProj;
    uniform mat4 uModel;
    uniform mat3 uNormalBasis;
    uniform vec4 uColorMul;
    varying vec3 vNormal;
    varying vec3 vWorld;
    varying vec4 vColor;
    void main() {
      vec4 world = uModel * vec4(aPosition, 1.0);
      vWorld = world.xyz;
      vNormal = normalize(uNormalBasis * aNormal);
      vColor = vec4(aColor.rgb * uColorMul.rgb, aColor.a * uColorMul.a);
      gl_Position = uViewProj * world;
    }
  `, `
    precision mediump float;
    varying vec3 vNormal;
    varying vec3 vWorld;
    varying vec4 vColor;
    uniform vec3 uLightDir;
    uniform vec3 uLightColor;
    uniform vec3 uHemiSky;
    uniform vec3 uHemiGround;
    uniform vec3 uRimTint;
    uniform vec3 uEye;
    void main() {
      vec3 n = normalize(vNormal);
      vec3 l = normalize(uLightDir);
      vec3 v = normalize(uEye - vWorld);
      if (dot(n, v) < 0.0) n = -n;
      vec3 h = normalize(l + v);
      float ndl = max(dot(n, l), 0.0);
      float ndv = max(dot(n, v), 0.0);
      float ndh = max(dot(n, h), 0.0);
      float hemiT = 0.5 * (clamp(n.z, -1.0, 1.0) + 1.0);
      vec3 hemi = mix(uHemiGround, uHemiSky, hemiT);
      float keyDiff = 0.54 * ndl;
      float specBroad = pow(ndh, 9.0) * 0.18;
      float specTight = pow(ndh, 28.0) * 0.08;
      float rim = pow(1.0 - ndv, 2.4) * 0.035;
      vec3 base = vColor.rgb * (hemi + uLightColor * keyDiff);
      vec3 specCol = mix(vec3(0.84, 0.86, 0.90), vColor.rgb * 1.06 + vec3(0.03), 0.18);
      float glow = max(vColor.a - 1.0, 0.0);
      vec3 color = base + specCol * (specBroad + specTight) + uRimTint * rim + vColor.rgb * (0.14 * glow);
      gl_FragColor = vec4(color, 1.0);
    }
  `);

  const trailProgram = createGlProgram(gl, `
    attribute vec3 aPosition;
    attribute vec4 aColor;
    uniform mat4 uViewProj;
    varying vec4 vColor;
    void main() {
      vColor = aColor;
      gl_Position = uViewProj * vec4(aPosition, 1.0);
    }
  `, `
    precision mediump float;
    varying vec4 vColor;
    void main() { gl_FragColor = vColor; }
  `);

  const meshLoc = {
    position: gl.getAttribLocation(meshProgram, 'aPosition'),
    normal: gl.getAttribLocation(meshProgram, 'aNormal'),
    color: gl.getAttribLocation(meshProgram, 'aColor'),
    viewProj: gl.getUniformLocation(meshProgram, 'uViewProj'),
    model: gl.getUniformLocation(meshProgram, 'uModel'),
    normalBasis: gl.getUniformLocation(meshProgram, 'uNormalBasis'),
    colorMul: gl.getUniformLocation(meshProgram, 'uColorMul'),
    light: gl.getUniformLocation(meshProgram, 'uLightDir'),
    lightColor: gl.getUniformLocation(meshProgram, 'uLightColor'),
    hemiSky: gl.getUniformLocation(meshProgram, 'uHemiSky'),
    hemiGround: gl.getUniformLocation(meshProgram, 'uHemiGround'),
    rimTint: gl.getUniformLocation(meshProgram, 'uRimTint'),
    eye: gl.getUniformLocation(meshProgram, 'uEye'),
  };
  const trailLoc = {
    position: gl.getAttribLocation(trailProgram, 'aPosition'),
    color: gl.getAttribLocation(trailProgram, 'aColor'),
    viewProj: gl.getUniformLocation(trailProgram, 'uViewProj'),
  };

  const trailBuffers = { position: gl.createBuffer(), color: gl.createBuffer() };
  const staticMeshes = {
    rod1: { key: '', mesh: null },
    rod2: { key: '', mesh: null },
    sphere: { mesh: createStaticMesh(gl, buildUnitSphereMesh()) },
  };

  function ensureRodMesh(index, length, radius, lcRatio, rodFit, rodColor) {
    const key = JSON.stringify({
      length,
      radius,
      lcRatio,
      showBar: state.showBar,
      stripeAngle: state.barStripeAngle,
      stripeWidth: state.barStripeWidthScale,
      rodColor,
      barColor1: state.barColor1,
      barColor2: state.barColor2,
      fit: rodFit ? {
        ok: !!rodFit.ok,
        a: rodFit.a,
        b: rodFit.b,
        c: rodFit.c,
        R: rodFit.R,
        loss: rodFit.loss,
        mode: rodFit.mode,
      } : null,
    });
    const cache = index === 1 ? staticMeshes.rod1 : staticMeshes.rod2;
    if (cache.mesh && cache.key === key) return cache.mesh;
    cache.key = key;
    cache.mesh = createStaticMesh(gl, buildLocalRodMesh(length, radius, lcRatio, rodFit, rodColor));
    return cache.mesh;
  }

  function bindMeshAttributes(mesh) {
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.position);
    gl.enableVertexAttribArray(meshLoc.position);
    gl.vertexAttribPointer(meshLoc.position, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.normal);
    gl.enableVertexAttribArray(meshLoc.normal);
    gl.vertexAttribPointer(meshLoc.normal, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.color);
    gl.enableVertexAttribArray(meshLoc.color);
    gl.vertexAttribPointer(meshLoc.color, 4, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, mesh.index);
  }

  function toRgb01(rgb) { return new Float32Array([(rgb[0]||0)/255, (rgb[1]||0)/255, (rgb[2]||0)/255]); }

  function drawStaticMesh(mesh, viewProj, eye, light, modelMatrix, normalBasis, colorMul = [1, 1, 1, 1]) {
    if (!mesh || !mesh.count) return;
    gl.useProgram(meshProgram);
    gl.uniformMatrix4fv(meshLoc.viewProj, false, viewProj);
    gl.uniformMatrix4fv(meshLoc.model, false, modelMatrix);
    gl.uniformMatrix3fv(meshLoc.normalBasis, false, normalBasis);
    gl.uniform4fv(meshLoc.colorMul, new Float32Array(colorMul));
    gl.uniform3fv(meshLoc.eye, new Float32Array(eye));
    gl.uniform3fv(meshLoc.light, new Float32Array(light));
    gl.uniform3fv(meshLoc.lightColor, toRgb01(state.keyLightColor));
    gl.uniform3fv(meshLoc.hemiSky, toRgb01(state.hemiSkyColor));
    gl.uniform3fv(meshLoc.hemiGround, toRgb01(state.hemiGroundColor));
    gl.uniform3fv(meshLoc.rimTint, toRgb01(state.rimLightColor));
    bindMeshAttributes(mesh);
    gl.drawElements(gl.TRIANGLES, mesh.count, gl.UNSIGNED_SHORT, 0);
  }

  function drawTrailRibbon(trailData, viewProj) {
    if (!trailData || trailData.count < 3) return;
    gl.useProgram(trailProgram);
    gl.uniformMatrix4fv(trailLoc.viewProj, false, viewProj);
    gl.bindBuffer(gl.ARRAY_BUFFER, trailBuffers.position);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(trailData.positions), gl.DYNAMIC_DRAW);
    gl.enableVertexAttribArray(trailLoc.position);
    gl.vertexAttribPointer(trailLoc.position, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, trailBuffers.color);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(trailData.colors), gl.DYNAMIC_DRAW);
    gl.enableVertexAttribArray(trailLoc.color);
    gl.vertexAttribPointer(trailLoc.color, 4, gl.FLOAT, false, 0, 0);
    gl.drawArrays(gl.TRIANGLES, 0, trailData.count);
  }

  function setShellConfig(cfg, summary) {
    if (!shellRenderer || !shellRenderer.setConfig) return;
    shellRenderer.setConfig(cfg, summary);
  }

  function drawPotentialShell(viewProj) {
    if (!shellRenderer || !shellRenderer.draw) return;
    shellRenderer.draw(viewProj);
  }

  function renderScene(pose, now, trails = null) {
    const aspect = Math.max(canvasElement.width / Math.max(canvasElement.height, 1), 1e-6);
    const projection = mat4Perspective(0.70, aspect, 0.01, 100.0, Number(state.screenOffsetX) || 0, Number(state.screenOffsetY) || 0);
    const basis = getCameraBasis();
    const view = mat4LookAt(basis.eye, camera.target, basis.up);
    const viewProj = mat4Multiply(projection, view);
    const light = lightDirection(basis);

    gl.viewport(0, 0, canvasElement.width, canvasElement.height);
    gl.clearColor(state.backgroundColor[0] / 255, state.backgroundColor[1] / 255, state.backgroundColor[2] / 255, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);
    gl.depthMask(true);
    gl.disable(gl.BLEND);
    gl.disable(gl.CULL_FACE);

    const rodMesh1 = ensureRodMesh(1, state.l1Vis, state.rodRadius1, state.lcRatio1, state.rodFit1, state.rodColor1);
    const rodMesh2 = ensureRodMesh(2, state.l2Vis, state.rodRadius2, state.lcRatio2, state.rodFit2, state.rodColor2);
    const rodModel1 = makeRigidModelMatrix(pose.root, pose.n1, pose.u1, pose.v1);
    const rodModel2 = makeRigidModelMatrix(pose.joint, pose.n2, pose.u2, pose.v2);
    const rodNormal1 = makeRotation3(pose.n1, pose.u1, pose.v1);
    const rodNormal2 = makeRotation3(pose.n2, pose.u2, pose.v2);
    drawStaticMesh(rodMesh1, viewProj, basis.eye, light, rodModel1, rodNormal1);
    drawStaticMesh(rodMesh2, viewProj, basis.eye, light, rodModel2, rodNormal2);

    const sphereMesh = staticMeshes.sphere.mesh;
    const identityNormal = new Float32Array([1,0,0,0,1,0,0,0,1]);
    drawStaticMesh(sphereMesh, viewProj, basis.eye, light, makeRigidModelMatrix(pose.root, [1,0,0], [0,1,0], [0,0,1], state.ballRadiusRoot, state.ballRadiusRoot, state.ballRadiusRoot), identityNormal, toColor4(state.ballColorRoot));
    drawStaticMesh(sphereMesh, viewProj, basis.eye, light, makeRigidModelMatrix(pose.joint, [1,0,0], [0,1,0], [0,0,1], state.ballRadiusJoint, state.ballRadiusJoint, state.ballRadiusJoint), identityNormal, toColor4(state.ballColorJoint));
    drawStaticMesh(sphereMesh, viewProj, basis.eye, light, makeRigidModelMatrix(pose.tip, [1,0,0], [0,1,0], [0,0,1], state.ballRadiusTip, state.ballRadiusTip, state.ballRadiusTip), identityNormal, toColor4(state.ballColorTip));

    if (state.showPotentialShell && shellRenderer) {
      try {
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        gl.depthMask(false);
        drawPotentialShell(viewProj);
      } catch (error) {
        console.error('[renderer] draw shell failed', error);
        state.showPotentialShell = false;
      } finally {
        gl.depthMask(true);
        gl.disable(gl.BLEND);
      }
    }

    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.depthMask(false);
    const trailOne = trails && trails.trail1 ? trails.trail1 : null;
    const trailTwo = trails && trails.trail2 ? trails.trail2 : null;
    drawTrailRibbon(buildTrailRibbonArrays(trailOne, now, colorMapSpeed1, basis.eye), viewProj);
    drawTrailRibbon(buildTrailRibbonArrays(trailTwo, now, colorMapSpeed2, basis.eye), viewProj);
    gl.depthMask(true);
  }

  return { renderScene, setShellConfig };
}

export { createSceneRenderer, pushTrail };
