(function(){"use strict";
(function(global){
  function clamp(x,a,b){ return Math.min(b, Math.max(a, x)); }
  function qNormalize(q){ const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0]/n, q[1]/n, q[2]/n, q[3]/n]; }
  function qRow3(q){ const qq=qNormalize(q); const w=qq[0],x=qq[1],y=qq[2],z=qq[3]; return [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]; }
  function positionsFromY(y, params){
    const l1=Math.max(params[0],1e-9), l2=Math.max(params[1],1e-9);
    const n1=qRow3(y.slice(0,4)); const n2=qRow3(y.slice(4,8));
    const joint=[l1*n1[0], l1*n1[1], l1*n1[2]];
    const tip=[joint[0]+l2*n2[0], joint[1]+l2*n2[1], joint[2]+l2*n2[2]];
    return {n1,n2,joint,tip};
  }
  function estimatePotentialEnergy(y, params){
    const l1=params[0], m1=params[2], m2=params[3], lc1=params[4], lc2=params[5], g=Math.abs(params[10]);
    const {n1,n2}=positionsFromY(y, params);
    return -g*(m1*lc1*n1[2] + m2*(l1*n1[2] + lc2*n2[2]));
  }
  function estimateInitialKineticRough(cfg){
    const params=cfg.params;
    const l1=Math.max(params[0],1e-9), l2=Math.max(params[1],1e-9);
    const m1=Math.max(params[2],1e-9), m2=Math.max(params[3],1e-9);
    const lc1=Math.max(Math.abs(params[4]),1e-9), lc2=Math.max(Math.abs(params[5]),1e-9);
    const Iperp1=Math.max(params[6],0.0), Iperp2=Math.max(params[7],0.0);
    const dt=Math.max(cfg.simDt || (1/240), 1e-6);
    if (cfg.initMode === 'pair' && Array.isArray(cfg.yPrev) && cfg.yPrev.length >= 8){
      const p0=positionsFromY(cfg.yPrev, params), p1=positionsFromY(cfg.yCur, params);
      const dx1=Math.hypot(p1.joint[0]-p0.joint[0], p1.joint[1]-p0.joint[1], p1.joint[2]-p0.joint[2]);
      const dx2=Math.hypot(p1.tip[0]-p0.tip[0], p1.tip[1]-p0.tip[1], p1.tip[2]-p0.tip[2]);
      const s1=dx1/dt, s2=dx2/dt;
      return 0.5*(m1+m2)*s1*s1 + 0.5*m2*s2*s2;
    }
    if (Array.isArray(cfg.angularVelocity) && cfg.angularVelocity.length >= 6){
      const w1=Math.hypot(cfg.angularVelocity[0],cfg.angularVelocity[1],cfg.angularVelocity[2]);
      const w2=Math.hypot(cfg.angularVelocity[3],cfg.angularVelocity[4],cfg.angularVelocity[5]);
      const s1=l1*w1;
      const s2=Math.hypot(s1, l2*w2);
      return 0.5*(m1+m2)*s1*s1 + 0.5*m2*s2*s2;
    }
    const p = cfg.initMode === 'right' ? cfg.pRight : cfg.pLeft;
    if (Array.isArray(p) && p.length >= 6){
      const p1=Math.hypot(p[0],p[1],p[2]);
      const p2=Math.hypot(p[3],p[4],p[5]);
      const I1eff=Math.max(Iperp1 + m1*lc1*lc1, 1e-8);
      const I2eff=Math.max(Iperp2 + m2*lc2*lc2, 1e-8);
      const w1=p1/I1eff, w2=p2/I2eff;
      const s1=l1*w1;
      const s2=Math.hypot(s1, l2*w2);
      return 0.5*(m1+m2)*s1*s1 + 0.5*m2*s2*s2;
    }
    return 0.0;
  }
  function potentialEnvelopeInfo(cfg){
    const l1=Math.max(cfg.params[0],1e-9), l2=Math.max(cfg.params[1],1e-9);
    const m1=Math.max(cfg.params[2],1e-9), m2=Math.max(cfg.params[3],1e-9);
    const lc1=Math.max(Math.abs(cfg.params[4]),1e-9), lc2=Math.max(Math.abs(cfg.params[5]),1e-9);
    const g=Math.abs(cfg.params[10]) || 9.81;
    const n1=qRow3(cfg.yCur.slice(0,4)); const n2=qRow3(cfg.yCur.slice(4,8));
    const a=m1*lc1 + m2*l1, b=m2*lc2;
    const s0=a*n1[2] + b*n2[2];
    const U0=-g*s0;
    const K0=Math.max(0, estimateInitialKineticRough(cfg));
    const E0=U0+K0;
    const threshold = -E0 / g;
    return { l1,l2,m1,m2,lc1,lc2,a,b,g,U0,K0,E0,threshold, currentS:s0 };
  }
  function feasibleXY(env,x,y){ return x>=-1-1e-9 && x<=1+1e-9 && y>=-1-1e-9 && y<=1+1e-9 && env.a*x + env.b*y >= env.threshold - 1e-9; }
  function polygonVertices(env){
    const pts=[];
    const add=(x,y)=>{
      if(!Number.isFinite(x)||!Number.isFinite(y)) return;
      if(!feasibleXY(env,x,y)) return;
      x=clamp(x,-1,1); y=clamp(y,-1,1);
      for(const p of pts) if(Math.abs(p[0]-x)<1e-7&&Math.abs(p[1]-y)<1e-7) return;
      pts.push([x,y]);
    };
    [[-1,-1],[-1,1],[1,-1],[1,1]].forEach(([x,y])=>add(x,y));
    if(env.b>1e-12){ add(-1,(env.threshold+env.a)/env.b); add(1,(env.threshold-env.a)/env.b); }
    if(env.a>1e-12){ add((env.threshold+env.b)/env.a,-1); add((env.threshold-env.b)/env.a,1); }
    return pts;
  }
  function zRangeFromEnv(env){
    const verts=polygonVertices(env);
    let minZ=Infinity,maxZ=-Infinity;
    if (env.a > 1e-12){
      const xMin = clamp((env.threshold - env.b) / env.a, -1, 1);
      minZ = Math.min(minZ, -env.l1);
      maxZ = Math.max(maxZ, -env.l1 * xMin);
    }
    for(const [x,y] of verts){
      const z2 = -(env.l1*x + env.l2*y);
      minZ = Math.min(minZ, z2);
      maxZ = Math.max(maxZ, z2);
    }
    if(!Number.isFinite(minZ)||!Number.isFinite(maxZ)){
      const total=env.l1+env.l2;
      return {minZ:-total, maxZ:total};
    }
    return {minZ,maxZ};
  }
  function makeFallbackSummary(cfg){
    const l1 = Math.max(Math.abs(cfg?.params?.[0] ?? 0.5), 1e-6);
    const l2 = Math.max(Math.abs(cfg?.params?.[1] ?? 1.0), 1e-6);
    const maxReach = l1 + l2;
    return { ok:false, reason:'fallback', minZ:-maxReach, maxZ:maxReach, midZ:0, maxRadius:maxReach, env:null };
  }
  function estimatePotentialEnvelopeSummary(cfg){
    try {
      const env = potentialEnvelopeInfo(cfg);
      const zr = zRangeFromEnv(env);
      const maxRadius = Math.hypot(env.l1 + env.l2, Math.max(Math.abs(zr.minZ), Math.abs(zr.maxZ)));
      return { ok:true, minZ:zr.minZ, maxZ:zr.maxZ, midZ:0.5*(zr.minZ+zr.maxZ), maxRadius, env };
    } catch(err){
      console.error('[PotentialShell] estimate summary failed', err);
      return makeFallbackSummary(cfg);
    }
  }
  function compile(gl, type, src){ const sh=gl.createShader(type); gl.shaderSource(sh, src); gl.compileShader(sh); if(!gl.getShaderParameter(sh, gl.COMPILE_STATUS)){ throw new Error(gl.getShaderInfoLog(sh)||'shader compile failed'); } return sh; }
  function link(gl, vsSrc, fsSrc, varyings){ const vs=compile(gl, gl.VERTEX_SHADER, vsSrc); const fs=compile(gl, gl.FRAGMENT_SHADER, fsSrc); const p=gl.createProgram(); gl.attachShader(p,vs); gl.attachShader(p,fs); if(varyings){ gl.transformFeedbackVaryings(p, varyings, gl.INTERLEAVED_ATTRIBS); } gl.linkProgram(p); if(!gl.getProgramParameter(p, gl.LINK_STATUS)){ throw new Error(gl.getProgramInfoLog(p)||'program link failed'); } return p; }


  function createRenderer(gl){
    const isGL2 = (typeof WebGL2RenderingContext !== 'undefined') && (gl instanceof WebGL2RenderingContext);
    if(!isGL2){ return { supported:false, setConfig(){}, clear(){}, draw(){}, info(){ return {gpuMultipass:false,supported:false}; } }; }

    let resources = null;
    let grid = null;
    let gridKey = '';

    function ensureResources(){
      if (resources) return resources;
      const computeVS = `#version 300 es
        precision highp float;
        in float aZ;
        uniform float uL1,uL2,uA,uB,uThreshold;
        out vec3 vProfile;
        bool feasible(float x,float y){
          return x>=-1.0 && x<=1.0 && y>=-1.0 && y<=1.0 && (uA*x + uB*y >= uThreshold);
        }
        float jointR(float z){
          float x = -z / max(uL1, 1e-6);
          if (x < -1.0 || x > 1.0) return -1.0;
          if (uB > 1e-7) {
            if (uA * x + uB < uThreshold) return -1.0;
          } else if (uA * x < uThreshold) return -1.0;
          return uL1 * sqrt(max(0.0, 1.0 - x*x));
        }
        bool tipRange(float z, out float xLo, out float xHi){
          xLo = max(-1.0, (-uL2 - z)/max(uL1,1e-6));
          xHi = min( 1.0, ( uL2 - z)/max(uL1,1e-6));
          float k = uA - uB*uL1/max(uL2,1e-6);
          float rhs = uThreshold + uB*z/max(uL2,1e-6);
          if (abs(k) < 1e-7){ if (rhs > 0.0) return false; }
          else if (k > 0.0) xLo = max(xLo, rhs/k);
          else xHi = min(xHi, rhs/k);
          return xLo <= xHi;
        }
        float tipRho(float z, float x){
          float y = (-z - uL1*x)/max(uL2,1e-6);
          if (!feasible(x,y)) return -1.0;
          return uL1*sqrt(max(0.0, 1.0 - x*x)) + uL2*sqrt(max(0.0, 1.0 - y*y));
        }
        void main(){
          float z = aZ;
          float rj = jointR(z);
          float xLo, xHi;
          float rtOut = -1.0;
          if (tipRange(z, xLo, xHi)) {
            float bestX = xLo;
            for(int i=0;i<160;i++){
              float t = float(i) / 159.0;
              float x = mix(xLo, xHi, t);
              float r = tipRho(z, x);
              if (r >= 0.0 && r > rtOut) { rtOut = r; bestX = x; }
            }
            float span = max(1e-4, (xHi - xLo) / 32.0);
            float lo2 = max(xLo, bestX - span);
            float hi2 = min(xHi, bestX + span);
            for(int j=0;j<96;j++){
              float t = float(j) / 95.0;
              float x = mix(lo2, hi2, t);
              float r = tipRho(z, x);
              if (r >= 0.0) rtOut = max(rtOut, r);
            }
          }
          vProfile = vec3(z, rj, rtOut);
          gl_Position = vec4(0.0);
        }
      `;
      const passthroughFS = `#version 300 es
        precision mediump float;
        out vec4 outColor;
        void main(){ outColor = vec4(0.0); }
      `;
      const computeProgram = link(gl, computeVS, passthroughFS, ['vProfile']);
      const computeLoc = {
        aZ: gl.getAttribLocation(computeProgram,'aZ'),
        l1: gl.getUniformLocation(computeProgram,'uL1'), l2: gl.getUniformLocation(computeProgram,'uL2'),
        a: gl.getUniformLocation(computeProgram,'uA'), b: gl.getUniformLocation(computeProgram,'uB'), th: gl.getUniformLocation(computeProgram,'uThreshold'),
      };
      const drawVS = `#version 300 es
        precision highp float;
        in vec3 aPosition;
        uniform mat4 uViewProj;
        void main(){ gl_Position = uViewProj * vec4(aPosition, 1.0); }
      `;
      const drawFS = `#version 300 es
        precision mediump float;
        uniform vec4 uColor;
        out vec4 outColor;
        void main(){ outColor = uColor; }
      `;
      const drawProgram = link(gl, drawVS, drawFS);
      resources = {
        computeProgram,
        computeLoc,
        zBuf: gl.createBuffer(),
        tfBuf: gl.createBuffer(),
        tf: gl.createTransformFeedback(),
        drawProgram,
        drawLoc: { pos: gl.getAttribLocation(drawProgram,'aPosition'), vp: gl.getUniformLocation(drawProgram,'uViewProj'), color: gl.getUniformLocation(drawProgram,'uColor') },
        drawBuf: gl.createBuffer(),
      };
      return resources;
    }

    function summaryKey(summary){
      if(!summary || !summary.env) return '';
      const e = summary.env;
      return [summary.minZ, summary.maxZ, e.l1, e.l2, e.a, e.b, e.threshold].map((v) => Number(v).toPrecision(7)).join('|');
    }

    function computeProfiles(summary){
      const r = ensureResources();
      const env = summary.env;
      const N = 320;
      const zs = new Float32Array(N);
      for(let i=0;i<N;i++) zs[i] = summary.minZ + (summary.maxZ - summary.minZ) * i / (N - 1);
      gl.useProgram(r.computeProgram);
      gl.uniform1f(r.computeLoc.l1, env.l1);
      gl.uniform1f(r.computeLoc.l2, env.l2);
      gl.uniform1f(r.computeLoc.a, env.a);
      gl.uniform1f(r.computeLoc.b, env.b);
      gl.uniform1f(r.computeLoc.th, env.threshold);
      gl.bindBuffer(gl.ARRAY_BUFFER, r.zBuf);
      gl.bufferData(gl.ARRAY_BUFFER, zs, gl.STATIC_DRAW);
      gl.enableVertexAttribArray(r.computeLoc.aZ);
      gl.vertexAttribPointer(r.computeLoc.aZ, 1, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.TRANSFORM_FEEDBACK_BUFFER, r.tfBuf);
      gl.bufferData(gl.TRANSFORM_FEEDBACK_BUFFER, N * 3 * 4, gl.DYNAMIC_COPY);
      gl.bindTransformFeedback(gl.TRANSFORM_FEEDBACK, r.tf);
      gl.bindBufferBase(gl.TRANSFORM_FEEDBACK_BUFFER, 0, r.tfBuf);
      gl.enable(gl.RASTERIZER_DISCARD);
      gl.beginTransformFeedback(gl.POINTS);
      gl.drawArrays(gl.POINTS, 0, N);
      gl.endTransformFeedback();
      gl.disable(gl.RASTERIZER_DISCARD);
      const out = new Float32Array(N * 3);
      gl.getBufferSubData(gl.TRANSFORM_FEEDBACK_BUFFER, 0, out);
      return out;
    }

    function buildGridFromProfiles(summary, prof){
      const positions=[]; const strips=[];
      const pushStrip=(pts)=>{ if(!pts||pts.length<2) return; const first=positions.length/3; for(const p of pts) positions.push(p[0],p[1],p[2]); strips.push({first,count:pts.length}); };
      const rows=[];
      for(let i=0;i<prof.length;i+=3){
        const z=prof[i], rj=prof[i+1], rtOut=prof[i+2];
        const outer = Math.max(rj>1e-6?rj:0, rtOut>1e-6?rtOut:0);
        if (outer > 1e-6) rows.push({ z, outer });
      }
      if(rows.length < 2) return { positions:new Float32Array(), strips:[] };
      const EPS=1e-5, N_SEG=120, N_LON=16, LAT_SKIP=12;
      const rowIndices=[];
      for(let i=0;i<rows.length;i++) if(i===0 || i===rows.length-1 || (i % LAT_SKIP)===0) rowIndices.push(i);
      function addRing(radius,z){
        if(!(radius>EPS)) return;
        const ring=[];
        for(let i=0;i<=N_SEG;i++){
          const phi = 2*Math.PI*i/N_SEG;
          ring.push([radius*Math.cos(phi), radius*Math.sin(phi), z]);
        }
        pushStrip(ring);
      }
      function addMeridians(){
        for(let ip=0; ip<N_LON; ++ip){
          const phi = 2*Math.PI*ip/N_LON;
          const line=[];
          for(const r of rows){
            const rho = r.outer;
            if(rho>EPS) line.push([rho*Math.cos(phi), rho*Math.sin(phi), r.z]);
          }
          pushStrip(line);
        }
      }
      for(const idx of rowIndices) addRing(rows[idx].outer, rows[idx].z);
      addMeridians();
      const top = rows[rows.length - 1];
      const topApex = [0,0,summary.maxZ];
      if (top.outer > EPS){
        for(let ip=0; ip<N_LON; ++ip){
          const phi = 2*Math.PI*ip/N_LON;
          const p = [top.outer*Math.cos(phi), top.outer*Math.sin(phi), top.z];
          pushStrip([p, topApex]);
        }
      }
      const bot = rows[0];
      const botApex = [0,0,summary.minZ];
      if (bot.outer > EPS){
        for(let ip=0; ip<N_LON; ++ip){
          const phi = 2*Math.PI*ip/N_LON;
          const p = [bot.outer*Math.cos(phi), bot.outer*Math.sin(phi), bot.z];
          pushStrip([p, botApex]);
        }
      }
      return { positions:new Float32Array(positions), strips };
    }

    function setConfig(cfg, summary){
      if(!summary || !summary.env){ grid = null; gridKey = ''; return; }
      const nextKey = summaryKey(summary);
      if (nextKey && nextKey === gridKey && grid) return;
      const prof = computeProfiles(summary);
      grid = buildGridFromProfiles(summary, prof);
      gridKey = nextKey;
      const r = ensureResources();
      gl.bindBuffer(gl.ARRAY_BUFFER, r.drawBuf);
      gl.bufferData(gl.ARRAY_BUFFER, grid.positions, gl.STATIC_DRAW);
      gl.bindBuffer(gl.ARRAY_BUFFER, null);
    }
    function clear(){ grid = null; gridKey = ''; }
    function draw(viewProj){
      if(!grid || !grid.strips || !grid.strips.length) return;
      const r = ensureResources();
      gl.useProgram(r.drawProgram);
      gl.uniformMatrix4fv(r.drawLoc.vp, false, viewProj);
      gl.uniform4f(r.drawLoc.color, 0.60, 0.64, 0.68, 0.22);
      gl.bindBuffer(gl.ARRAY_BUFFER, r.drawBuf);
      gl.enableVertexAttribArray(r.drawLoc.pos);
      gl.vertexAttribPointer(r.drawLoc.pos, 3, gl.FLOAT, false, 0, 0);
      for(const strip of grid.strips) gl.drawArrays(gl.LINE_STRIP, strip.first, strip.count);
      gl.bindBuffer(gl.ARRAY_BUFFER, null);
    }
    function info(){ return { gpuMultipass:true, supported:true }; }
    return { supported:true, setConfig, draw, clear, info };
  }

  function info(){ return { gpuMultipass:true }; }
  global.PotentialShell = { estimatePotentialEnvelopeSummary, makeFallbackSummary, createRenderer, info };
})(window);

const __modules=Object.create(null), __instances=Object.create(null);
__modules["we_property_bridge.js"] = function(exports, __require){
// Classic script for early WE callbacks; also safe as a main.js side-effect import.
(function (global) {
  if (global.__MCLD_WE_BRIDGE__) return;
  const keys = { general: 'mcld_we_general_props_cache', user: 'mcld_we_user_props_cache' };
  const methods = { general: 'applyGeneralProperties', user: 'applyUserProperties' };
  function read(kind) {
    try {
      const parsed = JSON.parse(global.localStorage.getItem(keys[kind]));
      return parsed && typeof parsed === 'object' && !Array.isArray(parsed) ? parsed : {};
    } catch (_) { return {}; }
  }
  const values = { general: read('general'), user: read('user') };
  let receiver = null;
  function accept(kind, patch) {
    if (!patch || typeof patch !== 'object' || Array.isArray(patch)) return;
    // With a receiver, persist after application succeeds. During bootstrap,
    // retain callbacks immediately so decoding/loading cannot lose them.
    if (receiver) receiver[methods[kind]](patch);
    values[kind] = { ...values[kind], ...patch };
    try { global.localStorage.setItem(keys[kind], JSON.stringify(values[kind])); } catch (_) {}
  }
  const listener = {
    applyGeneralProperties(properties) { accept('general', properties); },
    applyUserProperties(properties) { accept('user', properties); },
  };
  global.__MCLD_WE_BRIDGE__ = {
    listener,
    connect(next) {
      if (!next || typeof next.applyGeneralProperties !== 'function' || typeof next.applyUserProperties !== 'function') {
        throw new Error('Invalid WE property receiver');
      }
      receiver = next;
      if (Object.keys(values.general).length) receiver.applyGeneralProperties({ ...values.general });
      if (Object.keys(values.user).length) receiver.applyUserProperties({ ...values.user });
    },
  };
  global.wallpaperPropertyListener = listener;
})(window);

};
__modules["runtime_core.js"] = function(exports, __require){
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


Object.assign(exports, {
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
});

};
__modules["scene_renderer.js"] = function(exports, __require){
const {
  state, camera, canvas, ShellModule,
  add, sub, mul, dot, cross, len, norm, clamp,
  colorMapSpeed1, colorMapSpeed2, getCameraBasis, trailAt, trailCount, trailTrimPrefix,
} = __require("runtime_core.js");

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

Object.assign(exports, { createSceneRenderer, pushTrail });

};
__modules["worker_stream.js"] = function(exports, __require){
class WorkerStream {
  constructor({ state, onStatusNote }) {
    this.state = state;
    this.onStatusNote = typeof onStatusNote === 'function' ? onStatusNote : () => {};
    this.client = null;
    this.ready = false;
    this.status = '';
    this.fillRequested = false;
    this.pendingApply = null;
    this.readyPromise = null;
    this.resolveReady = null;
    this.rejectReady = null;
    this.queueItems = [];
    this.queueStart = 0;
  }

  hasClient() { return !!this.client; }
  isReady() { return !!this.ready; }

  computeBufferConfig() {
    const simDt = Math.max(Number(this.state?.simDt) || (1 / 240), 1e-6);
    const playSpeed = Math.max(Number(this.state?.renderSpeed) || 0, 0);
    if (!(playSpeed > 0)) {
      return { lowWater: 96, highWater: 192, chunkSize: 12 };
    }
    const rate = playSpeed / simDt;
    const lowWater = Math.max(96, Math.min(768, Math.ceil(rate * 0.40)));
    const highWater = Math.max(lowWater + 24, Math.min(1536, Math.ceil(rate * 0.85)));
    const chunkSize = Math.max(8, Math.min(24, Math.ceil(rate * 0.06)));
    return { lowWater, highWater, chunkSize };
  }

  lowWatermark() {
    return this.computeBufferConfig().lowWater;
  }
  queueLength() { return this.queueItems.length - this.queueStart; }
  queuePeek() { return this.queueStart < this.queueItems.length ? this.queueItems[this.queueStart] : null; }
  queueClear() { this.queueItems.length = 0; this.queueStart = 0; this.fillRequested = false; }
  queuePushMany(frames) { if (Array.isArray(frames) && frames.length) this.queueItems.push(...frames); }
  queueShift() {
    if (this.queueStart >= this.queueItems.length) return null;
    const value = this.queueItems[this.queueStart++];
    if (this.queueStart > 64 && this.queueStart * 2 >= this.queueItems.length) {
      this.queueItems = this.queueItems.slice(this.queueStart);
      this.queueStart = 0;
    }
    return value;
  }

  _handleMessage = (event) => {
    const data = event.data || {};
    if (data.type === 'ready') {
      this.ready = true;
      this.state.useWasm = true;
      this.status = 'Worker ready';
      if (this.resolveReady) { this.resolveReady(true); this.resolveReady = this.rejectReady = null; }
      return;
    }
    if (data.type === 'unavailable') {
      this.ready = false;
      this.state.useWasm = false;
      this.status = `Worker unavailable: ${data.message || 'unknown'}`;
      this.onStatusNote(this.status);
      if (this.resolveReady) { this.resolveReady(false); this.resolveReady = this.rejectReady = null; }
      return;
    }
    if (data.type === 'frames') {
      this.fillRequested = false;
      if (this.state.initializing) return;
      if (data.generation != null && data.generation !== this.state.workerGeneration) return;
      this.queuePushMany(data.frames);
      return;
    }
    if (data.type === 'applied') {
      this.fillRequested = false;
      if (this.pendingApply) {
        this.pendingApply.resolve(data);
        this.pendingApply = null;
      }
      return;
    }
    if (data.type === 'apply_error') {
      this.fillRequested = false;
      if (this.pendingApply) {
        this.pendingApply.reject(new Error(data.message || 'worker apply failed'));
        this.pendingApply = null;
      }
      return;
    }
    if (data.type === 'status') this.status = data.message || '';
  };

  start(simDt) {
    this.readyPromise = new Promise((resolve, reject) => { this.resolveReady = resolve; this.rejectReady = reject; });
    if (!window.Worker) {
      this.onStatusNote('Web Worker not supported');
      if (this.resolveReady) { this.resolveReady(false); this.resolveReady = this.rejectReady = null; }
      return null;
    }
    try {
      const worker = new Worker(window.__MCLD_INLINE_WORKER_URL__ || './simulation_worker.js');
      worker.onmessage = this._handleMessage;
      worker.onerror = (error) => {
        console.error('Worker failed.', error);
        this.onStatusNote(`Worker failed: ${error && error.message ? error.message : error}`);
      };
      worker.postMessage({ type: 'boot', simDt, bufferConfig: this.computeBufferConfig() });
      this.client = worker;
      return worker;
    } catch (error) {
      console.error('Worker start failed.', error);
      this.onStatusNote(`Worker start failed: ${error && error.message ? error.message : error}`);
      if (this.resolveReady) { this.resolveReady(false); this.resolveReady = this.rejectReady = null; }
      return null;
    }
  }

  waitUntilReady() {
    return this.readyPromise || Promise.resolve(this.ready);
  }

  reportDrain(consumed) {
    if (!this.client || !this.ready) return;
    const bufferConfig = this.computeBufferConfig();
    const needFill = !this.state.initializing && this.queueLength() < bufferConfig.lowWater && !this.fillRequested;
    if (!(consumed > 0) && !needFill) return;
    this.client.postMessage({
      type: 'drain',
      consumed: Math.max(0, consumed | 0),
      requestFill: needFill,
      queueLength: this.queueLength(),
      bufferConfig,
    });
    if (needFill) this.fillRequested = true;
  }

  requestFillIfNeeded() {
    this.reportDrain(0);
  }

  applyInit(cfg, savedState, l1Vis, l2Vis) {
    return new Promise((resolve, reject) => {
      if (!this.client) {
        reject(new Error('worker unavailable'));
        return;
      }
      this.pendingApply = { resolve, reject };
      this.fillRequested = false;
      this.client.postMessage({ type: 'applyInit', cfg, savedState, l1Vis, l2Vis, bufferConfig: this.computeBufferConfig() });
    });
  }
}

Object.assign(exports, { WorkerStream });

};
__modules["persist_state.js"] = function(exports, __require){
const { PERSIST_STORAGE_KEY, readPersistedSimState, clearPersistedSimState, fetchWallpaperConfigRecord } = __require("runtime_core.js");

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

Object.assign(exports, { maybePersistSimulationState, loadAndApplyConfigWithStateRestore });

};
__modules["hud_overlay.js"] = function(exports, __require){

function fmtFixed(value, width, digits = 2) {
  const text = Number.isFinite(value) ? Number(value).toFixed(digits) : '--';
  return text.padStart(width, ' ');
}
function fmtExp(value, width, digits = 2) {
  const text = Number.isFinite(value) ? Number(value).toExponential(digits) : '--';
  return text.padStart(width, ' ');
}
function fmtInt(value, width) {
  const text = Number.isFinite(value) ? String(Math.max(0, Math.floor(value))) : '--';
  return text.padStart(width, ' ');
}
function rodModeLabel(fit) {
  const mode = fit && typeof fit.mode === 'string' ? fit.mode : '';
  if (mode === 'manual') return 'manual';
  if (mode === 'auto') return 'auto';
  return '---';
}
function buildHudText({ state, workerReady, simTime, queueLength, latestDiag, stepWarningText }) {
  const runState = state.playing ? 'run' : 'pause';
  const shellState = state.showPotentialShell ? 'on' : 'off';
  const fpsActual = Number.isFinite(state.actualFps) ? Math.round(state.actualFps) : 0;
  const fpsTarget = Number.isFinite(state.targetFps) && state.targetFps > 0 ? Math.round(state.targetFps) : 0;
  const fpsText = `${String(fpsActual).padStart(3, ' ')}/${fpsTarget > 0 ? String(fpsTarget).padStart(3, ' ') : ' --'}`;
  const line1 = `perf  fps:${fpsText}  queue:${fmtInt(queueLength, 3)}  shell:${shellState}`;
  const line2 = `sim   state:${runState.padEnd(5, ' ')}  t:${fmtFixed(simTime, 7, 2)}  speed:${fmtFixed(state.renderSpeed, 4, 2)}x  spin:${fmtFixed(-state.autoRotateZ, 5, 2)}`;
  const line3 = `solve H:${fmtExp(latestDiag?.H_cur, 11, 3)}  q1:${fmtExp(latestDiag?.q1_err, 10, 2)}  q2:${fmtExp(latestDiag?.q2_err, 10, 2)}  over_tol:${fmtInt(state.overThresholdCount, 4)}`;
  const line4 = `rod   r1:${rodModeLabel(state.rodFit1).padEnd(6, ' ')} loss:${fmtFixed(state.rodFit1?.loss, 8, 4)}  r2:${rodModeLabel(state.rodFit2).padEnd(6, ' ')} loss:${fmtFixed(state.rodFit2?.loss, 8, 4)}`;
  const warning = stepWarningText ? `warn  ${stepWarningText}` : '';
  return warning ? `${line1}
${line2}
${line3}
${line4}
${warning}` : `${line1}
${line2}
${line3}
${line4}`;
}
function updateHudOverlay(statusElement, context) {
  if (!statusElement) return;
  const nextText = buildHudText(context);
  if (statusElement.textContent !== nextText) statusElement.textContent = nextText;
}
Object.assign(exports, { buildHudText, updateHudOverlay });

};
__modules["theme_state.js"] = function(exports, __require){

function clamp01(x){ return Math.max(0, Math.min(1, x)); }
function clamp255(x){ return Math.max(0, Math.min(255, Math.round(x))); }
function rgbToHsl(rgb){
  let [r,g,b]=rgb.map(v=>v/255);
  const max=Math.max(r,g,b), min=Math.min(r,g,b);
  let h=0,s=0; const l=(max+min)/2;
  if(max!==min){
    const d=max-min;
    s=l>0.5 ? d/(2-max-min) : d/(max+min);
    switch(max){
      case r: h=(g-b)/d + (g<b?6:0); break;
      case g: h=(b-r)/d + 2; break;
      default: h=(r-g)/d + 4; break;
    }
    h*=60;
  }
  return [h,s,l];
}
function hue2rgb(p,q,t){
  if(t<0) t+=1; if(t>1) t-=1;
  if(t<1/6) return p+(q-p)*6*t;
  if(t<1/2) return q;
  if(t<2/3) return p+(q-p)*(2/3-t)*6;
  return p;
}
function hslToRgb(h,s,l){
  h=((h%360)+360)%360;
  let r,g,b;
  if(s===0){ r=g=b=l; }
  else {
    const q=l<0.5 ? l*(1+s) : l+s-l*s;
    const p=2*l-q;
    r=hue2rgb(p,q,h/360+1/3); g=hue2rgb(p,q,h/360); b=hue2rgb(p,q,h/360-1/3);
  }
  return [clamp255(r*255), clamp255(g*255), clamp255(b*255)];
}
function mixRgb(a,b,t){ return [0,1,2].map(i=>clamp255(a[i]+(b[i]-a[i])*t)); }
function captureManualPalette(state){
  return {
    backgroundColor:[...state.backgroundColor], rodColor1:[...state.rodColor1], rodColor2:[...state.rodColor2],
    barColor1:[...state.barColor1], barColor2:[...state.barColor2],
    ballColorRoot:[...state.ballColorRoot], ballColorJoint:[...state.ballColorJoint], ballColorTip:[...state.ballColorTip],
    trace1Slow:[...state.trace1Slow], trace1Mid:[...state.trace1Mid], trace1Fast:[...state.trace1Fast],
    trace2Slow:[...state.trace2Slow], trace2Mid:[...state.trace2Mid], trace2Fast:[...state.trace2Fast],
  };
}
function deriveAutoPalette(schemeRgb, manual){
  let [h,s,l]=rgbToHsl(schemeRgb);
  if(s<0.12){
    const mh=rgbToHsl(manual.barColor1 || [182,156,62])[0];
    h=mh; s=0.24; l=0.52;
  }
  if(l<0.12) l=0.48;
  if(l>0.88) l=0.62;
  const sat=v=>clamp01(v), lig=v=>clamp01(v);
  return {
    backgroundColor:hslToRgb(h, sat(Math.max(0.18, Math.min(s*0.45,0.30))), 0.07),
    rodColor1:hslToRgb(h, 0.18, 0.07),
    rodColor2:hslToRgb(h, 0.22, 0.12),
    barColor1:hslToRgb(h, sat(Math.max(0.58, s*0.95)), lig(0.50)),
    barColor2:hslToRgb(h+34, sat(Math.max(0.54, s*0.86)), lig(0.54)),
    ballColorRoot:hslToRgb(h, 0.32, 0.86),
    ballColorJoint:hslToRgb(h, 0.12, 0.76),
    ballColorTip:hslToRgb(h+12, 0.20, 0.88),
    trace1Slow:hslToRgb(h-8, sat(Math.max(0.60, s*0.82)), 0.77),
    trace1Mid:hslToRgb(h, sat(Math.max(0.74, s)), 0.64),
    trace1Fast:hslToRgb(h+8, sat(Math.max(0.86, s)), 0.58),
    trace2Slow:hslToRgb(h+28, sat(Math.max(0.60, s*0.80)), 0.76),
    trace2Mid:hslToRgb(h+44, sat(Math.max(0.74, s*0.92)), 0.66),
    trace2Fast:hslToRgb(h+58, sat(Math.max(0.84, s)), 0.58),
  };
}
function deriveLightPalette(palette){
  const primary=palette.barColor1, secondary=palette.barColor2;
  const bg=palette.backgroundColor;
  return {
    keyLightColor: mixRgb([255,248,240], primary, 0.10),
    hemiSkyColor: mixRgb([46,54,70], primary, 0.14),
    hemiGroundColor: mixRgb([20,20,22], bg, 0.22),
    rimLightColor: mixRgb([255,255,255], secondary, 0.38),
  };
}
function applyPaletteToState(state, palette){
  for(const [k,v] of Object.entries(palette)) state[k]=[...v];
  const lights=deriveLightPalette(palette);
  state.keyLightColor=[...lights.keyLightColor];
  state.hemiSkyColor=[...lights.hemiSkyColor];
  state.hemiGroundColor=[...lights.hemiGroundColor];
  state.rimLightColor=[...lights.rimLightColor];
}
function createThemeState(state){
  const manual=captureManualPalette(state);
  return { auto:false, schemeColor:[...state.schemeColor], manual, effective:manual };
}
function ensureThemeState(state){
  if(!state.themeState) state.themeState=createThemeState(state);
  return state.themeState;
}
function setManualPaletteValue(themeState, key, rgb){ themeState.manual[key]=[...rgb]; }
function applyThemeState(state){
  const themeState=ensureThemeState(state);
  const palette=themeState.auto ? deriveAutoPalette(themeState.schemeColor, themeState.manual) : themeState.manual;
  themeState.effective=palette;
  applyPaletteToState(state, palette);
  return palette;
}
function updateThemeStateFromProperties(state, properties, parseColor){
  const themeState=ensureThemeState(state);
  let themeChanged=false;
  if(properties.autotheme){ const next=!!properties.autotheme.value; if(next!==themeState.auto){ themeState.auto=next; themeChanged=true; } }
  if(properties.schemecolor){ themeState.schemeColor=parseColor(properties.schemecolor.value, themeState.schemeColor); themeChanged=true; }
  const mapping={
    backgroundcolor:'backgroundColor', rodcolor1:'rodColor1', rodcolor2:'rodColor2', barcolor1:'barColor1', barcolor2:'barColor2',
    ballcolorroot:'ballColorRoot', ballcolorjoint:'ballColorJoint', ballcolortip:'ballColorTip',
    trace1slow:'trace1Slow', trace1mid:'trace1Mid', trace1fast:'trace1Fast', trace2slow:'trace2Slow', trace2mid:'trace2Mid', trace2fast:'trace2Fast'
  };
  for(const [prop,key] of Object.entries(mapping)){
    if(properties[prop]){ setManualPaletteValue(themeState, key, parseColor(properties[prop].value, themeState.manual[key] || [0,0,0])); themeChanged=true; }
  }
  if(themeChanged) applyThemeState(state);
  return themeChanged;
}
Object.assign(exports, { createThemeState, ensureThemeState, applyThemeState, updateThemeStateFromProperties, captureManualPalette });

};
__modules["main.js"] = function(exports, __require){
__require("we_property_bridge.js");
const {
  canvas, gl, statusEl, hudEl, state, camera, ENABLE_POTENTIAL_SHELL,
  DEFAULT_WEB_CONFIG, CAMERA_PITCH_LIMIT, CAMERA_DRAG_SENSITIVITY, CAMERA_RELEASE_SPEED_LIMIT, PERSIST_INTERVAL_MS,
  trailClear, cameraRotateOrbit, cameraSetPitch, quatNormalize, clamp,
  poseFromConfigState, clearPersistedSimState, estimateCurrentSpeedsFromPoseDelta,
  estimateTraceSpeedScales, syncVisualStateFromConfig, updateViewAndTraceStatsFromSummary,
  makeFallbackSummary, resize, poseFromFrame, parseWallpaperColorValue, estimatePotentialEnvelopeSummary,
  applySpeedValue, cloneConfig
} = __require("runtime_core.js");
const { createSceneRenderer, pushTrail } = __require("scene_renderer.js");
const { WorkerStream } = __require("worker_stream.js");
const { maybePersistSimulationState, loadAndApplyConfigWithStateRestore } = __require("persist_state.js");
const { updateHudOverlay } = __require("hud_overlay.js");
const { createThemeState, applyThemeState, updateThemeStateFromProperties } = __require("theme_state.js");

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

};
function __require(id){if(__instances[id])return __instances[id];if(!__modules[id])throw new Error("Missing bundled module: "+id);const exports=__instances[id]=Object.create(null);__modules[id](exports,__require);return exports;}
__require("main.js");
})();
