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
