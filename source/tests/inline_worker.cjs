const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { gunzipSync } = require('node:zlib');
const root = path.resolve(__dirname,'..','..');
(async()=>{
 const messages=[];
 const ctx=vm.createContext({console, WebAssembly, Blob, Response, DecompressionStream, TextDecoder, TextEncoder, URL, atob,
  setTimeout, clearTimeout, performance,
  fetch(){throw new Error('Inline worker attempted a network fetch');},
  importScripts(){throw new Error('Inline worker attempted importScripts');},
  postMessage(m){messages.push(m);},
 });
 ctx.self=ctx;
 const worker=fs.readFileSync(path.join(root,'build/worker_bundle.js'),'utf8');
 vm.runInContext(worker,ctx);
 await ctx.onmessage({data:{type:'boot',simDt:1/240}});
 assert(messages.some(m=>m.type==='ready'),JSON.stringify(messages));
 const wasmMatch=worker.match(/__decodeGzip\("(H4sI[^"]+)"\)/);
 assert(wasmMatch);
 assert(gunzipSync(Buffer.from(wasmMatch[1],'base64')).equals(fs.readFileSync(path.join(root,'source/mcld_solver.wasm'))));
 const cfg={solverMode:'hamilton_left',initMode:'left',simDt:1/240,maxIter:6,tol:1.5e-11,beta:1,
  params:[0.5,1,3,1,0.25,0.5,0.04,0.05,0.003,0.001,9.81],yPrev:[1,0,0,0,1,0,0,0],
  yCur:[1,0,0,0,Math.SQRT1_2,Math.SQRT1_2,0,0],pLeft:[0,0,0,0,0,0],pRight:[0,0,0,0,0,0],angularVelocity:[0,0,0,0,0,0]};
 await ctx.onmessage({data:{type:'applyInit',cfg,l1Vis:0.5,l2Vis:1,bufferConfig:{lowWater:24,highWater:32,chunkSize:4}}});
 const applied=messages.find(m=>m.type==='applied');
 assert(applied&&applied.ok,JSON.stringify(messages.filter(m=>m.type!=='frames')));
 assert(applied.frames.length>=24);assert(applied.frames.at(-1).t>0);
 assert(applied.initialFrame.persistSnapshot.qHex.length===8);
 console.log('PASS: actual inline WASM initialization, solver configuration, frame generation, hex snapshot; zero network/importScripts.');
})().catch(e=>{console.error(e);process.exitCode=1;});
