const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
(async () => {
  const notices = [];
  class Worker { postMessage() {} }
  const c = vm.createContext({window:{Worker}, Worker, console:{error(){}}});
  vm.runInContext(fs.readFileSync(path.join(root,'worker_stream.js'),'utf8').replace(/export\s*\{[^}]*\};?/,''),c);
  c.notify = message => notices.push(message);
  const make = () => vm.runInContext('new WorkerStream({state:{},onStatusNote:notify})',c);
  let s=make();s.start(1/240);s.client.onmessage({data:{type:'unavailable',message:'factory missing'}});
  assert.equal(await s.waitUntilReady(),false);assert.equal(notices.at(-1),'Worker unavailable: factory missing');
  s=make();s.start(1/240);s.client.onerror({message:'runtime failure'});
  assert.equal(notices.at(-1),'Worker failed: runtime failure');
  s=make();s.start(1/240);s.client.onmessage({data:{type:'ready'}});
  assert.equal(await s.waitUntilReady(),true);
  const promise=s.applyInit({},null,1,1);
  s.client.onmessage({data:{type:'apply_error',message:'apply failure'}});
  await assert.rejects(promise,/apply failure/);
  c.window.Worker=null;s=make();s.start(1/240);
  assert.equal(await s.waitUntilReady(),false);assert.equal(notices.at(-1),'Web Worker not supported');

  // Test the main.js status forwarding, equivalent to fix11h's callback.
  const main=fs.readFileSync(path.join(root,'main.js'),'utf8');
  const p=vm.createContext({});
  vm.runInContext(`let stepWarningText='',hudRefreshPending=false; const statusEl={textContent:''}; function requestHudRefresh(){hudRefreshPending=true;}
    ${main.slice(main.indexOf('function reportWorkerIssue('),main.indexOf('function noteMessage('))}
    reportWorkerIssue('Worker failed: test');
    globalThis.forwarded={warning:stepWarningText,text:statusEl.textContent,refresh:hudRefreshPending};`,p);
  assert.equal(p.forwarded.warning,'Worker failed: test');assert.equal(p.forwarded.text,p.forwarded.warning);assert(p.forwarded.refresh);
  const a=main.indexOf('  if (properties.persiststate) {');const b=main.indexOf('  state.showBar = true;',a);
  vm.runInContext(`const state={persistStateEnabled:true,simTimeOffset:9,playing:true,workerGeneration:42};
    let latestPersistSnapshot={old:true},lastPersistSaveMs=123,clears=0,reinitializations=0;
    function clearPersistedSimState(){clears++;}
    function loadAndApplyWallpaperConfig(){reinitializations++;}
    function applySwitch(properties){${main.slice(a,b)}}
    applySwitch({persiststate:{value:false}});
    globalThis.disabled={playing:state.playing,generation:state.workerGeneration,clears,reinitializations,snapshot:latestPersistSnapshot};
    applySwitch({persiststate:{value:true}});
    globalThis.enabled={playing:state.playing,timer:lastPersistSaveMs,reinitializations};`,p);
  assert.equal(p.disabled.playing,true);assert.equal(p.disabled.generation,42);assert.equal(p.disabled.clears,1);
  assert.equal(p.disabled.reinitializations,0);assert.equal(p.disabled.snapshot,null);
  assert.equal(p.enabled.playing,true);assert.equal(p.enabled.timer,0);assert.equal(p.enabled.reinitializations,0);
  assert(!main.includes('pendulumDiagnostics'));
  console.log('PASS: historical Worker status forwarding, existing HUD messages, initialization errors, and persistence switch without reinitialization.');
})().catch(e=>{console.error(e);process.exitCode=1;});
