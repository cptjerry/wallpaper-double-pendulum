const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root=path.resolve(__dirname,'..','..');
const html=fs.readFileSync(path.join(root,'wallpaper/index.html'),'utf8');
const scripts=[...html.matchAll(/<script([^>]*)>([\s\S]*?)<\/script>/g)];
const config=fs.readFileSync(path.join(root,'wallpaper/wallpaper_config.json'),'utf8');
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
async function start(storage, inject) {
 const errors=[],handlers={},frames=[],workers=[];
 const element=()=>({style:{},textContent:'',width:0,height:0,clientWidth:1920,clientHeight:1080,
  getContext:()=>null,addEventListener(){},setPointerCapture(){},releasePointerCapture(){},
  classList:{values:new Set(),toggle(n,on){if(on)this.values.add(n);else this.values.delete(n);},add(n){this.values.add(n);},remove(n){this.values.delete(n);},contains(n){return this.values.has(n);}}});
 const elements=Object.fromEntries(['stage','hud','status','hudTitle','bootVeil','workerPayload','appPayload'].map(x=>[x,element()]));
 for(const m of scripts){const id=m[1].match(/id="([^"]+)"/);if(id)elements[id[1]].textContent=m[2];}
 const body=element();body.classList.add('boot-hidden');
 const ctx=vm.createContext({console:{log(){},warn(){},error(...args){errors.push(args.map(String).join(' '));}},
  Blob,Response,DecompressionStream,TextDecoder,TextEncoder,URL,atob,performance,
  innerWidth:1920,innerHeight:1080,devicePixelRatio:1,navigator:{},
  localStorage:{getItem:k=>storage.get(k)??null,setItem:(k,v)=>storage.set(k,v),removeItem:k=>storage.delete(k)},
  addEventListener:(name,fn)=>{(handlers[name]??=new Set()).add(fn);},removeEventListener:(name,fn)=>handlers[name]?.delete(fn),
  requestAnimationFrame:fn=>frames.push(fn),
  fetch:async url=>{assert.equal(url,'./wallpaper_config.json');return new Response(config);},
 });
 ctx.window=ctx;
 ctx.document={body,getElementById:id=>elements[id]||null,createElement:()=>element()};
 body.appendChild=el=>{
  try{vm.runInContext(el.textContent,ctx,{filename:'inline-app.js'});}
  catch(error){for(const fn of handlers.error||[])fn({error,message:error.message});}
 };
 class Worker {
  constructor(url){
   assert(url.startsWith('blob:'));this.sent=[];workers.push(this);
   this.loaded=(async()=>{
    const code=await (await fetch(url)).text();
    const wc=vm.createContext({console,WebAssembly,Blob,Response,DecompressionStream,TextDecoder,TextEncoder,URL,atob,performance,setTimeout,clearTimeout,
     fetch(){throw new Error('Unexpected Worker fetch');},importScripts(){throw new Error('Unexpected Worker importScripts');},
     postMessage:m=>{queueMicrotask(()=>this.onmessage?.({data:m}));},
    });wc.self=wc;vm.runInContext(code,wc);this.wc=wc;
   })();
  }
  postMessage(m){this.sent.push(m);this.loaded.then(()=>this.wc.onmessage({data:m})).catch(e=>{errors.push(String(e));this.onerror?.({message:e.message});});}
 }
 ctx.Worker=Worker;
 vm.runInContext(scripts[0][2],ctx);
 const listener=ctx.wallpaperPropertyListener;
 if(inject)inject(ctx,'early');
 vm.runInContext(scripts.at(-1)[2],ctx);
 if(inject)inject(ctx,'decoding');
 for(let i=0;i<200;i++){
  if(workers[0]?.sent.some(m=>m.type==='drain')&&!body.classList.contains('boot-hidden'))break;
  await sleep(10);
 }
 assert.equal(errors.length,0,errors.join('\n'));
 assert.equal(body.classList.contains('boot-hidden'),false,elements.status.textContent);
 assert(workers[0]?.sent.some(m=>m.type==='applyInit'),'solver was not initialized');
 assert.strictEqual(ctx.wallpaperPropertyListener,listener,'property listener replaced during startup');
 frames.shift()(performance.now()+40);
 return {ctx,storage,workers,elements,body,errors};
}
(async()=>{
 const storage=new Map([
  ['mcld_we_general_props_cache',JSON.stringify({fps:60})],
  ['mcld_we_user_props_cache',JSON.stringify({playbackspeed:{value:0.4},hudposition:{value:1},persiststate:{value:false}})],
 ]);
 const first=await start(storage,(c,phase)=>{
  if(phase==='early')c.wallpaperPropertyListener.applyUserProperties({playbackspeed:{value:0.75},controlmode:{value:2},controlpitch:{value:90},rotationspeed:{value:0.2}});
  else{c.wallpaperPropertyListener.applyGeneralProperties({fps:30});c.wallpaperPropertyListener.applyUserProperties({size:{value:2}});}
 });
 assert(first.elements.status.textContent.includes('0.75x'),first.elements.status.textContent);
 let cached=JSON.parse(storage.get('mcld_we_user_props_cache'));
 assert.equal(cached.hudposition.value,1);assert.equal(cached.size.value,2);assert.equal(cached.playbackspeed.value,0.75);
 assert.equal(JSON.parse(storage.get('mcld_we_general_props_cache')).fps,30);
 const applies=first.workers[0].sent.filter(m=>m.type==='applyInit').length;
 first.ctx.wallpaperPropertyListener.applyUserProperties({persiststate:{value:true}});
 first.ctx.wallpaperPropertyListener.applyUserProperties({persiststate:{value:false},playbackspeed:{value:0.3}});
 assert.equal(first.workers[0].sent.filter(m=>m.type==='applyInit').length,applies,'persist toggle restarted solver');
 first.ctx.wallpaperPropertyListener.applyGeneralProperties({fps:48});
 assert.equal(JSON.parse(storage.get('mcld_we_user_props_cache')).playbackspeed.value,0.3);
 const second=await start(storage);
 assert(second.elements.status.textContent.includes('0.30x'),second.elements.status.textContent);
 assert(second.elements.status.textContent.includes('48'),second.elements.status.textContent);
 console.log('PASS: full HTML bootstrap + actual WASM Worker; early/during-decode/live properties; merged caches and restart replay; persistence toggle without reinitialization.');
})().catch(e=>{console.error(e);process.exitCode=1;});
