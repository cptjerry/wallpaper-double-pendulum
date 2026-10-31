// Preview-only instrumentation, inserted into an in-memory srcdoc document.
(function(){
  const host=parent.__wallpaperTestHost;
  const run=window.__WE_TEST_RUN__;
  const log=(kind,message)=>host?.log(run,kind,message);
  for(const kind of ['log','warn','error']){
    const original=console[kind].bind(console);
    console[kind]=(...args)=>{original(...args);log(kind,args.map(a=>a?.stack||a?.message||String(a)).join(' '));};
  }
  window.addEventListener('error',e=>{
    const src=e.target?.src||e.target?.href;
    log('error',src?'Resource load failed: '+src:(e.message||'Script error')+' '+(e.filename||'')+':'+(e.lineno||''));
  },true);
  window.addEventListener('unhandledrejection',e=>log('error','Promise: '+(e.reason?.stack||e.reason?.message||String(e.reason))));
  const NativeWorker=window.Worker;
  if(NativeWorker){
    window.Worker=class extends NativeWorker{
      constructor(...args){
        super(...args);
        this.addEventListener('error',e=>log('error','Worker: '+e.message));
        this.addEventListener('message',e=>{
          const d=e.data||{};
          if(['ready','applied','unavailable','apply_error','status'].includes(d.type))log(d.type.includes('error')||d.type==='unavailable'?'error':'log','Worker '+d.type+(d.message?': '+d.message:''));
        });
      }
    };
  }
  let listener;
  Object.defineProperty(window,'wallpaperPropertyListener',{
    configurable:true,
    get(){return listener;},
    set(value){listener=value;host?.listenerReady(window,run);},
  });
  log('log','Test hooks installed; waiting for the WE listener');
})();
