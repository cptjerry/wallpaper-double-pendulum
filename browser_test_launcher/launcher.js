'use strict';
const frame=document.getElementById('preview'),logs=document.getElementById('logs'),status=document.getElementById('state');
const settingsKey='double_pendulum_browser_test_settings';
const wallpaperBase=new URL('../wallpaper/',location.href);
const emulatorURL=new URL('./emulate_we.js',location.href);
let specs={},values={},inputs={},run=0,receivedRun=-1,cachedStart=false,lines=[];
function log(id,kind,message){
  if(id!==run)return;
  lines.push(new Date().toLocaleTimeString()+' ['+kind+'] '+message);
  if(lines.length>160)lines.shift();logs.textContent=lines.join('\n');logs.parentElement.scrollTop=logs.parentElement.scrollHeight;
}
function persist(){try{localStorage.setItem(settingsKey,JSON.stringify({values,fps:Number(document.getElementById('fps').value)}));}catch{}}
function userProperties(){return Object.fromEntries(Object.entries(values).map(([k,v])=>[k,{value:v}]));}
function refreshConditions(){
  for(const [key,spec] of Object.entries(specs)){
    const m=spec.condition?.match(/^(\w+)\.value\s*(!=|==)\s*(-?[\d.]+)$/);
    if(m&&inputs[key])inputs[key].row.hidden=m[2]==='=='?Number(values[m[1]])!==Number(m[3]):Number(values[m[1]])===Number(m[3]);
  }
  document.getElementById('raw').value=JSON.stringify(userProperties(),null,2);
}
function send(patch){
  persist();refreshConditions();
  const listener=frame.contentWindow?.wallpaperPropertyListener;
  if(!listener?.applyUserProperties){log(run,'log','Properties retained; waiting for the listener');return;}
  try{listener.applyUserProperties(patch);log(run,'log','user: '+JSON.stringify(patch));}catch(e){log(run,'error',e.stack||e.message);}
}
function setValues(patch){
  for(const [key,value] of Object.entries(patch)){
    values[key]=value;inputs[key]?.set(value);
  }
  send(Object.fromEntries(Object.entries(patch).map(([key,value])=>[key,{value}])));
}
function colorHex(value){return '#'+String(value).trim().split(/\s+/).slice(0,3).map(x=>Math.max(0,Math.min(255,Math.round(Number(x)*255))).toString(16).padStart(2,'0')).join('');}
function renderControls(){
  const container=document.getElementById('controls');container.textContent='';inputs={};
  const groups=[
    {name:'Playback & persistence',keys:['playbackspeed','trailseconds','persiststate'],open:true},
    {name:'Camera',keys:['controlmode','controlpitch','rotationspeed'],open:true},
    {name:'Layout & HUD',keys:['size','screenoffsetx','screenoffsety','hudposition','hudscale','showshell'],open:true},
    {name:'Theme & colours',keys:[],open:false},
  ];
  const assigned=new Set(groups.flatMap(g=>g.keys));
  groups[3].keys=Object.keys(specs).filter(k=>!assigned.has(k));
  for(const group of groups){
    const section=document.createElement('details');section.className='setting-group';section.open=group.open;
    const summary=document.createElement('summary');summary.textContent=group.name;
    const reset=document.createElement('button');reset.type='button';reset.className='group-reset';reset.textContent='Reset';
    reset.setAttribute('aria-label','Reset '+group.name);
    reset.onclick=event=>{event.preventDefault();event.stopPropagation();setValues(Object.fromEntries(group.keys.filter(k=>specs[k]).map(k=>[k,specs[k].value])));};
    summary.appendChild(reset);section.appendChild(summary);container.appendChild(section);
    if(group.name==='Theme & colours'){
      const hint=document.createElement('p');hint.className='muted';hint.textContent='Turn off Auto Theme to use the manual palette.';section.appendChild(hint);
    }
    for(const key of group.keys){
      const spec=specs[key];if(!spec)continue;
      const row=document.createElement('label');row.className='row';
      const name=document.createElement('span');name.textContent=spec.text||key;name.title=key;row.appendChild(name);
      let input,set,control;
      if(spec.type==='combo'){
        input=document.createElement('select');
        for(const [i,opt] of spec.options.entries()){const option=document.createElement('option');option.value=i;option.textContent=opt.label;input.appendChild(option);}
        set=v=>{input.value=spec.options.findIndex(o=>String(o.value)===String(v));};
        input.onchange=()=>setValues({[key]:spec.options[Number(input.value)].value});
      }else{
        input=document.createElement('input');
        if(spec.type==='bool'){input.type='checkbox';set=v=>{input.checked=!!v;};input.onchange=()=>setValues({[key]:input.checked});}
        else if(spec.type==='color'){input.type='color';set=v=>{input.value=colorHex(v);};input.onchange=()=>setValues({[key]:input.value.slice(1).match(/../g).map(x=>parseInt(x,16)/255).join(' ')});}
        else{
          input.type='number';input.min=spec.min??'';input.max=spec.max??'';input.step=spec.step??'any';
          const range=document.createElement('input');range.type='range';range.min=input.min;range.max=input.max;range.step=input.step;
          range.setAttribute('aria-label',(spec.text||key)+' slider');
          set=v=>{input.value=v;range.value=v;};
          input.onchange=()=>{if(input.checkValidity())setValues({[key]:Number(input.value)});};
          range.oninput=()=>setValues({[key]:Number(range.value)});
          control=document.createElement('div');control.className='slider-control';control.appendChild(range);control.appendChild(input);
        }
      }
      input.setAttribute('aria-label',spec.text||key);
      set(values[key]);row.appendChild(control||input);inputs[key]={row,set};section.appendChild(row);
    }
  }
  refreshConditions();
}
window.__wallpaperTestHost={
  log,
  listenerReady(child,id){
    if(id!==run||child!==frame.contentWindow||receivedRun===id)return;
    receivedRun=id;
    if(cachedStart){log(run,'log','Listener ready; no initial callbacks sent. Using the wallpaper cache.');return;}
    const delay=Math.max(0,Number(document.getElementById('delay').value)||0);
    const general={fps:Number(document.getElementById('fps').value)||0};
    const user=userProperties();
    const deliver=()=>{
      if(id!==run)return;
      try{child.wallpaperPropertyListener.applyGeneralProperties(general);child.wallpaperPropertyListener.applyUserProperties(user);log(id,'log','Sent general and full initial user properties');}
      catch(e){log(id,'error',e.stack||e.message);}
    };
    if(delay)setTimeout(deliver,delay);else deliver();
  },
};
async function load(useCache=false){
  const id=++run;receivedRun=-1;cachedStart=useCache;lines=[];logs.textContent='';status.textContent='Loading preview…';
  if(useCache){
    try{
      const user=JSON.parse(localStorage.getItem('mcld_we_user_props_cache')||'{}');
      for(const key of Object.keys(specs)){if(user[key]&&'value' in user[key]){values[key]=user[key].value;inputs[key]?.set(values[key]);}}
      const general=JSON.parse(localStorage.getItem('mcld_we_general_props_cache')||'{}');
      if(general.fps!=null)document.getElementById('fps').value=general.fps;
      refreshConditions();
    }catch(e){log(id,'error','Cache read failed: '+e.message);}
  }
  try{
    const response=await fetch(new URL('index.html',wallpaperBase),{cache:'no-store'});if(!response.ok)throw Error('index.html HTTP '+response.status);
    const original=await response.text();if(id!==run)return;
    // Only the iframe's in-memory document is instrumented. The file and its
    // ordinary HTTP endpoint are never rewritten or patched.
    const prelude='<base href="'+wallpaperBase.href+'"><script>window.__WE_TEST_RUN__='+id+';<\/script><script src="'+emulatorURL.href+'"><\/script>';
    if(!/<head\b[^>]*>/i.test(original))throw Error('index.html has no head element; cannot insert the test environment');
    frame.srcdoc=original.replace(/<head\b[^>]*>/i,m=>m+prelude);
    log(id,'log','Loaded original index.html; test scripts inserted into memory only');
  }catch(e){status.textContent='Preview load failed';log(id,'error',e.message);}
}
function selectPanel(which){
  const settings=which==='settings';
  document.getElementById('settingsPanel').hidden=!settings;
  document.getElementById('testsPanel').hidden=settings;
  document.getElementById('settingsTab').setAttribute('aria-selected',String(settings));
  document.getElementById('testsTab').setAttribute('aria-selected',String(!settings));
}
document.getElementById('settingsTab').onclick=()=>selectPanel('settings');
document.getElementById('testsTab').onclick=()=>selectPanel('tests');
document.getElementById('settingsToggle').onclick=()=>{
  const hidden=document.body.classList.toggle('sidebar-hidden');
  document.getElementById('settingsToggle').textContent=hidden?'Show settings':'Hide settings';
};
document.getElementById('reset').onclick=()=>{
  setValues(Object.fromEntries(Object.entries(specs).map(([k,v])=>[k,v.value])));
  log(run,'log','Wallpaper properties reset to project defaults');
};
document.getElementById('reload').onclick=()=>load(false);
document.getElementById('cached').onclick=()=>load(true);
document.getElementById('top').onclick=()=>setValues({controlmode:2,controlpitch:90,rotationspeed:0.25,hudposition:1});
document.getElementById('bottom').onclick=()=>setValues({controlmode:2,controlpitch:-90,rotationspeed:0.25,hudposition:1});
document.getElementById('mouse').onclick=()=>setValues({controlmode:0,hudposition:1});
document.getElementById('fps').onchange=()=>{
  persist();const general={fps:Number(document.getElementById('fps').value)||0};
  try{frame.contentWindow?.wallpaperPropertyListener?.applyGeneralProperties(general);log(run,'log','general: '+JSON.stringify(general));}catch(e){log(run,'error',e.message);}
};
document.getElementById('sendRaw').onclick=()=>{
  try{
    const patch=JSON.parse(document.getElementById('raw').value);if(!patch||Array.isArray(patch)||typeof patch!=='object')throw Error('Expected a property object');
    for(const [key,prop] of Object.entries(patch))if(key in specs&&prop&&'value' in prop){values[key]=prop.value;inputs[key]?.set(prop.value);}
    send(patch);
  }catch(e){log(run,'error',e.message);}
};
document.getElementById('clearSim').onclick=()=>{localStorage.removeItem('mcld_wallpaper_saved_state');log(run,'log','Simulation save cleared. A running wallpaper may save again; disable Restore Previous State before reloading.');};
document.getElementById('clearProps').onclick=()=>{for(const key of ['mcld_we_general_props_cache','mcld_we_user_props_cache'])localStorage.removeItem(key);log(run,'log','WE cache cleared. Use Start from cache only to test startup without cached properties.');};
document.getElementById('full').onclick=()=>document.body.classList.add('full');
function leaveFull(e){if(e.key==='Escape')document.body.classList.remove('full');}
document.getElementById('exitFull').onclick=()=>document.body.classList.remove('full');
window.addEventListener('keydown',leaveFull);frame.addEventListener('load',()=>{try{frame.contentWindow.addEventListener('keydown',leaveFull);}catch{}});
document.getElementById('copy').onclick=async()=>{try{await navigator.clipboard.writeText(lines.join('\n'));}catch(e){log(run,'error','Copy failed; select and copy the log text instead: '+e.message);}};
setInterval(()=>{
  try{
    const doc=frame.contentDocument;if(!doc||frame.contentWindow.__WE_TEST_RUN__!==run)return;
    const text=doc.getElementById('status')?.textContent||'';
    status.textContent=(doc.body?.classList.contains('boot-hidden')?'Starting':'Page visible')+(text?' · '+text.split('\n')[0]:'');
    status.title=text;
  }catch{}
},250);
(async()=>{
  try{
    if(location.protocol==='file:')throw Error('Open this page through a static server. Browsers normally block file:// access to neighbouring project files.');
    const response=await fetch(new URL('project.json',wallpaperBase));if(!response.ok)throw Error('project.json HTTP '+response.status);
    const project=await response.json();specs=project.general.properties;
    values=Object.fromEntries(Object.entries(specs).map(([k,v])=>[k,v.value]));
    values.hudposition=1; // Preview starts with a visible HUD for observation.
    try{const saved=JSON.parse(localStorage.getItem(settingsKey));if(saved?.values){for(const key of Object.keys(specs))if(key in saved.values)values[key]=saved.values[key];}if(saved?.fps!=null)document.getElementById('fps').value=saved.fps;}catch{}
    renderControls();await load(false);
  }catch(e){status.textContent='Preview launcher failed';log(run,'error',e.stack||e.message);}
})();
