const fs=require('fs'),vm=require('vm'),assert=require('assert/strict'),path=require('path');
const packageRoot=path.resolve(__dirname,'..','..');
const project=fs.readFileSync(path.join(packageRoot,'wallpaper/project.json'),'utf8'),html=fs.readFileSync(path.join(packageRoot,'wallpaper/index.html'),'utf8');
function element(){return {children:[],value:'',checked:false,style:{},textContent:'',parentElement:{scrollTop:0,scrollHeight:100},classList:{data:new Set(),add(x){this.data.add(x);},remove(x){this.data.delete(x);},toggle(x){if(this.data.has(x)){this.data.delete(x);return false;}this.data.add(x);return true;}},setAttribute(k,v){this[k]=v;},appendChild(c){this.children.push(c);},addEventListener(){},checkValidity(){return true;}};}
const ids=[...fs.readFileSync(path.join(packageRoot,'browser_test_launcher/index.html'),'utf8').matchAll(/id="([^"]+)"/g)].map(m=>m[1]);
const els=Object.fromEntries(ids.map(id=>[id,element()]));els.fps.value='60';els.delay.value='0';
const messages=[],child={wallpaperPropertyListener:{applyGeneralProperties:p=>messages.push(['g',p]),applyUserProperties:p=>messages.push(['u',p])}};
els.preview.contentWindow=child;
const store=new Map();const requested=[];
const c=vm.createContext({console,URL,Response,setTimeout,clearTimeout,setInterval(){},location:new URL('http://localhost:8765/nested/project/browser_test_launcher/index.html'),navigator:{clipboard:{writeText:async()=>{}}},
 document:{getElementById:id=>els[id],createElement:element,body:element()},
 localStorage:{getItem:k=>store.get(k)??null,setItem:(k,v)=>store.set(k,v),removeItem:k=>store.delete(k)},
 addEventListener(){},fetch:async url=>{requested.push(String(url));return new Response(String(url).endsWith('project.json')?project:html);},
});c.window=c;
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
 vm.runInContext(fs.readFileSync(path.join(packageRoot,'browser_test_launcher/launcher.js'),'utf8'),c);
 await sleep(50);
 assert(requested.includes('http://localhost:8765/nested/project/wallpaper/project.json'));
 assert(els.preview.srcdoc.includes('<base href="http://localhost:8765/nested/project/wallpaper/">'));
 assert(els.preview.srcdoc.includes('src="http://localhost:8765/nested/project/browser_test_launcher/emulate_we.js"'));
 c.__wallpaperTestHost.listenerReady(child,1);assert.equal(messages[0][1].fps,60);assert.equal(messages[1][1].hudposition.value,1);
 els.top.onclick();assert.equal(messages.at(-1)[1].controlpitch.value,90);
 els.bottom.onclick();assert.equal(messages.at(-1)[1].controlpitch.value,-90);
 els.mouse.onclick();assert.equal(messages.at(-1)[1].controlmode.value,0);
 const before=messages.length;await els.cached.onclick();c.__wallpaperTestHost.listenerReady(child,2);assert.equal(messages.length,before);
 els.delay.value='20';await els.reload.onclick();c.__wallpaperTestHost.listenerReady(child,3);assert.equal(messages.length,before);await sleep(40);assert.equal(messages.length,before+2);
 
 const schema=JSON.parse(project).general.properties;
 const rendered=vm.runInContext('Object.keys(inputs)',c);
 assert.equal(rendered.length,Object.keys(schema).length);
 assert(Object.keys(schema).every(k=>rendered.includes(k)));
 assert.equal(els.controls.children.length,4);
 els.testsTab.onclick();assert.equal(els.settingsPanel.hidden,true);assert.equal(els.testsPanel.hidden,false);
 els.settingsTab.onclick();assert.equal(els.settingsPanel.hidden,false);
 els.settingsToggle.onclick();assert.equal(els.settingsToggle.textContent,'Show settings');
 els.settingsToggle.onclick();assert.equal(els.settingsToggle.textContent,'Hide settings');
 els.top.onclick();assert.equal(vm.runInContext('inputs.controlpitch.row.hidden',c),false);
 els.mouse.onclick();assert.equal(vm.runInContext('inputs.controlpitch.row.hidden',c),true);
 // Playback group reset and global reset both use authoritative project defaults.
 els.controls.children[0].children[0].children[0].onclick({preventDefault(){},stopPropagation(){}});
 assert.equal(messages.at(-1)[1].playbackspeed.value,schema.playbackspeed.value);
 els.reset.onclick();for(const [key,spec] of Object.entries(schema))assert.equal(messages.at(-1)[1][key].value,spec.value);
 function flatten(el){return [el,...el.children.flatMap(flatten)];}
 const sliders=flatten(els.controls).filter(el=>el.type==='range');
 assert.equal(sliders.length,Object.values(schema).filter(s=>s.type==='slider').length);
 const speedSlider=sliders[0];speedSlider.value='0.8';speedSlider.oninput();assert.equal(messages.at(-1)[1].playbackspeed.value,0.8);
 console.log('PASS: all '+rendered.length+' project properties, 4 settings groups, sliders, conditional options, tabs, resets, live callbacks, and nested static paths.');
 
})().catch(e=>{console.error(e);process.exitCode=1});
