const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const code=fs.readFileSync(path.join(__dirname,'..','we_property_bridge.js'),'utf8');
const storage=new Map();
function context(mode){
 const c=vm.createContext({});c.window=c;
 if(mode==='blocked')Object.defineProperty(c,'localStorage',{get(){throw Error('blocked');}});
 else c.localStorage={getItem:k=>storage.get(k)??null,setItem:(k,v)=>storage.set(k,v)};
 vm.runInContext(code,c);return c;
}
let c=context();const listener=c.wallpaperPropertyListener;
listener.applyUserProperties({a:{value:1}});listener.applyUserProperties({b:{value:2}});
let received=[];c.__MCLD_WE_BRIDGE__.connect({applyGeneralProperties:p=>received.push(['g',p]),applyUserProperties:p=>received.push(['u',p])});
assert.equal(received[0][1].a.value,1);assert.equal(received[0][1].b.value,2);
listener.applyUserProperties({a:{value:3}});assert.equal(received.at(-1)[1].a.value,3);
vm.runInContext(code,c);assert.strictEqual(c.wallpaperPropertyListener,listener);
c=context();received=[];c.__MCLD_WE_BRIDGE__.connect({applyGeneralProperties(){},applyUserProperties:p=>received.push(p)});
assert.equal(received[0].a.value,3);assert.equal(received[0].b.value,2);
// Invalid cache JSON does not prevent fresh properties reaching the app.
storage.set('mcld_we_user_props_cache','{bad');c=context();received=[];
c.__MCLD_WE_BRIDGE__.connect({applyGeneralProperties(){},applyUserProperties:p=>received.push(p)});
assert.equal(received.length,0);c.wallpaperPropertyListener.applyUserProperties({fresh:{value:true}});assert.equal(received.length,1);
// Disabled localStorage still permits buffering/replay and normal live updates.
c=context('blocked');c.wallpaperPropertyListener.applyGeneralProperties({fps:60});received=[];
c.__MCLD_WE_BRIDGE__.connect({applyGeneralProperties:p=>received.push(p),applyUserProperties(){}});
assert.equal(received[0].fps,60);c.wallpaperPropertyListener.applyGeneralProperties({fps:30});assert.equal(received.at(-1).fps,30);
// A failing receiver must not replace the last successfully stored properties.
c=context();c.__MCLD_WE_BRIDGE__.connect({applyGeneralProperties(){},applyUserProperties(){}});
c.wallpaperPropertyListener.applyUserProperties({fresh:{value:false}});
c.__MCLD_WE_BRIDGE__.connect({applyGeneralProperties(){},applyUserProperties(p){if(p.fail)throw Error('apply failed');}});
assert.throws(()=>c.wallpaperPropertyListener.applyUserProperties({fail:{value:true}}),/apply failed/);
assert.equal(JSON.parse(storage.get('mcld_we_user_props_cache')).fail,undefined);
console.log('PASS: merge/replay, stable listener, restart, corrupt/blocked storage, and successful-application cache semantics.');
