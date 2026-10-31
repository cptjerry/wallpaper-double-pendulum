
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
export { createThemeState, ensureThemeState, applyThemeState, updateThemeStateFromProperties, captureManualPalette };
