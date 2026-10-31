"""Build this project's classic-script bundles and gzip inline release.
Run: python build_release.py
No npm bundler, runtime eval, external JS, or WASM compilation required.
"""
from pathlib import Path
import base64, gzip, json, re, shutil, hashlib

ROOT = Path(__file__).resolve().parent
OUT = ROOT.parent / 'wallpaper'
BUILD = ROOT.parent / 'build'
MODULES = ['we_property_bridge.js','runtime_core.js','scene_renderer.js','worker_stream.js',
           'persist_state.js','hud_overlay.js','theme_state.js','main.js']

def compressed(data):
    return base64.b64encode(gzip.compress(data, compresslevel=9, mtime=0)).decode('ascii')

def bundle():
    # Tailored to the project's named imports/exports plus side-effect imports.
    # Unsupported static syntax fails the build instead of producing a bad page.
    definitions = []
    for name in MODULES:
        text = (ROOT / name).read_text(encoding='utf-8')
        def named(m):
            target = m[2].removeprefix('./')
            assert target in MODULES, target
            return 'const {' + m[1] + '} = __require(' + json.dumps(target) + ');'
        text = re.sub(r"^import\s*\{([^}]+)\}\s*from\s*['\"]([^'\"]+)['\"];?", named, text, flags=re.M)
        def side(m):
            target = m[1].removeprefix('./')
            assert target in MODULES, target
            return '__require(' + json.dumps(target) + ');'
        text = re.sub(r"^import\s*['\"]([^'\"]+)['\"];?", side, text, flags=re.M)
        text = re.sub(r'export\s*\{([^}]+)\};?', r'Object.assign(exports, {\1});', text)
        assert not re.search(r'^\s*(import|export)\s', text, re.M), name
        definitions.append('__modules[' + json.dumps(name) + '] = function(exports, __require){\n' + text + '\n};')
    return ('(function(){"use strict";\n' + (ROOT/'shell_renderer.js').read_text() + '\n'
        + 'const __modules=Object.create(null), __instances=Object.create(null);\n'
        + '\n'.join(definitions) + '\n'
        + 'function __require(id){if(__instances[id])return __instances[id];'
          'if(!__modules[id])throw new Error("Missing bundled module: "+id);'
          'const exports=__instances[id]=Object.create(null);__modules[id](exports,__require);return exports;}\n'
        + '__require("main.js");\n})();\n')

DECODE = r'''
async function __decodeGzip(encoded) {
  if (typeof DecompressionStream !== 'function') throw new Error('This browser does not support gzip DecompressionStream');
  const bytes = Uint8Array.from(atob(encoded), c => c.charCodeAt(0));
  return new Uint8Array(await new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());
}
'''

def main():
    OUT.mkdir(exist_ok=True); BUILD.mkdir(exist_ok=True)
    wasm = (ROOT/'mcld_solver.wasm').read_bytes()
    solver = (ROOT/'mcld_solver.js').read_text()
    assert solver.count('var createMcldModule=') == 1
    solver = solver.replace('var createMcldModule=', 'self.createMcldModule=', 1)
    sim = (ROOT/'simulation_worker.js').read_text()
    assert sim.count("importScripts('./mcld_solver.js');") == 1
    sim = sim.replace("importScripts('./mcld_solver.js');", '// Factory and WASM supplied by the inline worker wrapper.', 1)
    worker = (DECODE + '\n' + solver + '\n'
        + 'const __originalFactory=self.createMcldModule;\n'
        + 'let __wasmBytesPromise;\n'
        + 'self.createMcldModule=async function(options={}){\n'
        + 'if(!__wasmBytesPromise)__wasmBytesPromise=__decodeGzip(' + json.dumps(compressed(wasm)) + ');\n'
        + 'return __originalFactory(Object.assign({},options,{wasmBinary:await __wasmBytesPromise}));\n};\n'
        + sim)
    app = bundle()
    (BUILD/'app_bundle.js').write_text(app, encoding='utf-8')
    (BUILD/'worker_bundle.js').write_text(worker, encoding='utf-8')
    bootstrap = r'''
(function(){
  'use strict';
  function fail(error){
    const el=document.getElementById('status');
    if(el)el.textContent='Bootstrap failed: '+(error && error.message ? error.message : String(error));
    document.body.classList.remove('boot-hidden');
    document.body.classList.add('boot-failed');
    console.error(error);
  }
  __DECODER__
  (async function(){
    const text=new TextDecoder();
    // Main startup must wait until the Worker URL exists. WASM bytes remain
    // embedded in the Worker and are passed directly into the Emscripten factory.
    const workerCode=text.decode(await __decodeGzip(document.getElementById('workerPayload').textContent.trim()));
    const workerUrl=URL.createObjectURL(new Blob([workerCode],{type:'text/javascript'}));
    window.__MCLD_INLINE_WORKER_URL__=workerUrl;
    window.addEventListener('pagehide',()=>URL.revokeObjectURL(workerUrl),{once:true});
    const appCode=text.decode(await __decodeGzip(document.getElementById('appPayload').textContent.trim()));
    let scriptError=null;
    const onError=event=>{scriptError=event.error||new Error(event.message||'Bundled app failed');};
    window.addEventListener('error',onError);
    try {
      const script=document.createElement('script');
      script.textContent=appCode;
      document.body.appendChild(script);
      if(scriptError)throw scriptError;
    } finally { window.removeEventListener('error',onError); }
  })().catch(fail);
})();
'''.replace('__DECODER__', DECODE)
    bridge = (ROOT/'we_property_bridge.js').read_text()
    assert '</script' not in bridge.lower() and '</script' not in bootstrap.lower()
    # Keep the exact expected DOM; the early bridge runs before async decoding.
    html = ('<!doctype html>\n<html lang="en"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width,initial-scale=1">'
        '<title>Wallpaper Double Pendulum</title><style>'+(ROOT/'styles.css').read_text()+'</style></head>\n'
        '<body class="boot-hidden"><div id="bootVeil" aria-hidden="true"></div>'
        '<canvas id="stage"></canvas><div id="hud"><div id="hudTitle">Wallpaper Double Pendulum</div>'
        '<pre id="status">Loading…</pre></div>\n<script>'+bridge+'</script>\n'
        '<script id="workerPayload" type="application/octet-stream">'+compressed(worker.encode())+'</script>\n'
        '<script id="appPayload" type="application/octet-stream">'+compressed(app.encode())+'</script>\n'
        '<script>'+bootstrap+'</script>\n</body></html>\n')
    (OUT/'index.html').write_text(html, encoding='utf-8')
    for name in ['project.json','preview.jpg','wallpaper_config.json','wallpaper_config.annotated.en.jsonc']:
        shutil.copyfile(ROOT/name, OUT/name)
    report = {'app_bytes':len(app.encode()), 'worker_bytes':len(worker.encode()),
              'wasm_bytes':len(wasm), 'html_bytes':len(html.encode()),
              'wasm_sha256':hashlib.sha256(wasm).hexdigest(),
              'app_has_static_import_export':bool(re.search(r'^\s*(import|export)\s',app,re.M)),
              'worker_has_global_factory':'self.createMcldModule=' in worker,
              'worker_has_importScripts_call':bool(re.search(r'^\s*importScripts\(',worker,re.M)),
              'runtime_files':sorted(p.name for p in OUT.iterdir() if p.is_file())}
    (BUILD/'bundle_diagnostics.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))

if __name__ == '__main__': main()
