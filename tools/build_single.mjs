/* =================================================================
 * build_single.mjs — Doppelklick-Einzeldatei horstsim_single.html
 * Bundelt die App (IIFE) und bettet WASM, CAD-GLB, Beispiel-XMLs
 * und CSS als Base64/Strings direkt ins HTML ein. Läuft dadurch
 * auch über file:// ohne lokalen Server.
 * ================================================================= */
import esbuild from 'esbuild';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { createHash } from 'node:crypto';

const sha = (b) => createHash('sha256').update(b).digest('hex');
const b64 = (p) => readFileSync(p).toString('base64');

const bundle = await esbuild.build({
  entryPoints: ['app/js/main.js'],
  bundle: true, minify: true, write: false,
  format: 'iife', platform: 'browser', target: 'es2022',
  alias: {
    three: './app/vendor/three/three.module.min.js',
    module: './tools/shims/module.js',
  },
  define: { 'import.meta.url': JSON.stringify('file:///horstsim/') },
  logLevel: 'silent',
});
const js = bundle.outputFiles[0].text;

const wasm = readFileSync('app/vendor/mujoco/mujoco.wasm');
const glb  = readFileSync('app/meshes/horst600_visual.glb');
const css  = readFileSync('app/css/app.css', 'utf8');
const files = {
  'models/humanoid.xml':    readFileSync('app/models/humanoid.xml', 'utf8'),
  'models/horst600_g2.xml': readFileSync('app/models/horst600_g2.xml', 'utf8'),
};

const safeScript = (s) => s.replace(/<\//g, '<\\/');
const payload = `<script>
(function(){
  function u8(b){const s=atob(b);const a=new Uint8Array(s.length);for(let i=0;i<s.length;i++)a[i]=s.charCodeAt(i);return a;}
  globalThis.__HORST_WASM = u8("${wasm.toString('base64')}");
  globalThis.__HORST_GLB  = u8("${glb.toString('base64')}").buffer;
  globalThis.__HORST_FILES = ${safeScript(JSON.stringify(files))};
})();
<\/script>`;

let html = readFileSync('app/index.html', 'utf8');
html = html.replace('<link rel="stylesheet" href="css/app.css">', () => `<style>${css}</style>`);
html = html.replace(/<script type="importmap">.*?<\/script>\n?/s, '');
html = html.replace('<script type="module" src="js/main.js"></script>',
  () => `${payload}\n<script>${safeScript(js)}<\/script>`);
html = html.replace('<title>horstSIM · Simulation</title>',
  '<title>horstSIM · Simulation (Einzeldatei)</title>');

mkdirSync('dist', { recursive: true });
writeFileSync('dist/horstsim_single.html', html);

/* Verifikation: eingebettete Base64-Blöcke rückextrahieren und hashen */
const out = readFileSync('dist/horstsim_single.html', 'utf8');
const mw = out.match(/__HORST_WASM = u8\("([A-Za-z0-9+/=]+)"\)/);
const mg = out.match(/__HORST_GLB  = u8\("([A-Za-z0-9+/=]+)"\)/);
const okW = mw && sha(Buffer.from(mw[1], 'base64')) === sha(wasm);
const okG = mg && sha(Buffer.from(mg[1], 'base64')) === sha(glb);
const markers = ['horst_basis', 'wasmBinary', '__HORST_FILES', 'horst600_visual', 'CAD-Meshes'].map(k => `${k}:${out.includes(k) ? 'ok' : 'FEHLT'}`);
console.log(`horstsim_single.html  ${(out.length / 1e6).toFixed(2)} MB`);
console.log('WASM-Roundtrip:', okW ? 'OK ' + sha(wasm).slice(0, 12) : 'FEHLER');
console.log('GLB-Roundtrip: ', okG ? 'OK ' + sha(glb).slice(0, 12) : 'FEHLER');
console.log('Marker:', markers.join('  '));
if (!okW || !okG) process.exit(1);
