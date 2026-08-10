/* =================================================================
 * build_meshes.mjs — Marketing-GLB → simulationsfertiges Visual-GLB
 *
 * Erkenntnisse aus der Analyse:
 *  - Struktur-Meshes (_SO1/_SO2/_SC_/_TR1/_TR2/_TR3/Werkzeugflansch):
 *    Vertexdaten liegen LOKAL bereits im Roboter-Basisframe
 *    (Z-up, Millimeter, exakte URDF-Nullpose). Ihre World-Matrizen
 *    enthalten zusätzlich eine leicht ausgelenkte Marketing-Pose
 *    → lokale Daten pur verwenden, ÷1000.
 *  - Deko-Meshes (P011142/P011145/Logo): lokal klein, per Transform
 *    in der POSIERTEN Szene platziert. Rückrechnung in die Nullpose:
 *    v_mm = inv(WM_Struktur[trägerLink]) · WM_deko · v_local
 *  - Die BG-Hierarchie ist die Kinematik: BG01..BG05 = link1..link5,
 *    Werkzeugflansch = link6, alles außerhalb = link0.
 * ================================================================= */
import { NodeIO, Document } from '@gltf-transform/core';
import { weld, simplify, dedup, prune, joinPrimitives } from '@gltf-transform/functions';
import { MeshoptSimplifier } from 'meshoptimizer';
import { mat4, vec4 } from 'gl-matrix';

const STRUCT = [['Werkzeugflansch', 6], ['_TR3', 5], ['_TR2', 4], ['_TR1', 3], ['_SC_', 2], ['_SO2', 1], ['_SO1', 0]];
const structLink = (name) => { for (const [k, v] of STRUCT) if ((name || '').includes(k)) return v; return null; };
const bgLink = (name) => {
  const n = name || '';
  if (n.includes('Werkzeugflansch')) return 6;
  for (let i = 5; i >= 1; i--) if (n.includes('BG0' + i)) return i;
  return null;
};

async function build(srcPath, outPath) {
  const io = new NodeIO();
  const src = await io.read(srcPath);
  const scene = src.getRoot().listScenes()[0];

  // Pass 1: Struktur-Worldmatrix je Link (Pose-Referenz) einsammeln
  const SWM = new Array(7).fill(null);
  const collect = (node) => {
    const sl = structLink(node.getName());
    if (node.getMesh() && sl !== null && !SWM[sl]) SWM[sl] = mat4.clone(node.getWorldMatrix());
    for (const c of node.listChildren()) collect(c);
  };
  for (const n of scene.listChildren()) collect(n);
  const missing = SWM.map((m, i) => m ? null : i).filter(v => v !== null);
  if (missing.length) throw new Error('Keine Struktur-Referenz für Links: ' + missing);

  // Ziel-Dokument
  const dst = new Document();
  const buffer = dst.createBuffer();
  const dScene = dst.createScene('robot');
  const linkNodes = [];
  for (let i = 0; i <= 6; i++) { const n = dst.createNode('link' + i); dScene.addChild(n); linkNodes.push(n); }
  const matCache = new Map();
  const cloneMaterial = (m) => {
    if (!m) return null;
    if (matCache.has(m)) return matCache.get(m);
    const d = dst.createMaterial(m.getName());
    d.setBaseColorFactor(m.getBaseColorFactor());
    d.setMetallicFactor(m.getMetallicFactor());
    d.setRoughnessFactor(m.getRoughnessFactor());
    d.setEmissiveFactor(m.getEmissiveFactor());
    d.setAlphaMode(m.getAlphaMode()); d.setAlpha(m.getAlpha());
    d.setDoubleSided(m.getDoubleSided());
    matCache.set(m, d); return d;
  };

  // Pass 2: Meshes kopieren
  const perLinkMat = Array.from({ length: 7 }, () => new Map());
  const dekoStats = { count: 0, min: [1e9, 1e9, 1e9], max: [-1e9, -1e9, -1e9] };
  const T = mat4.create(), N3 = mat4.create(), IDENT = mat4.create();
  const copyNode = (node, linkCtx) => {
    const ctx = bgLink(node.getName()) ?? linkCtx;
    const mesh = node.getMesh();
    if (mesh) {
      const sl = structLink(node.getName());
      const linkId = sl ?? ctx;
      const isStruct = sl !== null;
      if (isStruct) mat4.copy(T, IDENT);
      else { mat4.invert(T, SWM[linkId]); mat4.multiply(T, T, node.getWorldMatrix()); }
      mat4.invert(N3, T); mat4.transpose(N3, N3);
      const flip = mat4.determinant(T) < 0;
      for (const prim of mesh.listPrimitives()) {
        const sPos = prim.getAttribute('POSITION');
        if (!sPos) continue;
        const count = sPos.getCount();
        const pos = new Float32Array(count * 3);
        const v = vec4.create();
        for (let i = 0; i < count; i++) {
          sPos.getElement(i, v); v[3] = 1;
          if (!isStruct) vec4.transformMat4(v, v, T);
          pos[i * 3] = v[0] / 1000; pos[i * 3 + 1] = v[1] / 1000; pos[i * 3 + 2] = v[2] / 1000;
          if (!isStruct) for (let a = 0; a < 3; a++) {
            dekoStats.min[a] = Math.min(dekoStats.min[a], pos[i * 3 + a]);
            dekoStats.max[a] = Math.max(dekoStats.max[a], pos[i * 3 + a]);
          }
        }
        if (!isStruct) dekoStats.count++;
        const dPrim = dst.createPrimitive();
        dPrim.setAttribute('POSITION', dst.createAccessor().setType('VEC3').setArray(pos).setBuffer(buffer));
        const sNrm = prim.getAttribute('NORMAL');
        if (sNrm) {
          const nrm = new Float32Array(count * 3);
          const n = vec4.create();
          for (let i = 0; i < count; i++) {
            sNrm.getElement(i, n); n[3] = 0;
            if (!isStruct) vec4.transformMat4(n, n, N3);
            const l = Math.hypot(n[0], n[1], n[2]) || 1;
            nrm[i * 3] = n[0] / l; nrm[i * 3 + 1] = n[1] / l; nrm[i * 3 + 2] = n[2] / l;
          }
          dPrim.setAttribute('NORMAL', dst.createAccessor().setType('VEC3').setArray(nrm).setBuffer(buffer));
        }
        const sIdx = prim.getIndices();
        let idx;
        if (sIdx) { idx = new Uint32Array(sIdx.getCount()); for (let i = 0; i < idx.length; i++) idx[i] = sIdx.getScalar(i); }
        else { idx = new Uint32Array(count); for (let i = 0; i < count; i++) idx[i] = i; }
        if (flip) for (let i = 0; i + 2 < idx.length; i += 3) { const t = idx[i + 1]; idx[i + 1] = idx[i + 2]; idx[i + 2] = t; }
        dPrim.setIndices(dst.createAccessor().setType('SCALAR').setArray(idx).setBuffer(buffer));
        const mat = cloneMaterial(prim.getMaterial());
        if (mat) dPrim.setMaterial(mat);
        const key = mat ?? '__none__';
        const bucket = perLinkMat[linkId];
        if (!bucket.has(key)) bucket.set(key, []);
        bucket.get(key).push(dPrim);
      }
    }
    for (const c of node.listChildren()) copyNode(c, ctx);
  };
  for (const n of scene.listChildren()) copyNode(n, 0);

  for (let i = 0; i <= 6; i++) {
    const mesh = dst.createMesh('link' + i + '_mesh');
    for (const [, prims] of perLinkMat[i]) mesh.addPrimitive(prims.length > 1 ? joinPrimitives(prims) : prims[0]);
    linkNodes[i].setMesh(mesh);
  }

  await MeshoptSimplifier.ready;
  await dst.transform(
    weld(),
    simplify({ simplifier: MeshoptSimplifier, ratio: 0.4, error: 0.0015 }),
    dedup(),
    prune(),
  );
  await io.write(outPath, dst);

  // Verifikation
  const chk = await io.read(outPath);
  const fs = await import('node:fs');
  const { size } = await fs.promises.stat(outPath);
  console.log(`== ${outPath}  ${(size / 1e6).toFixed(2)} MB`);
  console.log('   Deko-Primitives:', dekoStats.count,
    'AABB:[' + dekoStats.min.map(v => +v.toFixed(3)) + ']..[' + dekoStats.max.map(v => +v.toFixed(3)) + ']');
  for (const node of chk.getRoot().listScenes()[0].listChildren()) {
    const mesh = node.getMesh(); if (!mesh) continue;
    const mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
    let tris = 0;
    for (const p of mesh.listPrimitives()) {
      const a = p.getAttribute('POSITION');
      const pmn = a.getMin([]), pmx = a.getMax([]);
      for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], pmn[k]); mx[k] = Math.max(mx[k], pmx[k]); }
      tris += p.getIndices().getCount() / 3;
    }
    console.log(`   ${node.getName()} tris:${Math.round(tris)} min:[${mn.map(v => +v.toFixed(4))}] max:[${mx.map(v => +v.toFixed(4))}]`);
  }
}
const args = process.argv.slice(2);
await build(args[0], args[1]);
