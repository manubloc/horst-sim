/* ============================================================
 * horstSIM – RobotMeshManager
 * Blendet die echten CAD-Meshes (aus den fruitcore-GLBs
 * aufbereitet, siehe tools/build_meshes.mjs) über die
 * MuJoCo-Körper. Physik/Kollision läuft weiter auf den
 * Primitiv-Geomen (group 3, im Renderer ausgeblendet).
 *
 * GLB-Konvention: 7 Nodes "link0".."link6", Vertices in Metern
 * im Roboter-Basisframe bei URDF-Nullpose.
 * Frame-Sync:  M_outer = [R|p] · T(-t0)  =  [R | p − R·t0]
 * mit t0 = Linkursprung (Basisframe, Nullpose) aus der Kinematik.
 * ============================================================ */

import * as THREE from 'three';
import { GLTFLoader } from '../vendor/three/GLTFLoader.js';

const T0 = [
  [0, 0, 0],
  [0, 0, 0.0925],
  [0, 0, 0.3435],
  [0, 0, 0.6175],
  [0.0855, 0, 0.673],
  [0.2672, 0, 0.673],
  [0.3314, 0, 0.673],
];
const LINKNAMES = ['horst_basis', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'];

export class RobotMeshManager {
  constructor(scene3) {
    this.scene3 = scene3;
    this._gltfPromise = null;
    this.groups = [];          // { outer, bodyId, t0 }
    this.geomIds = new Set();  // Geom-IDs der Roboterkörper (für Renderer-Skip)
    this.active = true;        // UI-Schalter
    this.available = false;    // GLB geladen und Roboter in der Szene
  }

  setActive(on) {
    this.active = on;
    for (const g of this.groups) g.outer.visible = on;
  }

  clear() {
    for (const g of this.groups) this.scene3.remove(g.outer);
    this.groups = [];
    this.geomIds.clear();
    this.available = false;
  }

  _load(url) {
    if (!this._gltfPromise) {
      const loader = new GLTFLoader();
      this._gltfPromise = globalThis.__HORST_GLB
        ? loader.parseAsync(globalThis.__HORST_GLB, '')
        : loader.loadAsync(url);
    }
    return this._gltfPromise;
  }

  /** Nach jedem Modell-Load aufrufen (fire-and-forget). */
  async rebuild(engine, url = 'meshes/horst600_visual.glb') {
    this.clear();
    if (!engine.loaded) return;
    const m = engine.model;
    const byName = new Map();
    for (let i = 0; i < m.nbody; i++) byName.set(m.body(i).name, i);
    const prefixes = [];
    for (const [name] of byName) {
      if (name && name.endsWith('horst_basis')) prefixes.push(name.slice(0, -'horst_basis'.length));
    }
    if (!prefixes.length) return;
    let gltf;
    try { gltf = await this._load(url); }
    catch (e) { console.warn('CAD-Meshes nicht ladbar – Primitive bleiben aktiv.', e); return; }
    const linkNodes = [];
    for (let i = 0; i <= 6; i++) linkNodes.push(gltf.scene.getObjectByName('link' + i));

    for (const p of prefixes) {
      for (let i = 0; i <= 6; i++) {
        const bodyId = byName.get(p + LINKNAMES[i]);
        if (bodyId === undefined || !linkNodes[i]) continue;
        const outer = new THREE.Group();
        outer.matrixAutoUpdate = false;
        const inst = linkNodes[i].clone(true);
        inst.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
        outer.add(inst);
        outer.visible = this.active;
        this.scene3.add(outer);
        this.groups.push({ outer, bodyId, t0: T0[i] });
      }
    }
    // Geoms aller Roboterkörper einsammeln – der Renderer blendet genau
    // diese Primitive aus, solange die CAD-Meshes aktiv sind.
    const robotBodies = new Set(this.groups.map(g => g.bodyId));
    for (let gid = 0; gid < m.ngeom; gid++) {
      if (robotBodies.has(m.geom_bodyid[gid])) this.geomIds.add(gid);
    }
    this.available = this.groups.length > 0;
  }

  /** Pro Frame nach mj_step/forward. */
  sync(engine) {
    if (!this.available || !this.active || !engine.loaded) return;
    const xpos = engine.data.xpos, r = engine.data.xmat;
    for (const g of this.groups) {
      const b3 = g.bodyId * 3, b9 = g.bodyId * 9, t = g.t0;
      const px = xpos[b3]     - (r[b9]     * t[0] + r[b9 + 1] * t[1] + r[b9 + 2] * t[2]);
      const py = xpos[b3 + 1] - (r[b9 + 3] * t[0] + r[b9 + 4] * t[1] + r[b9 + 5] * t[2]);
      const pz = xpos[b3 + 2] - (r[b9 + 6] * t[0] + r[b9 + 7] * t[1] + r[b9 + 8] * t[2]);
      g.outer.matrix.set(
        r[b9],     r[b9 + 1], r[b9 + 2], px,
        r[b9 + 3], r[b9 + 4], r[b9 + 5], py,
        r[b9 + 6], r[b9 + 7], r[b9 + 8], pz,
        0, 0, 0, 1);
      g.outer.matrixWorldNeedsUpdate = true;
    }
  }
}
