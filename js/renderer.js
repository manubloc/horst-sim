/* ============================================================
 * horstSIM – SceneRenderer
 * Rendert die von mjv_updateScene erzeugte abstrakte Szene
 * (inkl. Dekor-Geome wie Kontaktpunkte, Kraftpfeile, Gelenke)
 * mit three.js. Geometrien werden gepoolt und gecacht.
 * ============================================================ */

import * as THREE from 'three';
import { RobotMeshManager } from './robotmesh.js';

const T = {
  PLANE: 0, HFIELD: 1, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4,
  CYLINDER: 5, BOX: 6, MESH: 7,
  ARROW: 100, ARROW1: 101, ARROW2: 102, LINE: 103,
};

export class SceneRenderer {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.12;
    container.appendChild(this.renderer.domElement);
    this.canvas = this.renderer.domElement;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x031e20);
    this.scene.fog = new THREE.FogExp2(0x031e20, 0.035);

    this.camera = new THREE.PerspectiveCamera(42, 1, 0.01, 200);
    this.camera.up.set(0, 0, 1);

    this._buildLights();

    // Einheitsgeometrien (werden per Matrix skaliert)
    const cyl = new THREE.CylinderGeometry(1, 1, 2, 24); cyl.rotateX(Math.PI / 2);
    this.unit = {
      sphere: new THREE.SphereGeometry(1, 24, 18),
      box: new THREE.BoxGeometry(2, 2, 2),
      cyl,
      plane: new THREE.PlaneGeometry(2, 2),
    };
    this.geoCache = new Map();   // key -> BufferGeometry (Kapseln, Pfeile, Linien)
    this.meshCache = new Map();  // dataid -> BufferGeometry (Modell-Meshes)
    this.pool = [];              // wiederverwendete THREE.Mesh
    this.wireframe = false;

    this.robotMesh = new RobotMeshManager(this.scene);

    this.checker = this._makeChecker();
    this._m4 = new THREE.Matrix4();
    this._s4 = new THREE.Matrix4();
    this.resize();
  }

  _buildLights() {
    this.scene.add(new THREE.HemisphereLight(0x9be8ea, 0x02181a, 0.6));
    const sun = new THREE.DirectionalLight(0xffffff, 2.4);
    sun.position.set(2.6, 1.8, 4.2);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    const c = sun.shadow.camera;
    c.left = -4; c.right = 4; c.top = 4; c.bottom = -4; c.far = 20;
    sun.shadow.bias = -0.0002;
    this.sun = sun;
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0x5adbd6, 0.5);
    fill.position.set(-3.2, -2.4, 2.2);
    this.scene.add(fill);
  }

  _makeChecker() {
    const s = 512, c = document.createElement('canvas');
    c.width = c.height = s;
    const g = c.getContext('2d');
    g.fillStyle = '#052d30'; g.fillRect(0, 0, s, s);
    g.fillStyle = '#04282b';
    for (let y = 0; y < 8; y++) for (let x = 0; x < 8; x++) if ((x + y) % 2) g.fillRect(x * s / 8, y * s / 8, s / 8, s / 8);
    g.strokeStyle = 'rgba(90,219,214,0.10)'; g.lineWidth = 2;
    for (let i = 0; i <= 8; i++) {
      g.beginPath(); g.moveTo(i * s / 8, 0); g.lineTo(i * s / 8, s); g.stroke();
      g.beginPath(); g.moveTo(0, i * s / 8); g.lineTo(s, i * s / 8); g.stroke();
    }
    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = 8;
    return tex;
  }

  onModelChanged() {
    for (const g of this.meshCache.values()) g.dispose();
    this.meshCache.clear();
    for (const g of this.geoCache.values()) g.dispose();
    this.geoCache.clear();
  }

  setShadows(on) { this.renderer.shadowMap.enabled = on; this.sun.castShadow = on;
    this.pool.forEach(m => { m.material.needsUpdate = true; }); }
  setWireframe(on) { this.wireframe = on; }

  resize() {
    const w = this.container.clientWidth || 800, h = this.container.clientHeight || 600;
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  syncCamera(engine) {
    const cam = engine.cam, m = engine.model;
    const az = cam.azimuth * Math.PI / 180, el = cam.elevation * Math.PI / 180;
    const fx = Math.cos(el) * Math.cos(az), fy = Math.cos(el) * Math.sin(az), fz = Math.sin(el);
    const la = cam.lookat;
    this.camera.position.set(la[0] - fx * cam.distance, la[1] - fy * cam.distance, la[2] - fz * cam.distance);
    this.camera.lookAt(la[0], la[1], la[2]);
    this.camera.fov = m ? m.vis.global.fovy : 42;
    this.camera.updateProjectionMatrix();
  }

  _cached(key, build) {
    let g = this.geoCache.get(key);
    if (!g) {
      g = build();
      if (this.geoCache.size > 400) {           // Cache begrenzen
        const first = this.geoCache.keys().next().value;
        this.geoCache.get(first).dispose(); this.geoCache.delete(first);
      }
      this.geoCache.set(key, g);
    }
    return g;
  }

  _capsule(r, hl) {
    return this._cached(`c${r.toFixed(4)}_${hl.toFixed(4)}`, () => {
      const g = new THREE.CapsuleGeometry(r, hl * 2, 6, 16); g.rotateX(Math.PI / 2); return g;
    });
  }

  _arrow(w, len, kind) {
    return this._cached(`a${kind}_${w.toFixed(4)}_${len.toFixed(4)}`, () => {
      const parts = [];
      const shaftR = Math.max(w * 0.35, 1e-4);
      if (kind === 1) { // ohne Spitze
        const s = new THREE.CylinderGeometry(shaftR, shaftR, len, 12);
        s.rotateX(Math.PI / 2); s.translate(0, 0, len / 2); parts.push(s);
      } else {
        const headL = Math.min(len * 0.35, w * 2.2);
        const bodyL = Math.max(len - headL * (kind === 2 ? 2 : 1), 1e-4);
        const s = new THREE.CylinderGeometry(shaftR, shaftR, bodyL, 12);
        s.rotateX(Math.PI / 2); s.translate(0, 0, (kind === 2 ? headL : 0) + bodyL / 2); parts.push(s);
        const h = new THREE.ConeGeometry(w * 0.8, headL, 14);
        h.rotateX(Math.PI / 2); h.translate(0, 0, len - headL / 2); parts.push(h);
        if (kind === 2) {
          const h2 = new THREE.ConeGeometry(w * 0.8, headL, 14);
          h2.rotateX(-Math.PI / 2); h2.translate(0, 0, headL / 2); parts.push(h2);
        }
      }
      return mergeGeoms(parts);
    });
  }

  _line(len) {
    return this._cached(`l${len.toFixed(4)}`, () => {
      const g = new THREE.CylinderGeometry(0.0035, 0.0035, len, 6);
      g.rotateX(Math.PI / 2); g.translate(0, 0, len / 2); return g;
    });
  }

  _modelMesh(engine, dataid) {
    let g = this.meshCache.get(dataid);
    if (g) return g;
    const m = engine.model;
    const va = m.mesh_vertadr[dataid], vn = m.mesh_vertnum[dataid];
    const fa = m.mesh_faceadr[dataid], fn = m.mesh_facenum[dataid];
    const pos = new Float32Array(m.mesh_vert.slice(va * 3, (va + vn) * 3));
    const idx = new Uint32Array(fn * 3);
    const faces = m.mesh_face;
    for (let i = 0; i < fn * 3; i++) idx[i] = faces[fa * 3 + i]; // Indizes sind mesh-lokal
    g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setIndex(new THREE.BufferAttribute(idx, 1));
    g.computeVertexNormals();
    this.meshCache.set(dataid, g);
    return g;
  }

  _poolMesh(i) {
    let mesh = this.pool[i];
    if (!mesh) {
      mesh = new THREE.Mesh(this.unit.sphere, new THREE.MeshStandardMaterial({
        color: 0xffffff, metalness: 0.15, roughness: 0.55,
      }));
      mesh.matrixAutoUpdate = false;
      mesh.castShadow = true; mesh.receiveShadow = true;
      this.pool[i] = mesh;
      this.scene.add(mesh);
    }
    mesh.visible = true;
    return mesh;
  }

  /** Hauptaufruf pro Frame. */
  render(engine) {
    const mj = engine.mujoco;
    if (engine.loaded) {
      mj.mjv_updateScene(engine.model, engine.data, engine.vopt, engine.pert, engine.cam,
        mj.mjtCatBit.mjCAT_ALL.value, engine.scene);
      const scn = engine.scene;
      const geoms = scn.geoms;
      const n = scn.ngeom;
      const hideRobot = this.robotMesh.available && this.robotMesh.active;
      const OBJ_GEOM = mj.mjtObj.mjOBJ_GEOM.value;
      const robotGeoms = this.robotMesh.geomIds;
      try {
        for (let i = 0; i < n; i++) {
          const g = geoms.get(i);
          const mesh = this._poolMesh(i);
          if (hideRobot && g.objtype === OBJ_GEOM && robotGeoms.has(g.objid)) {
            mesh.visible = false;
          } else {
            this._applyGeom(engine, mesh, g);
          }
          g.delete();
        }
      } finally { geoms.delete(); }
      for (let i = n; i < this.pool.length; i++) this.pool[i].visible = false;
      this.robotMesh.sync(engine);
    }
    this.syncCamera(engine);
    this.renderer.render(this.scene, this.camera);
  }

  _applyGeom(engine, mesh, g) {
    const type = g.type, size = g.size, pos = g.pos, mat = g.mat, rgba = g.rgba;
    let geometry = null, sx = 1, sy = 1, sz = 1, isPlane = false;

    switch (type) {
      case T.PLANE: {
        geometry = this.unit.plane; isPlane = true;
        sx = size[0] > 0 ? size[0] : 15; sy = size[1] > 0 ? size[1] : 15; sz = 1;
        break;
      }
      case T.SPHERE: geometry = this.unit.sphere; sx = sy = sz = size[0]; break;
      case T.ELLIPSOID: geometry = this.unit.sphere; sx = size[0]; sy = size[1]; sz = size[2]; break;
      case T.BOX: geometry = this.unit.box; sx = size[0]; sy = size[1]; sz = size[2]; break;
      case T.CYLINDER: geometry = this.unit.cyl; sx = size[0]; sy = size[1] || size[0]; sz = size[2]; break;
      case T.CAPSULE: geometry = this._capsule(size[0], size[2]); break;
      case T.MESH: geometry = this._modelMesh(engine, g.dataid); break;
      case T.ARROW: geometry = this._arrow(size[0], size[2], 0); break;
      case T.ARROW1: geometry = this._arrow(size[0], size[2], 1); break;
      case T.ARROW2: geometry = this._arrow(size[0], size[2], 2); break;
      case T.LINE: geometry = this._line(size[2]); break;
      default: mesh.visible = false; return;   // HFIELD/FLEX/LABEL etc.
    }

    if (mesh.geometry !== geometry) mesh.geometry = geometry;
    this._m4.set(
      mat[0], mat[1], mat[2], pos[0],
      mat[3], mat[4], mat[5], pos[1],
      mat[6], mat[7], mat[8], pos[2],
      0, 0, 0, 1);
    if (sx !== 1 || sy !== 1 || sz !== 1) {
      this._s4.makeScale(sx, sy, sz);
      this._m4.multiply(this._s4);
    }
    mesh.matrix.copy(this._m4);
    mesh.matrixWorldNeedsUpdate = true;

    const mtl = mesh.material;
    mtl.color.setRGB(rgba[0], rgba[1], rgba[2]);
    const alpha = rgba[3];
    const wantTransp = alpha < 0.995;
    if (mtl.transparent !== wantTransp) { mtl.transparent = wantTransp; mtl.needsUpdate = true; }
    mtl.opacity = alpha;
    if (mtl.wireframe !== this.wireframe) { mtl.wireframe = this.wireframe; mtl.needsUpdate = true; }
    const wantMap = isPlane ? this.checker : null;
    if (mtl.map !== wantMap) {
      mtl.map = wantMap;
      if (wantMap) { const rep = Math.max(1, Math.round(sx)); this.checker.repeat.set(rep * 2, rep * 2); }
      mtl.needsUpdate = true;
    }
    mesh.castShadow = type < 100 && !isPlane && alpha > 0.5;
    mesh.receiveShadow = type < 100;
    mesh.visible = true;
  }

  screenshotURL() { return this.canvas.toDataURL('image/png'); }
  captureStream(fps = 30) { return this.canvas.captureStream(fps); }
}

/* Minimaler Geometrie-Merger (ersetzt BufferGeometryUtils). */
function mergeGeoms(list) {
  let vcount = 0, icount = 0;
  for (const g of list) { vcount += g.attributes.position.count; icount += g.index ? g.index.count : g.attributes.position.count; }
  const pos = new Float32Array(vcount * 3), nor = new Float32Array(vcount * 3), idx = new Uint32Array(icount);
  let vo = 0, io = 0;
  for (const g of list) {
    pos.set(g.attributes.position.array, vo * 3);
    if (g.attributes.normal) nor.set(g.attributes.normal.array, vo * 3);
    const gi = g.index ? g.index.array : [...Array(g.attributes.position.count).keys()];
    for (let k = 0; k < gi.length; k++) idx[io + k] = gi[k] + vo;
    io += gi.length; vo += g.attributes.position.count;
    g.dispose();
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  out.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  out.setIndex(new THREE.BufferAttribute(idx, 1));
  return out;
}
