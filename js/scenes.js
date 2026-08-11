/* ============================================================
 * horstSIM – Szenenbibliothek
 * MJCF-Generatoren für den fruitcore HORST600 G2 (pt1) und
 * Beispielszenen. Kinematik exakt aus horst600-g2-pt1-chain.xacro.
 * Geometrie: Primitiv-Approximation (STL-Meshes folgen später).
 * ============================================================ */

export const HORST = {
  teal:  '0 0.655 0.659 1',
  dark:  '0.165 0.165 0.165 1',
  black: '0.067 0.067 0.067 1',
  light: '0.8 0.8 0.8 1',
  lav:   '0.627 0.627 0.69 1',
};

// Gelenkdaten aus horst600-g2-pt1-chain.xacro (Reihenfolge j1..j6)
export const HORST600_JOINTS = [
  { axis: '0 0 1', range: [-2.9845130209, 2.9845130209], vel: 5.2359877560, org: [0, 0, 0.0925] },
  { axis: '0 1 0', range: [-1.7453292520, 2.1293016874], vel: 3.1415926536, org: [0, 0, 0.251] },
  { axis: '0 1 0', range: [-3.6651914292, 1.2566370614], vel: 4.5378560552, org: [0, 0, 0.274] },
  { axis: '1 0 0', range: [-3.0717794835, 3.0717794835], vel: 13.9626340160, org: [0.0855, 0, 0.0555] },
  { axis: '0 1 0', range: [-2.7576202182, 2.7576202182], vel: 13.0899693899, org: [0.1817, 0, 0] },
  { axis: '1 0 0', range: [-5.2359877560, 5.2359877560], vel: 15.7079632679, org: [0.0642, 0, 0] },
];

export const HORST600_HOME = [0, 0.45, -1.5, 0, 1.05, 0];

// Steifere Lageregler: der Schwerkraft-Schleppfehler am TCP sinkt von 16,6 mm
// auf 5,6 mm, gemessen ohne jedes Nachschwingen (Restbewegung 0,000 rad/s).
const KP = [1200, 1200, 750, 180, 150, 75];
const KV = [69, 69, 43, 10.4, 8.7, 4.3];
const FR = [150, 150, 90, 28, 22, 12];
const DAMP = [6, 6, 4, 1.2, 1.0, 0.5];
const ARM = [0.08, 0.08, 0.06, 0.02, 0.02, 0.01];

function v(a) { return a.join(' '); }

/**
 * Erzeugt den <body>-Baum + Aktuator-/Sensor-Blöcke für einen HORST600 G2.
 * @param {object} o  { prefix, pos:[x,y,z], zrot(rad), tcpSite:boolean }
 */
export function horstBody(o = {}) {
  const p = o.prefix ?? '';
  const pos = o.pos ?? [0, 0, 0];
  const zrot = o.zrot ?? 0;
  const J = HORST600_JOINTS;
  const jn = i => `${p}j${i + 1}`;

  const joint = i =>
    `<joint name="${jn(i)}" axis="${J[i].axis}" range="${v(J[i].range)}" ` +
    `damping="${DAMP[i]}" armature="${ARM[i]}" frictionloss="0.05"/>`;

  const body = `
    <body name="${p}horst_basis" pos="${v(pos)}" euler="0 0 ${zrot}" gravcomp="${o.gravcomp ?? 0}">
      <geom type="box" size="0.085 0.085 0.011" pos="0 0 0.011" rgba="${HORST.black}" mass="1.2"/>
      <geom type="cylinder" size="0.075 0.037" pos="0 0 0.056" rgba="${HORST.dark}" mass="4.5"/>
      <body name="${p}link_1" pos="${v(J[0].org)}" gravcomp="${o.gravcomp ?? 0}">
        ${joint(0)}
        <geom type="cylinder" size="0.062 0.075" pos="0 0 0.078" rgba="${HORST.teal}" mass="2.6"/>
        <geom type="cylinder" size="0.052 0.052" pos="0 0.004 0.251" euler="1.5708 0 0" rgba="${HORST.dark}" mass="0.9"/>
        <body name="${p}link_2" pos="${v(J[1].org)}" gravcomp="${o.gravcomp ?? 0}">
          ${joint(1)}
          <geom type="capsule" fromto="0 0.052 0  0 0.052 0.274" size="0.045" rgba="${HORST.teal}" mass="2.2"/>
          <geom type="cylinder" size="0.046 0.03" pos="0 0.02 0.274" euler="1.5708 0 0" rgba="${HORST.dark}" mass="0.4"/>
          <body name="${p}link_3" pos="${v(J[2].org)}" gravcomp="${o.gravcomp ?? 0}">
            ${joint(2)}
            <geom type="box" size="0.048 0.05 0.045" pos="0.012 0 0.012" rgba="${HORST.dark}" mass="1.4"/>
            <geom type="capsule" fromto="0 0 0  0.0855 0 0.0555" size="0.036" rgba="${HORST.black}" mass="0.4"/>
            <body name="${p}link_4" pos="${v(J[3].org)}" gravcomp="${o.gravcomp ?? 0}">
              ${joint(3)}
              <geom type="capsule" fromto="0 0 0  0.1817 0 0" size="0.036" rgba="${HORST.teal}" mass="1.0"/>
              <geom type="cylinder" size="0.037 0.032" pos="0.1817 0 0" euler="1.5708 0 0" rgba="${HORST.dark}" mass="0.35"/>
              <body name="${p}link_5" pos="${v(J[4].org)}" gravcomp="${o.gravcomp ?? 0}">
                ${joint(4)}
                <geom type="box" size="0.03 0.026 0.03" pos="0.018 0 0" rgba="${HORST.light}" mass="0.45"/>
                <body name="${p}link_6" pos="${v(J[5].org)}" gravcomp="${o.gravcomp ?? 0}">
                  ${joint(5)}
                  <geom type="cylinder" size="0.031 0.011" pos="0.011 0 0" euler="0 1.5708 0" rgba="${HORST.dark}" mass="0.18"/>
                  <site name="${p}tcp" pos="0.022 0 0" size="0.008" rgba="0.74 1 0.99 0.9"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>`;

  // steif: höhere Reglerverstärkung für Zellen mit Nutzlast am langen Arm
  const kf = o.steif ?? 1;
  const actuators = J.map((j, i) =>
    `<position name="${p}A${i + 1}" joint="${jn(i)}" kp="${(KP[i] * kf).toFixed(0)}" kv="${(KV[i] * Math.sqrt(kf)).toFixed(0)}" ` +
    `forcerange="-${(FR[i] * kf).toFixed(0)} ${(FR[i] * kf).toFixed(0)}" ctrlrange="${v(j.range)}"/>`).join('\n    ');

  const sensors = `
    <framepos name="${p}tcp_pos" objtype="site" objname="${p}tcp"/>
    <framelinvel name="${p}tcp_vel" objtype="site" objname="${p}tcp"/>
    ${J.map((_, i) => `<jointpos name="${p}q${i + 1}" joint="${jn(i)}"/>`).join('\n    ')}
    ${J.map((_, i) => `<actuatorfrc name="${p}tau${i + 1}" actuator="${p}A${i + 1}"/>`).join('\n    ')}`;

  return { body, actuators, sensors };
}

/** Standard-Header (Optionen + Visual + Boden). */
export function sceneHeader({ timestep = 0.002, integrator = 'implicitfast', gravity = '0 0 -9.81', floor = true, floorSize = 3 } = {}) {
  return {
    open: `<mujoco model="horstSIM">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="${timestep}" integrator="${integrator}" gravity="${gravity}"/>
  <visual>
    <global azimuth="135" elevation="-22" fovy="42"/>
    <map force="0.05" zfar="40"/>
    <scale forcewidth="0.06" contactwidth="0.14" contactheight="0.06" jointwidth="0.05" framelength="0.35" framewidth="0.02"/>
    <rgba haze="0.012 0.118 0.125 1" force="0.35 1 0.98 1" contactpoint="1 0.55 0.2 0.9" contactforce="0.35 1 0.98 0.9"/>
  </visual>
  <worldbody>
    ${floor ? `<geom name="boden" type="plane" size="${floorSize} ${floorSize} 0.1" rgba="0.97 1 1 1" friction="0.9 0.005 0.0001"/>` : ''}`,
    close: (act, sens, key) => `  </worldbody>
  ${act ? `<actuator>\n    ${act}\n  </actuator>` : ''}
  ${sens ? `<sensor>${sens}\n  </sensor>` : ''}
  ${key ?? ''}
</mujoco>`,
  };
}

/** Keyframe-String aus Roboterposen + freien Körpern (Reihenfolge = Deklarationsreihenfolge!). */
export function buildKeyframe(robots, freeBodies) {
  const qpos = [];
  const ctrl = [];
  for (const r of robots) { qpos.push(...(r.home ?? HORST600_HOME)); ctrl.push(...(r.home ?? HORST600_HOME)); }
  for (const f of freeBodies) qpos.push(...f.pos, ...(f.quat ?? [1, 0, 0, 0]));
  return `<keyframe><key name="home" qpos="${qpos.map(x => +(+x).toFixed(5)).join(' ')}" ctrl="${ctrl.join(' ')}"/></keyframe>`;
}

/* ---------------- Fertige Szenen ---------------- */

export function sceneHorstCell() {
  const h = sceneHeader({});
  const robot = horstBody({ pos: [0, 0, 0.37] });
  const objs = [
    { name: 'kiste_a', type: 'box', size: '0.024 0.024 0.024', rgba: '0.92 0.55 0.15 1', pos: [0.36, 0.10, 0.418], mass: 0.12 },
    { name: 'kiste_b', type: 'box', size: '0.024 0.024 0.024', rgba: '0.92 0.55 0.15 1', pos: [0.42, 0.02, 0.418], mass: 0.12 },
    { name: 'zylinder', type: 'cylinder', size: '0.019 0.032', rgba: '0.63 0.63 0.69 1', pos: [0.37, -0.09, 0.426], mass: 0.15 },
    { name: 'kugel', type: 'sphere', size: '0.022', rgba: '0.74 1 0.99 1', pos: [0.44, -0.14, 0.416], mass: 0.1 },
    /* Pick&Place-Teile: Namen tragen die Farbe (_rot/_blau) für die Automatik. */
    { name: 'kiste_rot_1',  type: 'box', size: '0.022 0.022 0.022', rgba: '0.88 0.16 0.14 1', pos: [0.34,  0.16, 0.394], mass: 0.11 },
    { name: 'kiste_rot_2',  type: 'box', size: '0.022 0.022 0.022', rgba: '0.88 0.16 0.14 1', pos: [0.20, -0.20, 0.394], mass: 0.11 },
    { name: 'kiste_blau_1', type: 'box', size: '0.022 0.022 0.022', rgba: '0.15 0.38 0.95 1', pos: [0.45,  0.02, 0.394], mass: 0.11 },
    { name: 'kiste_blau_2', type: 'box', size: '0.022 0.022 0.022', rgba: '0.15 0.38 0.95 1', pos: [0.27,  0.24, 0.394], mass: 0.11 },
    /* Kugel-Traube: kollabiert beim Start und rollt auseinander (Reibung bewusst niedrig). */
    { name: 'rollkugel_1', type: 'sphere', size: '0.022', rgba: '0.74 1 0.99 1',  pos: [0.386, -0.164, 0.392], mass: 0.10, friction: '0.5 0.0015 0.00003' },
    { name: 'rollkugel_2', type: 'sphere', size: '0.022', rgba: '0.35 1 0.98 1',  pos: [0.434, -0.164, 0.392], mass: 0.10, friction: '0.5 0.0015 0.00003' },
    { name: 'rollkugel_3', type: 'sphere', size: '0.022', rgba: '0.92 0.55 0.15 1', pos: [0.386, -0.116, 0.392], mass: 0.10, friction: '0.5 0.0015 0.00003' },
    { name: 'rollkugel_4', type: 'sphere', size: '0.022', rgba: '0.63 0.63 0.69 1', pos: [0.434, -0.116, 0.392], mass: 0.10, friction: '0.5 0.0015 0.00003' },
    { name: 'rollkugel_5', type: 'sphere', size: '0.021', rgba: '0.95 0.42 0.10 1', pos: [0.410, -0.152, 0.428], mass: 0.09, friction: '0.5 0.0015 0.00003' },
    { name: 'rollkugel_6', type: 'sphere', size: '0.024', rgba: '0.85 0.87 0.90 1', pos: [0.416, -0.126, 0.432], mass: 0.12, friction: '0.5 0.0015 0.00003' },
    /* Würfel-Regen aus verschiedenen Höhen; fallwuerfel_4 trifft die Traube. */
    { name: 'fallwuerfel_1', type: 'box', size: '0.026 0.026 0.026', rgba: '0.92 0.55 0.15 1', pos: [0.10, 0.20, 1.30], mass: 0.14, quat: [0.91923, 0.04635, 0.24523, 0.30451] },
    { name: 'fallwuerfel_2', type: 'box', size: '0.030 0.030 0.030', rgba: '0.63 0.63 0.69 1', pos: [0.50, 0.05, 1.50], mass: 0.18, quat: [0.84461, 0.1812, 0.20182, 0.46159] },
    { name: 'fallwuerfel_3', type: 'box', size: '0.024 0.024 0.024', rgba: '0.95 0.42 0.10 1', pos: [0.22, -0.24, 0.95], mass: 0.12, quat: [0.91188, 0.36951, 0.17672, -0.0267] },
    { name: 'fallwuerfel_4', type: 'box', size: '0.028 0.028 0.028', rgba: '0.85 0.87 0.90 1', pos: [0.41, -0.14, 1.05], mass: 0.16, quat: [0.89554, 0.00468, 0.42833, 0.12051] },
  ];
  const objXml = objs.map(o => `
    <body name="${o.name}" pos="${v(o.pos)}"${o.quat ? ` quat="${o.quat.join(' ')}"` : ''}>
      <freejoint/>
      <geom type="${o.type}" size="${o.size}" rgba="${o.rgba}" mass="${o.mass}" friction="${o.friction ?? '0.8 0.004 0.0001'}"/>
    </body>`).join('');
  const world = `
    <body name="tisch" pos="0.18 0 0">
      <geom type="box" size="0.54 0.42 0.018" pos="0 0 0.352" rgba="0.20 0.22 0.23 1"/>
      <!-- umlaufender Rand: 16 mm hoch, hält Kugeln auf dem Tisch -->
      <geom type="box" size="0.54 0.008 0.008"  pos="0 0.412 0.378"  rgba="0.30 0.33 0.35 1"/>
      <geom type="box" size="0.54 0.008 0.008"  pos="0 -0.412 0.378" rgba="0.30 0.33 0.35 1"/>
      <geom type="box" size="0.008 0.404 0.008" pos="0.532 0 0.378"  rgba="0.30 0.33 0.35 1"/>
      <geom type="box" size="0.008 0.404 0.008" pos="-0.532 0 0.378" rgba="0.30 0.33 0.35 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.51 0.39 0.176"   rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.51 -0.39 0.176"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.51 0.39 0.176"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.51 -0.39 0.176" rgba="0.12 0.13 0.14 1"/>
    </body>
    <body name="pad_rest" pos="0.40 -0.28 0.373">
      <geom type="box" size="0.095 0.078 0.003" rgba="0.52 0.52 0.58 0.55"/>
      <geom type="box" size="0.095 0.006 0.016" pos="0 0.084 0.019" rgba="0.52 0.52 0.58 0.55"/>
      <geom type="box" size="0.095 0.006 0.016" pos="0 -0.084 0.019" rgba="0.52 0.52 0.58 0.55"/>
      <geom type="box" size="0.006 0.090 0.016" pos="0.101 0 0.019" rgba="0.52 0.52 0.58 0.55"/>
      <geom type="box" size="0.006 0.090 0.016" pos="-0.101 0 0.019" rgba="0.52 0.52 0.58 0.55"/>
    </body>
    <body name="pad_rot" pos="0.10 0.32 0.373">
      <geom type="box" size="0.095 0.078 0.003" rgba="0.85 0.20 0.18 0.55"/>
      <geom type="box" size="0.095 0.006 0.016" pos="0 0.084 0.019" rgba="0.85 0.20 0.18 0.55"/>
      <geom type="box" size="0.095 0.006 0.016" pos="0 -0.084 0.019" rgba="0.85 0.20 0.18 0.55"/>
      <geom type="box" size="0.006 0.090 0.016" pos="0.101 0 0.019" rgba="0.85 0.20 0.18 0.55"/>
      <geom type="box" size="0.006 0.090 0.016" pos="-0.101 0 0.019" rgba="0.85 0.20 0.18 0.55"/>
    </body>
    <body name="pad_blau" pos="0.10 -0.32 0.373">
      <geom type="box" size="0.095 0.078 0.003" rgba="0.18 0.38 0.9 0.55"/>
      <geom type="box" size="0.095 0.006 0.016" pos="0 0.084 0.019" rgba="0.18 0.38 0.9 0.55"/>
      <geom type="box" size="0.095 0.006 0.016" pos="0 -0.084 0.019" rgba="0.18 0.38 0.9 0.55"/>
      <geom type="box" size="0.006 0.090 0.016" pos="0.101 0 0.019" rgba="0.18 0.38 0.9 0.55"/>
      <geom type="box" size="0.006 0.090 0.016" pos="-0.101 0 0.019" rgba="0.18 0.38 0.9 0.55"/>
    </body>
    <body name="wanne_kugel" pos="0.40 0.28 0.373">
      <geom type="box" size="0.07 0.06 0.003" rgba="0.35 1 0.98 0.35"/>
      <geom type="box" size="0.006 0.06 0.015" pos="0.064 0 0.018"  rgba="0.35 1 0.98 0.35"/>
      <geom type="box" size="0.006 0.06 0.015" pos="-0.064 0 0.018" rgba="0.35 1 0.98 0.35"/>
      <geom type="box" size="0.058 0.006 0.015" pos="0 0.054 0.018"  rgba="0.35 1 0.98 0.35"/>
      <geom type="box" size="0.058 0.006 0.015" pos="0 -0.054 0.018" rgba="0.35 1 0.98 0.35"/>
    </body>
    <body name="wanne_rest" pos="0.40 -0.28 0.373">
      <geom type="box" size="0.07 0.06 0.003" rgba="0.63 0.63 0.69 0.35"/>
      <geom type="box" size="0.006 0.06 0.015" pos="0.064 0 0.018"  rgba="0.63 0.63 0.69 0.35"/>
      <geom type="box" size="0.006 0.06 0.015" pos="-0.064 0 0.018" rgba="0.63 0.63 0.69 0.35"/>
      <geom type="box" size="0.058 0.006 0.015" pos="0 0.054 0.018"  rgba="0.63 0.63 0.69 0.35"/>
      <geom type="box" size="0.058 0.006 0.015" pos="0 -0.054 0.018" rgba="0.63 0.63 0.69 0.35"/>
    </body>
    <body name="ablage" pos="0.30 0.22 0.372">
      <geom type="box" size="0.07 0.07 0.002" pos="0 0 0" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.07 0.004 0.014" pos="0 0.066 0.012" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.07 0.004 0.014" pos="0 -0.066 0.012" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.004 0.07 0.014" pos="0.066 0 0.012" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.004 0.07 0.014" pos="-0.066 0 0.012" rgba="0.05 0.35 0.36 1"/>
    </body>
    ${robot.body}
    ${objXml}`;
  const key = buildKeyframe([{}], objs);
  return h.open + world + '\n' + h.close(robot.actuators, robot.sensors, key);
}

/**
 * Scanmutti: Pakete rutschen über eine Rampe vor den Roboter, werden je nach
 * Etikettenlage gewendet und aufs Förderband gelegt.
 * Rampenneigung 0.48 rad (27.5°); Reibwerte bewusst niedrig, damit die Pakete
 * gleiten (MuJoCo nimmt das Maximum der beiden Kontaktpartner).
 */
/**
 * Kugeln sortieren: rote und blaue Kugeln liegen verstreut auf dem Tisch,
 * zwei offene Kästen nehmen sie sortenrein auf. Die Kästen stehen frei, damit
 * der Roboter senkrecht von oben hineinlegen kann (keine Anfahrt durch die Wand).
 */
export function sceneKugelSort() {
  const h = sceneHeader({ floorSize: 3 });
  const robot = horstBody({ pos: [0, 0, 0.37] });
  const R = 0.024;
  const TISCH = 0.370;                              // Oberkante Tischplatte
  const lagen = [
    [0.30, 0.05], [0.42, 0.14], [0.24, -0.10], [0.46, -0.05], [0.35, 0.20],
    [0.20, 0.14], [0.44, 0.02], [0.31, -0.19], [0.25, 0.02], [0.39, -0.16],
    [0.48, 0.10], [0.22, -0.20],
  ];
  const kugeln = lagen.map(([x, y], i) => {
    const rot = i % 2 === 0;
    return {
      name: `kugel_${rot ? 'rot' : 'blau'}_${Math.floor(i / 2) + 1}`,
      pos: [x, y, TISCH + R + 0.002],
      rgba: rot ? '0.88 0.16 0.14 1' : '0.15 0.38 0.95 1',
    };
  });
  const kugelXml = kugeln.map(k => `
    <body name="${k.name}" pos="${v(k.pos)}">
      <freejoint/>
      <geom type="sphere" size="${R}" rgba="${k.rgba}" mass="0.10" friction="0.65 0.003 0.00006"/>
    </body>`).join('');

  /* Offener Kasten: Boden + vier Wände. Innenmaß 0,18 × 0,15 m, Rand 60 mm hoch. */
  const kasten = (name, x, y, rgba) => `
    <body name="${name}" pos="${x} ${y} ${TISCH}">
      <geom type="box" size="0.095 0.080 0.004" pos="0 0 0.004" rgba="${rgba} 1"/>
      <geom type="box" size="0.095 0.006 0.030" pos="0 0.086 0.038"  rgba="${rgba} 0.75"/>
      <geom type="box" size="0.095 0.006 0.030" pos="0 -0.086 0.038" rgba="${rgba} 0.75"/>
      <geom type="box" size="0.006 0.092 0.030" pos="0.101 0 0.038"  rgba="${rgba} 0.75"/>
      <geom type="box" size="0.006 0.092 0.030" pos="-0.101 0 0.038" rgba="${rgba} 0.75"/>
    </body>`;

  const world = `
    <body name="tisch" pos="0.18 0 0">
      <geom type="box" size="0.56 0.52 0.018" pos="0 0 0.352" rgba="0.20 0.22 0.23 1" friction="0.7 0.004 0.0001"/>
      <geom type="box" size="0.56 0.008 0.010" pos="0 0.512 0.380"  rgba="0.30 0.33 0.35 1"/>
      <geom type="box" size="0.56 0.008 0.010" pos="0 -0.512 0.380" rgba="0.30 0.33 0.35 1"/>
      <geom type="box" size="0.008 0.504 0.010" pos="0.552 0 0.380"  rgba="0.30 0.33 0.35 1"/>
      <geom type="box" size="0.008 0.504 0.010" pos="-0.552 0 0.380" rgba="0.30 0.33 0.35 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.53 0.49 0.176"   rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.53 -0.49 0.176"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.53 0.49 0.176"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.53 -0.49 0.176" rgba="0.12 0.13 0.14 1"/>
    </body>
    ${kasten('kasten_rot', 0.16, 0.36, '0.78 0.18 0.16')}
    ${kasten('kasten_blau', 0.16, -0.36, '0.15 0.36 0.88')}
    ${robot.body}
    ${kugelXml}`;
  const key = buildKeyframe([{}], kugeln);
  return h.open + world + '\n' + h.close(robot.actuators, robot.sensors, key);
}

/**
 * Palettieren: rote und blaue Würfel werden lagenweise im 2×2-Muster auf zwei
 * Paletten gestapelt. Der Stapel entsteht wirklich physikalisch – jede Lage
 * ruht auf der darunterliegenden.
 */
export function scenePalettieren() {
  const h = sceneHeader({ floorSize: 3 });
  const robot = horstBody({ pos: [0, 0, 0.37] });
  const HW = 0.026;                                 // halbe Würfelkante
  const TISCH = 0.370;
  /* 24 Würfel = zwölf je Farbe = drei volle Lagen im 2×2-Muster pro Palette. */
  const lagen = [
    [0.30, 0.02], [0.38, 0.10], [0.25, 0.13], [0.44, 0.03],
    [0.33, 0.19], [0.21, 0.05], [0.47, 0.13], [0.28, -0.06],
    [0.36, -0.13], [0.24, -0.17], [0.45, -0.09], [0.31, -0.21],
    [0.41, -0.20], [0.20, -0.08], [0.49, -0.02], [0.38, -0.03],
    [0.26, 0.23], [0.46, 0.20], [0.34, 0.06], [0.22, -0.02],
    [0.50, 0.06], [0.29, 0.09], [0.43, -0.16], [0.19, 0.18],
  ];
  const wuerfel = lagen.map(([x, y], i) => {
    const rot = i % 2 === 0;
    return {
      name: `wuerfel_${rot ? 'rot' : 'blau'}_${Math.floor(i / 2) + 1}`,
      pos: [x, y, TISCH + HW + 0.002],
      rgba: rot ? '0.88 0.16 0.14 1' : '0.15 0.38 0.95 1',
    };
  });
  const wXml = wuerfel.map(w => `
    <body name="${w.name}" pos="${v(w.pos)}">
      <freejoint/>
      <geom type="box" size="${HW} ${HW} ${HW}" rgba="${w.rgba}" mass="0.13" friction="0.9 0.005 0.0002"/>
    </body>`).join('');

  const palette = (name, x, y, rgba) => `
    <body name="${name}" pos="${x} ${y} ${TISCH}">
      <geom type="box" size="0.085 0.075 0.006" pos="0 0 0.006" rgba="${rgba} 1" friction="0.9 0.005 0.0002"/>
      <geom type="box" size="0.012 0.075 0.008" pos="0.070 0 -0.006" rgba="0.42 0.32 0.20 1"/>
      <geom type="box" size="0.012 0.075 0.008" pos="-0.070 0 -0.006" rgba="0.42 0.32 0.20 1"/>
      <geom type="box" size="0.012 0.075 0.008" pos="0 0 -0.006" rgba="0.42 0.32 0.20 1"/>
    </body>`;

  const world = `
    <body name="tisch" pos="0.18 0 0">
      <geom type="box" size="0.56 0.52 0.018" pos="0 0 0.352" rgba="0.20 0.22 0.23 1" friction="0.9 0.005 0.0002"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.53 0.49 0.176"   rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.53 -0.49 0.176"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.53 0.49 0.176"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.53 -0.49 0.176" rgba="0.12 0.13 0.14 1"/>
    </body>
    ${palette('palette_rot', 0.16, 0.36, '0.55 0.40 0.26')}
    ${palette('palette_blau', 0.16, -0.36, '0.55 0.40 0.26')}
    ${robot.body}
    ${wXml}`;
  const key = buildKeyframe([{}], wuerfel);
  return h.open + world + '\n' + h.close(robot.actuators, robot.sensors, key);
}

/** Einstellbare Paketmischung für Scanmutti (wird von der Oberfläche gesetzt). */
export const SCAN_CONFIG = {
  klassen: { XS: true, S: true, M: true, L: true },
  anzahl: 10,
  rundlauf: true,                                   // linke Bahn führt zur Rampe zurück
};

export function sceneScanmutti() {
  const h = sceneHeader({ floorSize: 4 });
  const robot = horstBody({ pos: [0, 0, 0.37] });
  const T = 0.50;                                   // Rampenneigung [rad] ≈ 29°
  const RL = 0.52;                                  // halbe Rampenlänge (größer: mehr Pakete)
  const RY = 0.70, RZ = 0.60;                       // Rampenmitte
  const cs = Math.cos(T), sn = Math.sin(T);
  /* Höhe der Rampenoberfläche an einer Weltkoordinate y (plus halbe Pakethöhe). */
  const zOnRamp = (y, halfH) => RZ + (y - RY) * (sn / cs) + (0.008 + halfH) / cs;
  const qT = (extra = 0) => {
    const a = T + extra;
    return [+Math.cos(a / 2).toFixed(5), +Math.sin(a / 2).toFixed(5), 0, 0];
  };

  /* Vier Größenklassen nach gängiger Paketstatistik, im Zellenmaßstab 1:3
     (die Originalmaße 20×15×8 bis 50×40×30 cm sprengen eine 1,2-m-Zelle).
     s = halbe Kantenmaße, et = Etikett, anteil = typische Häufigkeit. */
  const GROESSE = {
    XS: { lbh: '20×15×8',  s: [0.033, 0.025, 0.013], m: 0.14, et: [0.015, 0.010, 0.0020], anteil: 12 },
    S:  { lbh: '30×20×12', s: [0.050, 0.033, 0.020], m: 0.24, et: [0.022, 0.014, 0.0022], anteil: 23 },
    M:  { lbh: '40×30×20', s: [0.067, 0.050, 0.033], m: 0.42, et: [0.030, 0.020, 0.0024], anteil: 30 },
    L:  { lbh: '50×40×30', s: [0.083, 0.067, 0.050], m: 0.66, et: [0.038, 0.026, 0.0026], anteil: 35 },
  };
  const cfg = SCAN_CONFIG;
  const aktiv = Object.keys(GROESSE).filter(k => cfg.klassen[k]);
  const klassen = aktiv.length ? aktiv : ['S'];
  /* Reihenfolge nach Anteilen: je Klasse so viele Plätze, wie ihr Anteil hergibt. */
  const summe = klassen.reduce((a, k) => a + GROESSE[k].anteil, 0);
  const topf = [];
  klassen.forEach(k => { const n = Math.max(1, Math.round(cfg.anzahl * GROESSE[k].anteil / summe));
                         for (let i = 0; i < n; i++) topf.push(k); });
  const reihe = Array.from({ length: cfg.anzahl }, (_, i) => topf[(i * 3 + 1) % topf.length]);
  const pakete = reihe.map((g, i) => {
    const G = GROESSE[g];
    const y = 0.245 + i * 0.135;
    const kopfueber = i % 3 !== 0;                   // Etikett zufällig unten
    return {
      name: `paket_${g}_${i + 1}`, groesse: g, G,
      pos: [0.30 + (i % 2 ? 0.03 : -0.03), y, i === 0 ? 0.370 + G.s[2] + 0.002 : zOnRamp(y, G.s[2])],
      quat: i === 0 ? (kopfueber ? [0, 1, 0, 0] : [1, 0, 0, 0]) : qT(kopfueber ? Math.PI : 0),
    };
  });
  const paketXml = pakete.map(p => `
    <body name="${p.name}" pos="${v(p.pos)}" quat="${p.quat.join(' ')}">
      <freejoint/>
      <geom type="box" size="${p.G.s.join(' ')}" rgba="0.72 0.55 0.34 1" mass="${p.G.m}" friction="0.20 0.002 0.0001"/>
      <geom name="${p.name}_etikett" type="box" size="${p.G.et.join(' ')}" pos="0 0 ${(p.G.s[2] + 0.0018).toFixed(4)}"
            rgba="0.97 0.96 0.90 1" mass="0.004" friction="0.28 0.002 0.0001"/>
      <geom type="box" size="${(p.G.et[0] * 0.62).toFixed(4)} 0.0032 0.0006" pos="0 0.005 ${(p.G.s[2] + 0.0040).toFixed(4)}" rgba="0.10 0.12 0.13 1" mass="0.0005"/>
      <geom type="box" size="${(p.G.et[0] * 0.62).toFixed(4)} 0.0032 0.0006" pos="0 -0.004 ${(p.G.s[2] + 0.0040).toFixed(4)}" rgba="0.10 0.12 0.13 1" mass="0.0005"/>
    </body>`).join('');

  const world = `
    <body name="tisch" pos="0.18 0 0">
      <geom type="box" size="0.60 0.50 0.018" pos="0 0 0.352" rgba="0.20 0.22 0.23 1" friction="0.30 0.003 0.0001"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.57 0.47 0.176"   rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.57 -0.47 0.176"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.57 0.47 0.176"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.57 -0.47 0.176" rgba="0.12 0.13 0.14 1"/>
    </body>

    <!-- Zuführrampe (+Y), 0,84 m lang, mit Seitenführungen -->
    <body name="rampe" pos="0.30 ${RY} ${RZ}" euler="${T} 0 0">
      <geom type="box" size="0.26 ${RL} 0.008" rgba="0.26 0.29 0.31 1" friction="0.16 0.001 0.00005"/>
      <geom type="box" size="0.010 ${RL} 0.055" pos="0.268 0 0.055"  rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.010 ${RL} 0.055" pos="-0.268 0 0.055" rgba="0.05 0.35 0.36 1"/>
    </body>
    <body name="rampengestell" pos="0.30 0 0">
      <geom type="box" size="0.015 0.015 0.20" pos="0.17 0.30 0.20"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.015 0.015 0.20" pos="-0.17 0.30 0.20" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.015 0.015 0.36" pos="0.17 0.96 0.36"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.015 0.015 0.36" pos="-0.17 0.96 0.36" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.19 0.015 0.012" pos="0 0.96 0.73"     rgba="0.12 0.13 0.14 1"/>
    </body>
    <!-- Anschlag: hält die Pakete in der Greifzone vor dem Roboter -->
    <body name="anschlag" pos="0.30 0.176 0.398">
      <geom type="box" size="0.18 0.006 0.026" rgba="0.05 0.35 0.36 1"/>
    </body>

    <!-- Wendestation: flacher Absetztisch. Der Roboter stellt das Handgelenk
         auf 90° an und setzt das Paket damit gezielt auf eine Seitenfläche. -->

    <!-- Hauptband (+X) mit Weiche bei x = 0,70. Am Weichenfeld fehlen die
         Seitenwände, damit Pakete nach links und rechts ausgeschleust werden. -->
    <body name="band" pos="0 -0.34 0">
      <geom type="box" size="0.31 0.10 0.012" pos="0.31 0 0.402" rgba="0.13 0.15 0.16 1" friction="0.5 0.004 0.0001"/>
      <geom type="box" size="0.31 0.006 0.022" pos="0.31 0.106 0.436"  rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.31 0.006 0.022" pos="0.31 -0.106 0.436" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.26 0.10 0.012" pos="0.96 0 0.402" rgba="0.13 0.15 0.16 1" friction="0.5 0.004 0.0001"/>
      <geom type="box" size="0.26 0.006 0.022" pos="0.96 0.106 0.436"  rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.26 0.006 0.022" pos="0.96 -0.106 0.436" rgba="0.05 0.35 0.36 1"/>
      <geom type="cylinder" size="0.026 0.10" pos="1.22 0 0.402" euler="1.5708 0 0" rgba="0.35 1 0.98 0.5"/>
      <geom type="cylinder" size="0.026 0.10" pos="0.00 0 0.402" euler="1.5708 0 0" rgba="0.35 1 0.98 0.5"/>
      <geom type="box" size="0.02 0.02 0.195" pos="0.06 0.09 0.195"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.195" pos="0.06 -0.09 0.195" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.195" pos="1.16 0.09 0.195"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.195" pos="1.16 -0.09 0.195" rgba="0.12 0.13 0.14 1"/>
    </body>

    <!-- Weichenfeld: Drehteller-Optik, hier entscheidet sich die Bahn -->
    <body name="weiche" pos="0.70 -0.34 0">
      <geom type="box" size="0.09 0.10 0.012" pos="0 0 0.402" rgba="0.05 0.35 0.36 1" friction="0.45 0.004 0.0001"/>
      <geom type="cylinder" size="0.062 0.0035" pos="0 0 0.4155" rgba="0.35 1 0.98 0.30"/>
    </body>

    <!-- Abzweig RECHTS (−Y): endet in der Sammelbox -->
    <body name="bahn_rechts" pos="0.70 -0.68 0">
      <geom type="box" size="0.09 0.24 0.012" pos="0 0 0.402" rgba="0.13 0.15 0.16 1" friction="0.5 0.004 0.0001"/>
      <geom type="box" size="0.006 0.24 0.022" pos="0.096 0 0.436"  rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.006 0.24 0.022" pos="-0.096 0 0.436" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.02 0.02 0.195" pos="0.07 -0.21 0.195" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.195" pos="-0.07 -0.21 0.195" rgba="0.12 0.13 0.14 1"/>
    </body>

    <!-- Abzweig LINKS (+Y): Rundlauf, führt hinter dem Roboter zur Rampe zurück -->
    <body name="bahn_links" pos="0.70 0.12 0">
      <geom type="box" size="0.09 0.34 0.012" pos="0 0 0.402" rgba="0.13 0.15 0.16 1" friction="0.5 0.004 0.0001"/>
      <geom type="box" size="0.006 0.34 0.022" pos="0.096 0 0.436"  rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.006 0.34 0.022" pos="-0.096 0 0.436" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.02 0.02 0.195" pos="0.07 0.31 0.195" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.195" pos="-0.07 0.31 0.195" rgba="0.12 0.13 0.14 1"/>
    </body>

    <!-- Zielbox geradeaus: ist sie voll, endet das Programm -->
    <body name="zielbox" pos="1.42 -0.34 0">
      <geom type="box" size="0.17 0.15 0.008" pos="0 0 0.150" rgba="0.42 0.32 0.20 1" friction="0.8 0.004 0.0001"/>
      <geom type="box" size="0.17 0.008 0.085" pos="0 0.158 0.243"  rgba="0.55 0.42 0.26 0.85"/>
      <geom type="box" size="0.17 0.008 0.085" pos="0 -0.158 0.243" rgba="0.55 0.42 0.26 0.85"/>
      <geom type="box" size="0.008 0.166 0.085" pos="0.178 0 0.243"  rgba="0.55 0.42 0.26 0.85"/>
      <geom type="box" size="0.008 0.166 0.085" pos="-0.178 0 0.243" rgba="0.55 0.42 0.26 0.85"/>
      <geom type="box" size="0.02 0.02 0.071" pos="0.15 0.13 0.071"   rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.071" pos="0.15 -0.13 0.071"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.071" pos="-0.15 0.13 0.071"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.071" pos="-0.15 -0.13 0.071" rgba="0.12 0.13 0.14 1"/>
    </body>

    <!-- Sammelbox der rechten Bahn -->
    <body name="boxrechts" pos="0.70 -1.06 0">
      <geom type="box" size="0.15 0.14 0.008" pos="0 0 0.150" rgba="0.30 0.30 0.34 1" friction="0.8 0.004 0.0001"/>
      <geom type="box" size="0.15 0.008 0.085" pos="0 0.148 0.243"  rgba="0.42 0.42 0.48 0.85"/>
      <geom type="box" size="0.15 0.008 0.085" pos="0 -0.148 0.243" rgba="0.42 0.42 0.48 0.85"/>
      <geom type="box" size="0.008 0.156 0.085" pos="0.158 0 0.243"  rgba="0.42 0.42 0.48 0.85"/>
      <geom type="box" size="0.008 0.156 0.085" pos="-0.158 0 0.243" rgba="0.42 0.42 0.48 0.85"/>
      <geom type="box" size="0.02 0.02 0.071" pos="0.13 0.12 0.071"   rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.071" pos="0.13 -0.12 0.071"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.071" pos="-0.13 0.12 0.071"  rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.071" pos="-0.13 -0.12 0.071" rgba="0.12 0.13 0.14 1"/>
    </body>
    ${robot.body}
    ${paketXml}`;
  const key = buildKeyframe([{}], pakete);
  return h.open + world + '\n' + h.close(robot.actuators, robot.sensors, key);
}

export function sceneHorstSolo() {
  const h = sceneHeader({ floorSize: 2 });
  const robot = horstBody({ pos: [0, 0, 0] });
  const key = buildKeyframe([{}], []);
  return h.open + robot.body + '\n' + h.close(robot.actuators, robot.sensors, key);
}

export function scenePendulum() {
  const h = sceneHeader({ timestep: 0.001, integrator: 'RK4', floorSize: 2 });
  let chain = '<site name="anker" pos="0 0 1.6" size="0.02" rgba="0.35 1 0.98 1"/>';
  let inner = '';
  const n = 5;
  for (let i = 0; i < n; i++) {
    inner += `<body name="glied_${i + 1}" pos="0 0 ${i === 0 ? 1.6 : -0.24}">
      <joint type="hinge" axis="0 1 0" damping="0.002"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.24" size="0.018" rgba="${i % 2 ? HORST.teal : HORST.light}" mass="0.3"/>`;
  }
  inner += '</body>'.repeat(n);
  const key = `<keyframe><key name="ausgelenkt" qpos="1.9 0 0 0 0"/></keyframe>`;
  return h.open + chain + inner + '\n' + h.close('', '', key);
}

export function sceneStack() {
  const h = sceneHeader({ floorSize: 2.5 });
  let boxes = '';
  const free = [];
  for (let i = 0; i < 6; i++) {
    const z = 0.05 + i * 0.101;
    boxes += `
    <body name="stein_${i + 1}" pos="0 0 ${z}">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" rgba="${i % 2 ? '0.92 0.55 0.15 1' : HORST.teal}" mass="0.4" friction="0.85 0.004 0.0001"/>
    </body>`;
    free.push({ pos: [0, 0, z] });
  }
  boxes += `
    <body name="abrisskugel" pos="-1.4 0 0.35">
      <freejoint/>
      <geom type="sphere" size="0.11" rgba="0.63 0.63 0.69 1" mass="4"/>
    </body>`;
  const qpos = free.flatMap(f => [...f.pos, 1, 0, 0, 0]).concat([-1.4, 0, 0.35, 1, 0, 0, 0]);
  const qvel = new Array(free.length * 6).fill(0).concat([5.5, 0, 2.2, 0, 0, 0]);
  const key = `<keyframe><key name="wurf" qpos="${qpos.join(' ')}" qvel="${qvel.join(' ')}"/></keyframe>`;
  return h.open + boxes + '\n' + h.close('', '', key);
}

/* Humanoid (DeepMind-Beispiel) wird zur Laufzeit aus models/humanoid.xml geladen. */

/**
 * Hallenlogistik: Auf einem Rundlauf-Förderband kreisen Pakete. Eine fahrbare
 * Hubplattform mit zwei HORST-Armen fährt an das Band, greift die Pakete
 * beidseitig wie ein Mensch mit zwei Armen und stellt sie in die Regale.
 *
 * Aufbau der Plattform: Schlitten in X und Y, darauf ein Hubschlitten in Z,
 * darauf beide Roboterbasen (Präfixe "A_" und "B_").
 */
export function sceneHalle() {
  const h = sceneHeader({ floorSize: 7 });
  const BAND_Z = 0.62;                              // Oberkante Bandumlauf
  const X0 = 1.15, X1 = 3.15, Y0 = -0.95, Y1 = 0.95; // Mittellinie des Rundlaufs
  const BW = 0.24;                                  // halbe Bandbreite

  /* Vier gerade Bandabschnitte bilden den Umlauf. */
  const seg = (name, x, y, sx, sy) => `
    <body name="${name}" pos="${x} ${y} ${BAND_Z - 0.03}">
      <geom type="box" size="${sx} ${sy} 0.03" rgba="0.16 0.19 0.21 1" friction="0.25 0.002 0.0001"/>
      <geom type="box" size="${sx} 0.012 0.035" pos="0 ${sy - 0.012} 0.032" rgba="0.35 0.62 0.63 0.55"/>
      <geom type="box" size="${sx} 0.012 0.035" pos="0 ${-(sy - 0.012)} 0.032" rgba="0.35 0.62 0.63 0.55"/>
    </body>`;
  const segQ = (name, x, y, sx, sy) => `
    <body name="${name}" pos="${x} ${y} ${BAND_Z - 0.03}">
      <geom type="box" size="${sx} ${sy} 0.03" rgba="0.16 0.19 0.21 1" friction="0.25 0.002 0.0001"/>
      <geom type="box" size="0.012 ${sy} 0.035" pos="${sx - 0.012} 0 0.032" rgba="0.35 0.62 0.63 0.55"/>
      <geom type="box" size="0.012 ${sy} 0.035" pos="${-(sx - 0.012)} 0 0.032" rgba="0.35 0.62 0.63 0.55"/>
    </body>`;
  const mx = (X0 + X1) / 2, my = (Y0 + Y1) / 2, hx = (X1 - X0) / 2, hy = (Y1 - Y0) / 2;
  const band = `
    ${seg('band_vorn', mx, Y0, hx + BW, BW)}
    ${seg('band_hinten', mx, Y1, hx + BW, BW)}
    ${segQ('band_links', X0, my, BW, hy - BW)}
    ${segQ('band_rechts', X1, my, BW, hy - BW)}
    <body name="bandgestell" pos="0 0 0">
      ${[[X0, Y0], [X1, Y0], [X0, Y1], [X1, Y1]].map(([x, y]) => `
      <geom type="cylinder" size="0.035 ${(BAND_Z - 0.06) / 2}" pos="${x} ${y} ${(BAND_Z - 0.06) / 2}" rgba="0.12 0.14 0.15 1"/>`).join('')}
    </body>`;

  /* Pakete auf dem Umlauf, gleichmäßig verteilt. */
  const G = { klein: [0.055, 0.042, 0.030], gross: [0.085, 0.062, 0.045] };
  const N = 12;
  const umfang = 2 * (hx * 2 + hy * 2);
  const punkt = (t) => {                            // t ∈ [0,1) entlang des Umlaufs
    const u = t * umfang, a = hx * 2, b = a + hy * 2, c = b + hx * 2;
    if (u < a) return [X0 + u, Y0];
    if (u < b) return [X1, Y0 + (u - a)];
    if (u < c) return [X1 - (u - b), Y1];
    return [X0, Y1 - (u - c)];
  };
  const pakete = Array.from({ length: N }, (_, i) => {
    const gross = i % 3 === 2;
    const g = gross ? G.gross : G.klein;
    const [x, y] = punkt((i + 0.5) / N);
    return { name: `hpaket_${gross ? 'gross' : 'klein'}_${i + 1}`, s: g, m: gross ? 0.5 : 0.22,
             pos: [x, y, BAND_Z + g[2] + 0.004] };
  });
  const paketXml = pakete.map(p => `
    <body name="${p.name}" pos="${v(p.pos)}">
      <freejoint/>
      <geom type="box" size="${p.s.join(' ')}" rgba="0.72 0.55 0.34 1" mass="${p.m}" friction="0.35 0.003 0.0001"/>
      <geom type="box" size="${(p.s[0] * 0.5).toFixed(4)} ${(p.s[1] * 0.5).toFixed(4)} 0.0022"
            pos="0 0 ${(p.s[2] + 0.0016).toFixed(4)}" rgba="0.97 0.97 0.94 1" mass="0.001"/>
    </body>`).join('');

  /* Regale: drei Fächerebenen, links und rechts der Fahrbahn. */
  const regal = (name, x, y, zrot) => `
    <body name="${name}" pos="${x} ${y} 0" euler="0 0 ${zrot}">
      ${[0.34, 0.72, 1.10].map((z, i) => `
      <geom type="box" size="0.34 0.20 0.012" pos="0 0 ${z}" rgba="0.62 0.47 0.28 1" friction="0.8 0.004 0.0002"/>
      <geom type="box" size="0.34 0.012 0.030" pos="0 0.19 ${z + 0.04}" rgba="0.52 0.39 0.23 1"/>`).join('')}
      ${[[-0.32, -0.18], [0.32, -0.18], [-0.32, 0.18], [0.32, 0.18]].map(([dx, dy]) => `
      <geom type="box" size="0.022 0.022 0.62" pos="${dx} ${dy} 0.62" rgba="0.34 0.26 0.16 1"/>`).join('')}
    </body>`;

  /* Fahrbare Hubplattform mit zwei Armen. */
  /* Beide Arme nebeneinander in Fahrtrichtung montiert und zum Band gedreht:
     so fassen sie einen Karton links und rechts an wie ein Mensch. */
  const armA = horstBody({ prefix: 'A_', pos: [0.26, 0, 0.10], zrot: 1.5708, steif: 6, gravcomp: 1 });
  const armB = horstBody({ prefix: 'B_', pos: [-0.26, 0, 0.10], zrot: 1.5708, steif: 6, gravcomp: 1 });
  const plattform = `
    <body name="aggregat" pos="0.30 0 0">
      <joint name="Px" type="slide" axis="1 0 0" range="-0.35 2.6" damping="900" armature="6"/>
      <joint name="Py" type="slide" axis="0 1 0" range="-1.62 -0.9" damping="900" armature="6"/>
      <geom type="box" size="0.42 0.20 0.055" pos="0 0 0.10" rgba="0.13 0.15 0.16 1" mass="42"
            contype="0" conaffinity="0"/>
      ${[[-0.34, -0.15], [0.34, -0.15], [-0.34, 0.15], [0.34, 0.15]].map(([x, y]) => `
      <geom type="cylinder" size="0.045 0.028" pos="${x} ${y} 0.045" euler="1.5708 0 0" rgba="0.08 0.09 0.10 1" mass="1.5"
            contype="0" conaffinity="0"/>`).join('')}
      <body name="hubschlitten" pos="0 0 0.155">
        <joint name="Pz" type="slide" axis="0 0 1" range="0 0.75" damping="1200" armature="8"/>
        <geom type="box" size="0.38 0.17 0.028" pos="0 0 0.028" rgba="0.20 0.55 0.57 1" mass="14"
              contype="0" conaffinity="0"/>
        <geom type="box" size="0.05 0.05 0.10" pos="0 0 -0.06" rgba="0.10 0.12 0.13 1" mass="2"
              contype="0" conaffinity="0"/>
        ${armA.body}
        ${armB.body}
      </body>
    </body>`;

  /* Reihenfolge ist bindend: Der Keyframe listet erst die Gelenke der Maschinen,
     dann die freien Körper – also muss die Plattform VOR den Paketen stehen. */
  const world = `
    ${band}
    ${regal('regal_1', 0.35, -2.00, 0)}
    ${regal('regal_2', 1.55, -2.00, 0)}
    ${regal('regal_3', 2.75, -2.00, 0)}
    ${plattform}
    ${paketXml}`;

  const akt = `
    <position name="APx" joint="Px" kp="26000" kv="3200" ctrlrange="-0.35 2.6" forcerange="-9000 9000"/>
    <position name="APy" joint="Py" kp="26000" kv="3200" ctrlrange="-1.62 -0.9" forcerange="-9000 9000"/>
    <position name="APz" joint="Pz" kp="30000" kv="3600" ctrlrange="0 0.75" forcerange="-12000 12000"/>
    ${armA.actuators}
    ${armB.actuators}`;
  /* Ruhelage: Plattform steht VOR der vorderen Bandkante. Stand sie in der
     Mitte, steckte die Basis von Arm A im linken Bandrahmen (55 mm tief) –
     die Zwangskraft hielt dann das gesamte Fahrwerk fest. */
  const key = buildKeyframe(
    [{ home: [1.25, -1.30] }, { home: [0.30] }, { home: HORST600_HOME }, { home: HORST600_HOME }],
    pakete);
  return h.open + world + '\n' + h.close(akt, armA.sensors + armB.sensors, key);
}

/* ---------------- Großer Rundlauf (Scanmutti XL) ----------------
 * Hauptstrecke vom Roboter nach Osten, dort ein Dreiwege-Modul:
 * zwei Abzweige kippen in je eine Kiste, die Geradeausstrecke läuft
 * im Kreis und steigt am Ende bergauf zurück auf die Rampe.
 * Der Pfad ist zugleich Geometrie (Szene) und Fahrbahn (Antrieb).
 */
export const RUNDLAUF = {
  z: 0.42,                                  // Bandhöhe der Ebene
  bw: 0.15,                                 // halbe Bandbreite (L-Paket ist 0,083 halbbreit)
  v: 0.32,                                  // Bandgeschwindigkeit [m/s]
  weiche: [1.02, -0.42],                    // Mitte des Dreiwege-Moduls
  /* Der Kreis endet vor dem Roboter: Dort fallen die Pakete auf den Tisch und
     werden neu aufgenommen. Eine Rampe braucht es dafür nicht mehr. */
  haupt: [
    [0.02, -0.42, 0.42],
    [1.52, -0.42, 0.42],
    [1.52, 1.06, 0.42],
    [0.30, 1.06, 0.42],
    [0.30, 0.44, 0.42],                     // Abwurf vor dem Roboter
  ],
  zweigA: [[1.02, -0.42, 0.42], [1.02, -0.96, 0.40]],
  zweigB: [[1.02, -0.42, 0.42], [1.02, 0.18, 0.40]],
  kisteA: [1.02, -1.20, 0.0],
  kisteB: [1.02, 0.44, 0.0],
};

export function sceneRundlauf() {
  const h = sceneHeader({ floorSize: 4 });
  const robot = horstBody({ pos: [0, 0, 0.37] });
  const R = RUNDLAUF;
  const T = 0.50, RL = 0.42, RY = 0.62, RZ = 0.56;
  const cs = Math.cos(T), sn = Math.sin(T);
  const zOnRamp = (y, halfH) => RZ + (y - RY) * (sn / cs) + (0.008 + halfH) / cs;
  const qT = (extra = 0) => {
    const a = T + extra;
    return [+Math.cos(a / 2).toFixed(5), +Math.sin(a / 2).toFixed(5), 0, 0];
  };

  const GROESSE = {
    XS: { s: [0.033, 0.025, 0.013], m: 0.14, et: [0.015, 0.010, 0.0020], anteil: 12 },
    S:  { s: [0.050, 0.033, 0.020], m: 0.24, et: [0.022, 0.014, 0.0022], anteil: 23 },
    M:  { s: [0.067, 0.050, 0.033], m: 0.42, et: [0.030, 0.020, 0.0024], anteil: 30 },
    L:  { s: [0.083, 0.067, 0.050], m: 0.66, et: [0.038, 0.026, 0.0026], anteil: 35 },
  };
  const cfg = SCAN_CONFIG;
  const aktiv = Object.keys(GROESSE).filter(k => cfg.klassen[k]);
  const klassen = aktiv.length ? aktiv : ['S'];
  const summe = klassen.reduce((a, k) => a + GROESSE[k].anteil, 0);
  const topf = [];
  klassen.forEach(k => { const n = Math.max(1, Math.round(cfg.anzahl * GROESSE[k].anteil / summe));
                         for (let i = 0; i < n; i++) topf.push(k); });
  const reihe = Array.from({ length: cfg.anzahl }, (_, i) => topf[(i * 3 + 1) % topf.length]);

  /* --- Bandstücke aus dem Pfad erzeugen --- */
  const strecke = (a, b, name) => {
    const dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
    const len = Math.hypot(dx, dy, dz);
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
    const gier = Math.atan2(dy, dx);
    const nick = -Math.asin(dz / (len || 1));
    return `
    <body name="${name}" pos="${v(mid)}" euler="0 ${nick.toFixed(4)} ${gier.toFixed(4)}">
      <geom type="box" size="${(len / 2 + R.bw).toFixed(3)} ${R.bw} 0.018" pos="0 0 -0.018"
            rgba="0.16 0.19 0.21 1" friction="0.22 0.002 0.0001"/>
      <geom type="box" size="${(len / 2 + R.bw).toFixed(3)} 0.010 0.030" pos="0 ${(R.bw - 0.01).toFixed(3)} 0.012"
            rgba="0.35 0.62 0.63 0.5"/>
      <geom type="box" size="${(len / 2 + R.bw).toFixed(3)} 0.010 0.030" pos="0 ${(-(R.bw - 0.01)).toFixed(3)} 0.012"
            rgba="0.35 0.62 0.63 0.5"/>
    </body>`;
  };
  const stuetze = (p, name) => `
    <body name="${name}" pos="${p[0]} ${p[1]} 0">
      <geom type="cylinder" size="0.022 ${((p[2] - 0.02) / 2).toFixed(3)}" pos="0 0 ${((p[2] - 0.02) / 2).toFixed(3)}"
            rgba="0.12 0.14 0.15 1"/>
    </body>`;

  const bahnen = R.haupt.slice(0, -1).map((p, i) => strecke(p, R.haupt[i + 1], `bahn_${i + 1}`)).join('')
    + strecke(R.zweigA[0], R.zweigA[1], 'zweig_a')
    + strecke(R.zweigB[0], R.zweigB[1], 'zweig_b')
    + R.haupt.map((p, i) => stuetze(p, `stuetze_${i + 1}`)).join('')
    + stuetze(R.zweigA[1], 'stuetze_a') + stuetze(R.zweigB[1], 'stuetze_b');

  /* Dreiwege-Modul als sichtbarer Block */
  const weiche = `
    <body name="weiche" pos="${R.weiche[0]} ${R.weiche[1]} ${R.z - 0.019}">
      <geom type="box" size="${R.bw} ${R.bw} 0.020" rgba="0.85 0.55 0.16 0.9"/>
      <geom type="cylinder" size="0.028 0.026" pos="0 0 0.040" rgba="0.20 0.55 0.57 1"/>
    </body>`;

  const kiste = (name, p) => `
    <body name="${name}" pos="${p[0]} ${p[1]} 0.0">
      <geom type="box" size="0.19 0.15 0.008" pos="0 0 0.008" rgba="0.30 0.36 0.38 1"/>
      <geom type="box" size="0.19 0.010 0.16" pos="0 0.145 0.16" rgba="0.42 0.70 0.70 0.35"/>
      <geom type="box" size="0.19 0.010 0.16" pos="0 -0.145 0.16" rgba="0.42 0.70 0.70 0.35"/>
      <geom type="box" size="0.010 0.15 0.16" pos="0.18 0 0.16" rgba="0.42 0.70 0.70 0.35"/>
      <geom type="box" size="0.010 0.15 0.16" pos="-0.18 0 0.16" rgba="0.42 0.70 0.70 0.35"/>
    </body>`;

  const tisch = `
    <body name="tisch" pos="0.16 0.02 0.0">
      <geom type="box" size="0.52 0.46 0.012" pos="0 0 0.358" rgba="0.62 0.70 0.72 1" friction="0.7 0.004 0.0002"/>
      ${[[-0.48, -0.42], [0.48, -0.42], [-0.48, 0.42], [0.48, 0.42]].map(([x, y]) => `
      <geom type="cylinder" size="0.020 0.179" pos="${x} ${y} 0.179" rgba="0.20 0.24 0.26 1"/>`).join('')}
    </body>`;

  /* Alle Pakete gleichmäßig über den Umlauf verteilen – der Kreis selbst
     führt sie vor den Roboter zurück, eine Zuführrampe entfällt. */
  const bogen = R.haupt.slice(0, -1).map((p, i) => Math.hypot(
    R.haupt[i + 1][0] - p[0], R.haupt[i + 1][1] - p[1], R.haupt[i + 1][2] - p[2]));
  const gesamt = bogen.reduce((a, b) => a + b, 0);
  const punktBei = (u) => {
    let rest = u % gesamt;
    for (let i = 0; i < bogen.length; i++) {
      if (rest <= bogen[i]) {
        const a = R.haupt[i], b = R.haupt[i + 1], t = bogen[i] ? rest / bogen[i] : 0;
        return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
      }
      rest -= bogen[i];
    }
    return R.haupt[0];
  };
  const pakete = reihe.map((g, i) => {
    const G = GROESSE[g];
    const kopfueber = i % 3 !== 0;
    const q = punktBei(0.35 + i * (gesamt * 0.88) / reihe.length);
    return { name: `paket_${g}_${i + 1}`, G,
             pos: [q[0], q[1], q[2] + G.s[2] + 0.010],
             quat: kopfueber ? [0, 1, 0, 0] : [1, 0, 0, 0] };
  });
  const paketXml = pakete.map(p => `
    <body name="${p.name}" pos="${v(p.pos)}" quat="${p.quat.join(' ')}">
      <freejoint/>
      <geom type="box" size="${p.G.s.join(' ')}" rgba="0.72 0.55 0.34 1" mass="${p.G.m}" friction="0.22 0.002 0.0001"/>
      <geom type="box" size="${p.G.et[0]} ${p.G.et[1]} ${p.G.et[2]}" pos="0 0 ${(p.G.s[2] + 0.0018).toFixed(4)}"
            rgba="0.97 0.97 0.94 1" mass="0.001"/>
    </body>`).join('');

  const world = tisch + bahnen + weiche
    + kiste('kiste_a', R.kisteA) + kiste('kiste_b', R.kisteB) + robot.body + paketXml;
  const key = buildKeyframe([{}], pakete);
  return h.open + world + '\n' + h.close(robot.actuators, robot.sensors, key);
}

export const SCENES = [
  { id: 'horst_cell', name: 'HORST600 – Arbeitszelle', make: sceneHorstCell },
  { id: 'kugelsort', name: 'Part Separation – Kugeln sortieren', make: sceneKugelSort },
  { id: 'palettieren', name: 'Pick & Place – Palettieren', make: scenePalettieren },
  { id: 'scanmutti', name: 'Scanmutti – Paketzuführung', make: sceneScanmutti },
  { id: 'rundlauf', name: 'Rundlauf XL – Dreiwege-Sorter', make: sceneRundlauf },
  { id: 'horst_solo', name: 'HORST600 – Solo', make: sceneHorstSolo },
  { id: 'pendulum',  name: 'Beispiel: Pendelkette', make: scenePendulum },
  { id: 'stack',     name: 'Beispiel: Kistenstapel', make: sceneStack },
  { id: 'humanoid',  name: 'Beispiel: Humanoid (DeepMind)', url: 'models/humanoid.xml' },
];
