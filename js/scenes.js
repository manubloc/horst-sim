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

const KP = [400, 400, 250, 60, 50, 25];
const KV = [40, 40, 25, 6, 5, 2.5];
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
    <body name="${p}horst_basis" pos="${v(pos)}" euler="0 0 ${zrot}" gravcomp="0">
      <geom type="box" size="0.085 0.085 0.011" pos="0 0 0.011" rgba="${HORST.black}" mass="1.2"/>
      <geom type="cylinder" size="0.075 0.037" pos="0 0 0.056" rgba="${HORST.dark}" mass="4.5"/>
      <body name="${p}link_1" pos="${v(J[0].org)}">
        ${joint(0)}
        <geom type="cylinder" size="0.062 0.075" pos="0 0 0.078" rgba="${HORST.teal}" mass="2.6"/>
        <geom type="cylinder" size="0.052 0.052" pos="0 0.004 0.251" euler="1.5708 0 0" rgba="${HORST.dark}" mass="0.9"/>
        <body name="${p}link_2" pos="${v(J[1].org)}">
          ${joint(1)}
          <geom type="capsule" fromto="0 0.052 0  0 0.052 0.274" size="0.045" rgba="${HORST.teal}" mass="2.2"/>
          <geom type="cylinder" size="0.046 0.03" pos="0 0.02 0.274" euler="1.5708 0 0" rgba="${HORST.dark}" mass="0.4"/>
          <body name="${p}link_3" pos="${v(J[2].org)}">
            ${joint(2)}
            <geom type="box" size="0.048 0.05 0.045" pos="0.012 0 0.012" rgba="${HORST.dark}" mass="1.4"/>
            <geom type="capsule" fromto="0 0 0  0.0855 0 0.0555" size="0.036" rgba="${HORST.black}" mass="0.4"/>
            <body name="${p}link_4" pos="${v(J[3].org)}">
              ${joint(3)}
              <geom type="capsule" fromto="0 0 0  0.1817 0 0" size="0.036" rgba="${HORST.teal}" mass="1.0"/>
              <geom type="cylinder" size="0.037 0.032" pos="0.1817 0 0" euler="1.5708 0 0" rgba="${HORST.dark}" mass="0.35"/>
              <body name="${p}link_5" pos="${v(J[4].org)}">
                ${joint(4)}
                <geom type="box" size="0.03 0.026 0.03" pos="0.018 0 0" rgba="${HORST.light}" mass="0.45"/>
                <body name="${p}link_6" pos="${v(J[5].org)}">
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

  const actuators = J.map((j, i) =>
    `<position name="${p}A${i + 1}" joint="${jn(i)}" kp="${KP[i]}" kv="${KV[i]}" ` +
    `forcerange="-${FR[i]} ${FR[i]}" ctrlrange="${v(j.range)}"/>`).join('\n    ');

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
      <geom type="box" size="0.42 0.30 0.018" pos="0 0 0.352" rgba="0.20 0.22 0.23 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.39 0.27 0.176" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.39 -0.27 0.176" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.39 0.27 0.176" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.39 -0.27 0.176" rgba="0.12 0.13 0.14 1"/>
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

export const SCENES = [
  { id: 'horst_cell', name: 'HORST600 – Arbeitszelle', make: sceneHorstCell },
  { id: 'horst_solo', name: 'HORST600 – Solo', make: sceneHorstSolo },
  { id: 'pendulum',  name: 'Beispiel: Pendelkette', make: scenePendulum },
  { id: 'stack',     name: 'Beispiel: Kistenstapel', make: sceneStack },
  { id: 'humanoid',  name: 'Beispiel: Humanoid (DeepMind)', url: 'models/humanoid.xml' },
];
