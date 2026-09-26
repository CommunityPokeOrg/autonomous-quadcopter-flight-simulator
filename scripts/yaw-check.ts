import { Simulation } from '../src/sim/Simulation';
import { DRONE_PRESETS } from '../src/sim/drones';
import { wrapAngle } from '../src/control/constants';

// Verifies autonomous yaw tracking:
//  1. wrapAngle math, incl. the ±π boundary.
//  2. A synthetic mission that forces a ~169° turn followed by a +21° turn
//     across the wrap boundary — the drone must converge on each bearing and
//     take the short way around (leg rotation ≈ |Δbearing|, not 2π − |Δ|).
//  3. Every preset's real mission still completes with bounded total yaw
//     rotation — regression check for the pursuit/pirouette failure where the
//     tracker chased the bearing while hovering over a waypoint.

const DT = 1 / 240;
const MAX_T = 240;
const MIN_LEG_DIST = 1.5; // m: bearing tracking is unambiguously active beyond this
// (the tracker latches below 1.0 m, so 1.0–1.5 m stays out of the noisy zone)

let failed = false;
const fail = (msg: string) => {
  console.log(`FAIL: ${msg}`);
  failed = true;
};
const check = (cond: boolean, msg: string) => {
  if (!cond) fail(msg);
};

// --- 1. wrapAngle unit checks ---
const cases: [number, number][] = [
  [0, 0],
  [Math.PI / 2, Math.PI / 2],
  [-Math.PI / 2, -Math.PI / 2],
  [Math.PI, -Math.PI], // boundary folds to -π
  [-Math.PI, -Math.PI],
  [(3 * Math.PI) / 2, -Math.PI / 2],
  [(-3 * Math.PI) / 2, Math.PI / 2],
  [2 * Math.PI + 0.3, 0.3],
  [-2 * Math.PI - 0.3, -0.3],
  [10 * Math.PI + 1, 1],
];
for (const [input, expected] of cases) {
  const got = wrapAngle(input);
  check(
    Math.abs(got - expected) < 1e-9,
    `wrapAngle(${input.toFixed(4)}) = ${got.toFixed(4)}, expected ${expected.toFixed(4)}`,
  );
}
console.log('wrapAngle unit checks done');

const ids = process.argv.slice(2);
const toRun = ids.length ? ids : Object.keys(DRONE_PRESETS);

/** Per-leg stats: heading-error samples (dxy > MIN_LEG_DIST) and ∫|d yaw|. */
interface LegStats {
  errs: number[];
  rot: number;
}

function fly(id: string, waypoints: { x: number; y: number; z: number; name: string }[]): {
  legs: Map<number, LegStats>;
  done: boolean;
  totalRot: number;
  maxWz: number;
  time: number;
} {
  const sim = new Simulation(id);
  const nav = sim.controller.navigator;
  nav.waypoints = waypoints.map((w) => ({ ...w }));
  nav.reset();
  sim.start();

  const legs = new Map<number, LegStats>();
  let totalRot = 0;
  let maxWz = 0;
  let prevYaw = 0;
  let prevInit = false;

  for (let i = 0; i < MAX_T / DT; i++) {
    sim.step(DT);
    const s = sim.model.state;
    check(
      isFinite(s.position.x + s.position.y + s.position.z + s.euler.yaw),
      `${id}: diverged at t=${sim.time.toFixed(2)}`,
    );
    const wz = s.angularVelocity.z;
    maxWz = Math.max(maxWz, Math.abs(wz));
    const dYaw = prevInit ? Math.abs(wrapAngle(s.euler.yaw - prevYaw)) : 0;
    totalRot += dYaw;
    prevYaw = s.euler.yaw;
    prevInit = true;

    const wp = nav.current;
    if (wp) {
      const st = legs.get(nav.index) ?? { errs: [], rot: 0 };
      st.rot += dYaw;
      const dx = wp.x - s.position.x;
      const dy = wp.y - s.position.y;
      if (!nav.done && Math.hypot(dx, dy) > MIN_LEG_DIST) {
        st.errs.push(Math.abs(wrapAngle(Math.atan2(dy, dx) - s.euler.yaw)));
      }
      legs.set(nav.index, st);
    }
    if (nav.done) break;
  }
  return { legs, done: sim.controller.navigator.done, totalRot, maxWz, time: sim.time };
}

// --- 2. synthetic wraparound mission ---
// takeoff straight up (heading must hold), then a long leg at bearing ≈ +170°
// (long enough to finish a ~170° turn and settle), then a leg at bearing
// ≈ -170°: correct wrap turns ~+20° CCW, broken wrap would turn ~340° CW.
for (const id of toRun) {
  const cfg = DRONE_PRESETS[id];
  if (!cfg) {
    fail(`unknown drone preset '${id}'`);
    continue;
  }
  console.log(`=== ${id} wraparound mission ===`);
  const r = fly(id, [
    { x: 0, y: 0, z: 4, name: 'up' },
    { x: -30, y: 5, z: 4, name: 'leg1' },
    { x: -44.8, y: 2.4, z: 4, name: 'leg2' },
    { x: 0, y: 0, z: 4, name: 'home' },
  ]);
  check(r.done, `${id}: wraparound mission timed out`);

  const leg0 = r.legs.get(0);
  const leg1 = r.legs.get(1);
  const leg2 = r.legs.get(2);
  if (leg0) {
    console.log(`  leg0 (takeoff): rot=${leg0.rot.toFixed(3)} rad`);
    check(leg0.rot < 0.15, `${id}: heading drifted ${leg0.rot.toFixed(2)} rad on vertical takeoff`);
  }
  if (!leg1 || leg1.errs.length < 20 || !leg2 || leg2.errs.length < 20) {
    fail(`${id}: not enough samples on wraparound legs`);
    continue;
  }
  const min1 = Math.min(...leg1.errs);
  const min2 = Math.min(...leg2.errs);
  const med1 = [...leg1.errs].sort((a, b) => a - b)[Math.floor(leg1.errs.length / 2)]!;
  const med2 = [...leg2.errs].sort((a, b) => a - b)[Math.floor(leg2.errs.length / 2)]!;
  console.log(
    `  leg1 (→170°): min |err|=${min1.toFixed(3)} median=${med1.toFixed(3)} rot=${leg1.rot.toFixed(2)} rad\n` +
      `  leg2 (→-170°): min |err|=${min2.toFixed(3)} median=${med2.toFixed(3)} rot=${leg2.rot.toFixed(2)} rad`,
  );
  check(min1 < 0.25, `${id}: leg1 never pointed at waypoint (min err ${min1.toFixed(2)})`);
  check(leg1.rot < 4.5, `${id}: leg1 yawed ${leg1.rot.toFixed(2)} rad — long way around?`);
  check(min2 < 0.12, `${id}: leg2 never pointed at waypoint (min err ${min2.toFixed(2)})`);
  check(med2 < 0.2, `${id}: leg2 cruise heading error ${med2.toFixed(2)} rad`);
  check(leg2.rot < 1.5, `${id}: leg2 yawed ${leg2.rot.toFixed(2)} rad — did not wrap at ±π`);
}

// --- 3. preset mission regression ---
for (const id of toRun) {
  const cfg = DRONE_PRESETS[id];
  if (!cfg) continue;
  console.log(`=== ${id} preset mission ===`);
  const r = fly(id, cfg.mission);
  console.log(`  t=${r.time.toFixed(1)}s total yaw rot=${r.totalRot.toFixed(2)} rad max |wz|=${r.maxWz.toFixed(2)}`);
  check(r.done, `${id}: preset mission timed out`);
  check(r.totalRot < 14, `${id}: yawed ${r.totalRot.toFixed(1)} rad total — hunting/pursuit?`);
  check(
    r.maxWz < cfg.control.yawTrack.maxRate * 2 + 0.3,
    `${id}: yaw rate ${r.maxWz.toFixed(2)} rad/s far above maxRate`,
  );
}

console.log(failed ? 'FAILED' : 'OK');
process.exit(failed ? 1 : 0);
