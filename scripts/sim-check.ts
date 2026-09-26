import { Simulation } from '../src/sim/Simulation';
import { DRONE_PRESETS } from '../src/sim/drones';

// Run the full autonomous mission headlessly for each airframe (or the ids
// given as CLI args), verifying every waypoint is reached and it lands.
const ids = process.argv.slice(2);
const toRun = ids.length ? ids : Object.keys(DRONE_PRESETS);
const DT = 1 / 240;
const MAX_T = 180;

let failed = false;
for (const id of toRun) {
  if (!DRONE_PRESETS[id]) {
    console.log(`unknown drone preset '${id}'`);
    failed = true;
    continue;
  }
  console.log(`=== ${id} ===`);
  const sim = new Simulation(id);
  sim.start();
  let reached = 0;
  const reachedIdx = new Set<number>();
  for (let i = 0; i < MAX_T / DT; i++) {
    sim.step(DT);
    const s = sim.model.state;
    if (!isFinite(s.position.x + s.position.y + s.position.z)) {
      console.log(`DIVERGED at t=${sim.time.toFixed(2)}`);
      failed = true;
      break;
    }
    const nav = sim.controller.navigator;
    const wp = nav.current;
    if (wp) {
      const d = nav.distanceTo(s.position.x, s.position.y, s.position.z);
      if (d < 0.4 && !reachedIdx.has(nav.index)) {
        reachedIdx.add(nav.index);
        reached++;
        console.log(`t=${sim.time.toFixed(1)}s reached wp ${nav.index} (${wp.name})`);
      }
    }
    if (nav.done) {
      console.log(
        `MISSION DONE at t=${sim.time.toFixed(1)}s, landed=${sim.landed}, z=${s.position.z.toFixed(2)}`,
      );
      break;
    }
  }
  const s = sim.model.state;
  console.log(
    `final pos ${s.position.x.toFixed(2)},${s.position.y.toFixed(2)},${s.position.z.toFixed(2)} reached=${reached} done=${sim.controller.navigator.done}`,
  );
  if (!sim.controller.navigator.done) {
    console.log('TIMEOUT — mission not complete');
    failed = true;
  }
}
console.log(failed ? 'FAILED' : 'OK');
process.exit(failed ? 1 : 0);
