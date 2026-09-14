import { Simulation } from '../src/sim/Simulation';

const sim = new Simulation();
sim.start();
const DT = 1 / 240;
let maxErr = 0;
let reached = 0;
const reachedIdx = new Set<number>();
for (let i = 0; i < 240 * 90; i++) {
  sim.step(DT);
  const s = sim.model.state;
  if (!isFinite(s.position.x + s.position.y + s.position.z)) {
    console.log(`DIVERGED at t=${sim.time.toFixed(2)}`);
    process.exit(1);
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
    maxErr = Math.max(maxErr, d > 30 ? 0 : 0);
  }
  if (nav.done) {
    console.log(`MISSION DONE at t=${sim.time.toFixed(1)}s, landed=${sim.landed}, z=${s.position.z.toFixed(2)}`);
    break;
  }
}
const s = sim.model.state;
console.log(`final pos ${s.position.x.toFixed(2)},${s.position.y.toFixed(2)},${s.position.z.toFixed(2)} reached=${reached} done=${sim.controller.navigator.done}`);
if (!sim.controller.navigator.done) {
  console.log('TIMEOUT — mission not complete');
  process.exit(1);
}
console.log('OK');
