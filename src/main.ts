import { Simulation } from './sim/Simulation';
import { Renderer3D } from './render/Scene';
import { HUD } from './ui/HUD';

const PHYS_DT = 1 / 240;
const MAX_FRAME_DT = 0.1;
const MAX_SUBSTEPS = 80;

const canvas = document.getElementById('scene') as HTMLCanvasElement;
const hudRoot = document.getElementById('hud') as HTMLElement;

const sim = new Simulation();
const renderer = new Renderer3D(canvas, sim);
renderer.syncWaypoints();

const keys = new Set<string>();
window.addEventListener('keydown', (e) => keys.add(e.code));
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());

function pollManualInput(): void {
  const k = keys;
  sim.manualInput.pitch = (k.has('KeyW') ? 1 : 0) - (k.has('KeyS') ? 1 : 0);
  sim.manualInput.roll = (k.has('KeyD') ? 1 : 0) - (k.has('KeyA') ? 1 : 0);
  sim.manualInput.yawRate = (k.has('KeyQ') ? 1 : 0) - (k.has('KeyE') ? 1 : 0);
  sim.manualInput.throttle =
    (k.has('KeyR') || k.has('ShiftLeft') || k.has('ShiftRight') ? 1 : 0) -
    (k.has('KeyF') || k.has('ControlLeft') || k.has('ControlRight') ? 1 : 0);
}

const hud = new HUD(hudRoot, sim, {
  onStartPause: () => (sim.running ? sim.pause() : sim.start()),
  onReset: () => {
    sim.reset();
    renderer.syncWaypoints();
  },
  onModeChange: (mode) => sim.controller.setMode(mode),
  onCameraChange: (chase) => renderer.setChase(chase),
  onSpeedChange: (v) => (sim.speed = v),
  onLoopChange: (loop) => (sim.controller.navigator.loop = loop),
});

// expose for headless testing
(window as unknown as { __sim: Simulation }).__sim = sim;

let last = performance.now();
let accumulator = 0;
let elapsed = 0;

function frame(now: number): void {
  requestAnimationFrame(frame);
  let dt = (now - last) / 1000;
  last = now;
  if (dt > MAX_FRAME_DT) dt = MAX_FRAME_DT; // tab-switch clamp
  elapsed += dt;

  if (sim.running) {
    pollManualInput();
    accumulator += dt * sim.speed;
    let steps = 0;
    while (accumulator >= PHYS_DT && steps < MAX_SUBSTEPS) {
      sim.step(PHYS_DT);
      accumulator -= PHYS_DT;
      steps++;
    }
    if (steps >= MAX_SUBSTEPS) accumulator = 0; // drop backlog after long stall
  }

  renderer.render(dt, elapsed);
  hud.update();
}

sim.start();
requestAnimationFrame(frame);
