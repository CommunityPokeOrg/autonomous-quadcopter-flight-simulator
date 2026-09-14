import * as CANNON from 'cannon-es';
import { QuadcopterModel } from './QuadcopterModel';
import { FlightController, type FlightMode, type ManualInput } from '../control/FlightController';

export interface TrajectorySample {
  x: number;
  y: number;
  z: number;
  t: number;
}

export type RunState = 'running' | 'paused' | 'stopped';

const TRAJ_MAX = 2000;
const TRAJ_DT = 0.05;

export class Simulation {
  readonly world: CANNON.World;
  readonly model: QuadcopterModel;
  readonly controller: FlightController;
  readonly trajectory: TrajectorySample[] = [];

  running = false;
  time = 0;
  speed = 1; // sim speed multiplier
  private trajTimer = 0;
  manualInput: ManualInput = { pitch: 0, roll: 0, yawRate: 0, throttle: 0 };

  constructor() {
    this.world = new CANNON.World({ gravity: new CANNON.Vec3(0, 0, -9.81) });
    this.world.broadphase = new CANNON.SAPBroadphase(this.world);
    this.world.allowSleep = true;

    const ground = new CANNON.Body({
      mass: 0,
      shape: new CANNON.Plane(),
      position: new CANNON.Vec3(0, 0, 0),
    });
    // plane's normal is +z by default in cannon-es — already correct
    const groundMat = new CANNON.Material('ground');
    const droneMat = new CANNON.Material('drone');
    ground.material = groundMat;
    this.world.addBody(ground);
    this.world.addContactMaterial(
      new CANNON.ContactMaterial(groundMat, droneMat, {
        friction: 0.3,
        restitution: 0.1,
      }),
    );

    this.model = new QuadcopterModel();
    this.model.body.material = droneMat;
    this.world.addBody(this.model.body);

    this.controller = new FlightController();
  }

  get mode(): FlightMode {
    return this.controller.mode;
  }

  start(): void {
    this.running = true;
    this.model.body.wakeUp();
  }

  pause(): void {
    this.running = false;
  }

  reset(): void {
    this.running = false;
    this.time = 0;
    this.trajTimer = 0;
    this.trajectory.length = 0;
    this.model.reset();
    this.controller.reset();
    this.manualInput = { pitch: 0, roll: 0, yawRate: 0, throttle: 0 };
  }

  /** Advance physics by dt seconds (already scaled by sim speed). */
  step(dt: number): void {
    this.controller.update(this.model, this.manualInput, dt);
    this.model.applyForces(dt);
    this.world.step(dt);
    this.time += dt;
    this.trajTimer += dt;
    if (this.trajTimer >= TRAJ_DT) {
      this.trajTimer = 0;
      const p = this.model.body.position;
      this.trajectory.push({ x: p.x, y: p.y, z: p.z, t: this.time });
      if (this.trajectory.length > TRAJ_MAX) this.trajectory.shift();
    }
  }

  get landed(): boolean {
    const s = this.model.state;
    return s.position.z < 0.25 && s.velocity.length() < 0.3;
  }
}
