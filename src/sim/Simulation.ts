import * as CANNON from 'cannon-es';
import { QuadcopterModel } from './QuadcopterModel';
import { DRONE_PRESETS, DEFAULT_DRONE_ID, type DroneConfig } from './drones';
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
  model: QuadcopterModel;
  controller: FlightController;
  readonly trajectory: TrajectorySample[] = [];
  private droneMat: CANNON.Material;

  running = false;
  time = 0;
  speed = 1; // sim speed multiplier
  private trajTimer = 0;
  manualInput: ManualInput = { pitch: 0, roll: 0, yawRate: 0, throttle: 0 };

  constructor(droneId: string = DEFAULT_DRONE_ID) {
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
    this.droneMat = new CANNON.Material('drone');
    ground.material = groundMat;
    this.world.addBody(ground);
    this.world.addContactMaterial(
      new CANNON.ContactMaterial(groundMat, this.droneMat, {
        friction: 0.3,
        restitution: 0.1,
      }),
    );

    const cfg = DRONE_PRESETS[droneId] ?? DRONE_PRESETS[DEFAULT_DRONE_ID]!;
    this.model = this.addDrone(cfg);
    this.controller = new FlightController(cfg);
  }

  get droneConfig(): DroneConfig {
    return this.model.config;
  }

  private addDrone(cfg: DroneConfig): QuadcopterModel {
    const model = new QuadcopterModel(cfg);
    model.body.material = this.droneMat;
    this.world.addBody(model.body);
    return model;
  }

  /**
   * Swap in a different airframe preset: rebuilds the rigid body and the
   * whole controller stack (incl. its mission), then resets the sim.
   */
  setDrone(id: string): void {
    const cfg = DRONE_PRESETS[id];
    if (!cfg || cfg === this.model.config) return;
    this.world.removeBody(this.model.body);
    this.model = this.addDrone(cfg);
    this.controller = new FlightController(cfg);
    this.reset();
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
