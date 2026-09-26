import * as CANNON from 'cannon-es';
import { GRAVITY } from '../control/constants';
import { DRONE_PRESETS, DEFAULT_DRONE_ID, type DroneConfig } from './drones';

export interface QuadState {
  position: CANNON.Vec3;
  velocity: CANNON.Vec3;
  quaternion: CANNON.Quaternion;
  /** roll (x), pitch (y), yaw (z) in radians, aerospace-ish ZYX order */
  euler: { roll: number; pitch: number; yaw: number };
  angularVelocity: CANNON.Vec3;
  motorOmegas: number[];
  totalThrust: number;
}

// spin directions: +1 CCW (seen from top), -1 CW — FL & RR one way, FR & RL the other
const SPIN_DIRS = [1, -1, 1, -1];

// X configuration: rotors at 45° diagonals.
// Order: FL(+x,+y)  FR(+x,-y)  RR(-x,-y)  RL(-x,+y)  in body frame (x fwd, y left, z up)
function rotorOffsets(arm: number): CANNON.Vec3[] {
  const a = arm * Math.SQRT1_2;
  return [
    new CANNON.Vec3(a, a, 0),
    new CANNON.Vec3(a, -a, 0),
    new CANNON.Vec3(-a, -a, 0),
    new CANNON.Vec3(-a, a, 0),
  ];
}

export class QuadcopterModel {
  readonly config: DroneConfig;
  readonly body: CANNON.Body;
  readonly rotorPositions: CANNON.Vec3[];
  readonly rotorSpin = SPIN_DIRS;
  private omegas = [0, 0, 0, 0];
  private omegaCmds = [0, 0, 0, 0];

  constructor(config: DroneConfig = DRONE_PRESETS[DEFAULT_DRONE_ID]!) {
    this.config = config;
    this.rotorPositions = rotorOffsets(config.arm);
    const [hx, hy, hz] = config.bodyHalfExtents;
    this.body = new CANNON.Body({
      mass: config.mass,
      shape: new CANNON.Box(new CANNON.Vec3(hx!, hy!, hz!)),
      angularDamping: 0,
      linearDamping: 0,
    });
    // box inertia from arm length
    const { mass, arm } = config;
    this.body.inertia.set(
      (mass * arm * arm) / 6,
      (mass * arm * arm) / 6,
      (mass * arm * arm) / 3,
    );
    this.body.updateMassProperties();
    this.body.sleepSpeedLimit = 0.15;
    this.body.sleepTimeLimit = 0.6;
    this.reset();
  }

  reset(): void {
    this.body.position.set(0, 0, 0.05);
    this.body.velocity.set(0, 0, 0);
    this.body.angularVelocity.set(0, 0, 0);
    this.body.quaternion.set(0, 0, 0, 1);
    this.body.force.set(0, 0, 0);
    this.body.torque.set(0, 0, 0);
    this.omegas = [0, 0, 0, 0];
    this.omegaCmds = [0, 0, 0, 0];
    this.body.wakeUp();
  }

  /** Rotor speed that exactly hovers: T_total = m·g. */
  get hoverOmega(): number {
    return Math.sqrt((this.config.mass * GRAVITY) / (4 * this.config.kT));
  }

  /** Commanded rotor speeds in rad/s, one per rotor, clamped to [0, omegaMax]. */
  setMotorCommands(cmds: number[]): void {
    const max = this.config.omegaMax;
    for (let i = 0; i < 4; i++) {
      const c = cmds[i] ?? 0;
      this.omegaCmds[i] = Math.min(max, Math.max(0, c));
    }
  }

  /** First-order rotor lag toward commanded speed. Call each substep. */
  private updateMotors(dt: number): void {
    const alpha = 1 - Math.exp(-dt / this.config.tauMotor);
    for (let i = 0; i < 4; i++) {
      this.omegas[i]! += alpha * (this.omegaCmds[i]! - this.omegas[i]!);
    }
  }

  /** Apply thrust, reaction torque and drag. Call each physics substep. */
  applyForces(dt: number): void {
    this.updateMotors(dt);
    const body = this.body;
    const { kT, kQ, linDrag, angDrag } = this.config;

    // body z axis in world frame
    const bodyZ = body.quaternion.vmult(new CANNON.Vec3(0, 0, 1));

    let yawTorque = 0;
    for (let i = 0; i < 4; i++) {
      const w = this.omegas[i]!;
      const thrust = kT * w * w;
      // applyLocalForce expects force & point in body-local frame
      body.applyLocalForce(new CANNON.Vec3(0, 0, thrust), this.rotorPositions[i]!);
      // reaction torque on the airframe opposes the rotor spin direction
      yawTorque -= SPIN_DIRS[i]! * kQ * w * w;
    }
    const tq = bodyZ.scale(yawTorque);
    body.applyTorque(tq);

    // linear drag (world frame)
    const drag = body.velocity.scale(-linDrag);
    body.applyForce(drag, new CANNON.Vec3(0, 0, 0));

    // angular drag in body frame
    const wBody = body.quaternion.conjugate().vmult(body.angularVelocity);
    const adBody = wBody.scale(-angDrag);
    const adWorld = body.quaternion.vmult(adBody);
    body.applyTorque(adWorld);
  }

  get mass(): number {
    return this.config.mass;
  }

  get state(): QuadState {
    // ZYX euler extraction for body frame x-fwd, y-left, z-up.
    // Positive pitch about +y is nose-DOWN in this frame, so we negate it
    // to report pitch as nose-up positive.
    const q = this.body.quaternion;
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    const pitchMat = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
    const pitch = -pitchMat;
    const roll = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    let totalThrust = 0;
    for (const w of this.omegas) totalThrust += this.config.kT * w * w;
    return {
      position: this.body.position.clone(),
      velocity: this.body.velocity.clone(),
      quaternion: this.body.quaternion.clone(),
      euler: { roll, pitch, yaw },
      angularVelocity: this.body.quaternion.conjugate().vmult(this.body.angularVelocity),
      motorOmegas: [...this.omegas],
      totalThrust,
    };
  }
}
