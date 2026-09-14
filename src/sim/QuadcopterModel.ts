import * as CANNON from 'cannon-es';

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

const ARM = 0.25; // m, rotor distance from center
const MASS = 1.2; // kg
const KT = 8.0e-6; // thrust coefficient: T = kT * w^2  (N per (rad/s)^2)
const KQ = 1.4e-7; // torque coefficient
const OMEGA_MAX = 1100; // rad/s motor saturation
const TAU_MOTOR = 0.02; // s, first-order rotor lag
const LIN_DRAG = 0.25; // N per (m/s)
const ANG_DRAG = 0.012; // N·m per (rad/s)

// X configuration: rotors at 45° diagonals.
// Order: FL(+x,+y)  FR(+x,-y)  RR(-x,-y)  RL(-x,+y)  in body frame (x fwd, y left, z up)
const ROTOR_OFFSETS: CANNON.Vec3[] = [
  new CANNON.Vec3(ARM * Math.SQRT1_2, ARM * Math.SQRT1_2, 0),
  new CANNON.Vec3(ARM * Math.SQRT1_2, -ARM * Math.SQRT1_2, 0),
  new CANNON.Vec3(-ARM * Math.SQRT1_2, -ARM * Math.SQRT1_2, 0),
  new CANNON.Vec3(-ARM * Math.SQRT1_2, ARM * Math.SQRT1_2, 0),
];
// spin directions: +1 CCW (seen from top), -1 CW — FL & RR one way, FR & RL the other
const SPIN_DIRS = [1, -1, 1, -1];

export const ROTOR_POSITIONS = ROTOR_OFFSETS;
export const ROTOR_SPIN = SPIN_DIRS;
export const ARM_LENGTH = ARM;
export const MOTOR_OMEGA_MAX = OMEGA_MAX;
export const HOVER_OMEGA = Math.sqrt((MASS * 9.81) / (4 * KT));

export class QuadcopterModel {
  readonly body: CANNON.Body;
  private omegas = [0, 0, 0, 0];
  private omegaCmds = [0, 0, 0, 0];

  constructor() {
    this.body = new CANNON.Body({
      mass: MASS,
      shape: new CANNON.Box(new CANNON.Vec3(0.09, 0.09, 0.03)),
      angularDamping: 0,
      linearDamping: 0,
    });
    // box inertia from arm length
    this.body.inertia.set(
      (MASS * ARM * ARM) / 6,
      (MASS * ARM * ARM) / 6,
      (MASS * ARM * ARM) / 3,
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

  /** Commanded rotor speeds in rad/s, one per rotor, clamped to [0, OMEGA_MAX]. */
  setMotorCommands(cmds: number[]): void {
    for (let i = 0; i < 4; i++) {
      const c = cmds[i] ?? 0;
      this.omegaCmds[i] = Math.min(OMEGA_MAX, Math.max(0, c));
    }
  }

  /** First-order rotor lag toward commanded speed. Call each substep. */
  private updateMotors(dt: number): void {
    const alpha = 1 - Math.exp(-dt / TAU_MOTOR);
    for (let i = 0; i < 4; i++) {
      this.omegas[i]! += alpha * (this.omegaCmds[i]! - this.omegas[i]!);
    }
  }

  /** Apply thrust, reaction torque and drag. Call each physics substep. */
  applyForces(dt: number): void {
    this.updateMotors(dt);
    const body = this.body;

    // body z axis in world frame
    const bodyZ = body.quaternion.vmult(new CANNON.Vec3(0, 0, 1));

    let yawTorque = 0;
    for (let i = 0; i < 4; i++) {
      const w = this.omegas[i]!;
      const thrust = KT * w * w;
      // applyLocalForce expects force & point in body-local frame
      body.applyLocalForce(new CANNON.Vec3(0, 0, thrust), ROTOR_OFFSETS[i]!);
      // reaction torque on the airframe opposes the rotor spin direction
      yawTorque -= SPIN_DIRS[i]! * KQ * w * w;
    }
    const tq = bodyZ.scale(yawTorque);
    body.applyTorque(tq);

    // linear drag (world frame)
    const drag = body.velocity.scale(-LIN_DRAG);
    body.applyForce(drag, new CANNON.Vec3(0, 0, 0));

    // angular drag in body frame
    const wBody = body.quaternion.conjugate().vmult(body.angularVelocity);
    const adBody = wBody.scale(-ANG_DRAG);
    const adWorld = body.quaternion.vmult(adBody);
    body.applyTorque(adWorld);
  }

  get mass(): number {
    return MASS;
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
    for (const w of this.omegas) totalThrust += KT * w * w;
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
