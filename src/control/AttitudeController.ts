import { PID } from './PID';
import type { DroneConfig } from '../sim/drones';

export interface AttitudeSetpoint {
  roll: number; // rad
  pitch: number; // rad
  yawRate: number; // rad/s
  thrust: number; // N total
}

export interface AttitudeMeasurement {
  roll: number;
  pitch: number;
  rollRate: number;
  pitchRate: number;
  yawRate: number;
}

/**
 * Angle-mode attitude controller: PID on roll/pitch angle error and yaw rate,
 * mixed to 4 motor commands for the X configuration.
 *
 * Body frame: x forward, y left, z up.
 * Rotor layout (X): 0 FL(+x,+y,CCW), 1 FR(+x,-y,CW), 2 RR(-x,-y,CCW), 3 RL(-x,+y,CW)
 *
 * roll torque (+roll = right side down): increase left rotors (0,3), decrease right (1,2)
 * pitch torque (+pitch = nose up): increase rear rotors (2,3), decrease front (0,1)
 * yaw torque (+yaw = CCW): reaction torque = -spin*kQ*w^2, so to yaw CCW increase CW rotors (1,3)
 */
export class AttitudeController {
  private rollPID: PID;
  private pitchPID: PID;
  private yawRatePID: PID;
  private readonly cfg: DroneConfig;

  constructor(cfg: DroneConfig) {
    this.cfg = cfg;
    const { attitude, yawRate } = cfg.control;
    this.rollPID = new PID(attitude.gains, attitude.out, attitude.int);
    this.pitchPID = new PID(attitude.gains, attitude.out, attitude.int);
    this.yawRatePID = new PID(yawRate.gains, yawRate.out, yawRate.int);
  }

  update(sp: AttitudeSetpoint, m: AttitudeMeasurement, dt: number): number[] {
    const tauX = this.rollPID.update(sp.roll, m.roll, dt);
    // pitch sp/measurement are nose-up positive; torque about +y is nose-down,
    // so negate the PID output before mixing.
    const tauY = -this.pitchPID.update(sp.pitch, m.pitch, dt);
    const tauZ = this.yawRatePID.update(sp.yawRate, m.yawRate, dt);
    return this.mixer(sp.thrust, tauX, tauY, tauZ);
  }

  /**
   * Solve for motor speeds from desired total thrust + body torques.
   * T_i = kT w_i^2, tau_z contribution = -spin*kQ w_i^2.
   * Invert the linear system in "w^2" space, then sqrt.
   */
  private mixer(thrust: number, tauX: number, tauY: number, tauZ: number): number[] {
    // Per-rotor w^2 contributions. Arm components all equal a = arm/sqrt(2).
    const a = this.cfg.arm / Math.SQRT2;
    const kT = this.cfg.kT;
    const kQ = this.cfg.kQ;

    // tau_x = a * kT * (w0^2 + w3^2 - w1^2 - w2^2)   (left minus right)
    // tau_y = a * kT * (w2^2 + w3^2 - w0^2 - w1^2)   (rear minus front)
    // tau_z = -kQ * (w0^2 - w1^2 + w2^2 - w3^2)      (CCW rotors drag body CW)
    // thrust = kT * sum(w_i^2)
    const S = thrust / (4 * kT); // mean w^2
    const dX = tauX / (2 * a * kT);
    const dY = tauY / (2 * a * kT);
    const dZ = -tauZ / (2 * kQ);

    const w2 = [
      S + dX / 2 - dY / 2 + dZ / 2, // 0 FL: left, front, CCW
      S - dX / 2 - dY / 2 - dZ / 2, // 1 FR: right, front, CW
      S - dX / 2 + dY / 2 + dZ / 2, // 2 RR: right, rear, CCW
      S + dX / 2 + dY / 2 - dZ / 2, // 3 RL: left, rear, CW
    ];
    return w2.map((v) => Math.sqrt(Math.max(0, v)));
  }

  reset(): void {
    this.rollPID.reset();
    this.pitchPID.reset();
    this.yawRatePID.reset();
  }
}
