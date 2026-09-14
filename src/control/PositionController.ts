import { PID } from './PID';
import { GRAVITY, MASS_KG } from './constants';

export interface PositionSetpoint {
  x: number;
  y: number;
  z: number;
}

export interface PositionMeasurement {
  x: number;
  y: number;
  z: number;
  vx: number;
  vy: number;
  vz: number;
  yaw: number; // rad
  roll: number; // rad, current (for tilt compensation)
  pitch: number; // rad, nose-up positive
}

export interface AttitudeDemand {
  roll: number;
  pitch: number;
  thrust: number; // N
}

const MAX_TILT = (30 * Math.PI) / 180;
const MAX_SPEED = 6; // m/s soft speed cap via velocity feedforward

/**
 * Outer loop: desired position -> desired roll/pitch (in the body-yaw frame)
 * plus collective thrust with gravity and tilt compensation.
 */
export class PositionController {
  private xPID = new PID({ kp: 1.6, ki: 0.25, kd: 1.9 }, 8, 2);
  private yPID = new PID({ kp: 1.6, ki: 0.25, kd: 1.9 }, 8, 2);
  private zPID = new PID({ kp: 4.0, ki: 0.6, kd: 2.6 }, 8, 2);

  update(sp: PositionSetpoint, m: PositionMeasurement, dt: number): AttitudeDemand {
    // horizontal PIDs output desired accelerations (m/s^2)
    const axW = this.xPID.update(sp.x, m.x, dt);
    const ayW = this.yPID.update(sp.y, m.y, dt);
    const azW = this.zPID.update(sp.z, m.z, dt);

    // rotate world accel demand into the yaw (body-horizontal) frame
    const cy = Math.cos(m.yaw);
    const sy = Math.sin(m.yaw);
    const aFwd = cy * axW + sy * ayW; // along body x
    const aLat = -sy * axW + cy * ayW; // along body y (left)

    // small-angle: a_fwd = -g*pitch(nose-up), a_lat = -g*roll
    let pitch = -aFwd / GRAVITY;
    let roll = -aLat / GRAVITY;
    const tilt = Math.hypot(roll, pitch);
    if (tilt > MAX_TILT) {
      const s = MAX_TILT / tilt;
      roll *= s;
      pitch *= s;
    }

    // collective thrust: m(g+az) / (cos roll * cos pitch)
    const cosTilt = Math.cos(roll) * Math.cos(pitch);
    const thrust = (MASS_KG * (GRAVITY + azW)) / Math.max(0.3, cosTilt);

    return { roll, pitch, thrust };
  }

  reset(): void {
    this.xPID.reset();
    this.yPID.reset();
    this.zPID.reset();
  }
}

export { MAX_TILT, MAX_SPEED };
