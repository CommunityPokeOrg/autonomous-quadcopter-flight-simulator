import { AttitudeController } from './AttitudeController';
import { PositionController } from './PositionController';
import { WaypointNavigator } from './WaypointNavigator';
import type { QuadcopterModel } from '../sim/QuadcopterModel';
import { GRAVITY, MASS_KG } from './constants';
import { HOVER_OMEGA } from '../sim/QuadcopterModel';
import { PID } from './PID';

export type FlightMode = 'autonomous' | 'manual';

export interface ManualInput {
  pitch: number; // -1..1 (W/S)
  roll: number; // -1..1 (A/D)
  yawRate: number; // -1..1 (Q/E)
  throttle: number; // -1..1 (R/F or Shift/Ctrl)
}

const MANUAL_TILT = (25 * Math.PI) / 180;
const MANUAL_YAW_RATE = 1.6; // rad/s at full stick
const MANUAL_CLIMB = 2.5; // m/s at full throttle stick

export class FlightController {
  mode: FlightMode = 'autonomous';
  readonly navigator = new WaypointNavigator();
  private attitude = new AttitudeController();
  private position = new PositionController();
  private manualAltPID = new PID({ kp: 4.0, ki: 0.6, kd: 2.4 }, 6, 2);
  private manualHoldZ = 1;
  private manualHadInput = false;

  update(model: QuadcopterModel, input: ManualInput, dt: number): void {
    const s = model.state;
    const meas = {
      roll: s.euler.roll,
      pitch: s.euler.pitch,
      rollRate: s.angularVelocity.x,
      pitchRate: -s.angularVelocity.y, // nose-up positive
      yawRate: s.angularVelocity.z,
    };

    if (this.mode === 'autonomous') {
      this.navigator.update(s.position.x, s.position.y, s.position.z, dt);
      const wp = this.navigator.current;
      if (wp) {
        const demand = this.position.update(
          { x: wp.x, y: wp.y, z: wp.z },
          {
            x: s.position.x,
            y: s.position.y,
            z: s.position.z,
            vx: s.velocity.x,
            vy: s.velocity.y,
            vz: s.velocity.z,
            yaw: s.euler.yaw,
            roll: s.euler.roll,
            pitch: s.euler.pitch,
          },
          dt,
        );
        const cmds = this.attitude.update(
          { roll: demand.roll, pitch: demand.pitch, yawRate: this.yawRateTo(wp, s), thrust: demand.thrust },
          meas,
          dt,
        );
        model.setMotorCommands(cmds);
      }
    } else {
      // manual mode
      const hasMove =
        Math.abs(input.pitch) + Math.abs(input.roll) + Math.abs(input.throttle) > 0.05;
      if (hasMove) this.manualHadInput = true;
      if (!this.manualHadInput) this.manualHoldZ = Math.max(0.6, s.position.z);
      if (Math.abs(input.throttle) > 0.05) {
        this.manualHoldZ = s.position.z; // follow while climbing/descending
      }

      const vz = s.velocity.z;
      // altitude hold: PID on hold altitude when stick centered, direct climb otherwise
      let thrust: number;
      if (Math.abs(input.throttle) > 0.05) {
        const az = this.manualAltPID.update(
          vz + input.throttle * MANUAL_CLIMB,
          vz,
          dt,
        );
        thrust = (MASS_KG * (GRAVITY + az)) / Math.max(0.3, Math.cos(s.euler.roll) * Math.cos(s.euler.pitch));
      } else {
        const az = this.manualAltPID.update(this.manualHoldZ, s.position.z, dt);
        thrust = (MASS_KG * (GRAVITY + az)) / Math.max(0.3, Math.cos(s.euler.roll) * Math.cos(s.euler.pitch));
      }

      const cmds = this.attitude.update(
        {
          roll: input.roll * MANUAL_TILT,
          pitch: input.pitch * MANUAL_TILT,
          yawRate: input.yawRate * MANUAL_YAW_RATE,
          thrust,
        },
        meas,
        dt,
      );
      model.setMotorCommands(cmds);
    }
  }

  /** Gentle yaw hold: command yaw rate proportional to wrapped heading error to nearest waypoint direction is distracting; hold initial heading. */
  private yawTarget = 0;
  private yawInit = false;
  private yawRateTo(_wp: unknown, s: { euler: { yaw: number } }): number {
    if (!this.yawInit) {
      this.yawTarget = s.euler.yaw;
      this.yawInit = true;
    }
    let e = this.yawTarget - s.euler.yaw;
    while (e > Math.PI) e -= 2 * Math.PI;
    while (e < -Math.PI) e += 2 * Math.PI;
    return Math.max(-0.8, Math.min(0.8, e * 1.2));
  }

  setMode(mode: FlightMode): void {
    this.mode = mode;
    this.manualHadInput = false;
    this.attitude.reset();
    this.position.reset();
    this.manualAltPID.reset();
  }

  hoverOmega(): number {
    return HOVER_OMEGA;
  }

  reset(): void {
    this.navigator.reset();
    this.attitude.reset();
    this.position.reset();
    this.manualAltPID.reset();
    this.manualHadInput = false;
    this.manualHoldZ = 1;
    this.yawInit = false;
  }
}
