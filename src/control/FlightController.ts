import { AttitudeController } from './AttitudeController';
import { PositionController } from './PositionController';
import { WaypointNavigator, type Waypoint } from './WaypointNavigator';
import type { QuadcopterModel } from '../sim/QuadcopterModel';
import type { DroneConfig } from '../sim/drones';
import { DRONE_PRESETS, DEFAULT_DRONE_ID } from '../sim/drones';
import { GRAVITY, wrapAngle } from './constants';
import { PID } from './PID';

export type FlightMode = 'autonomous' | 'manual';

export interface ManualInput {
  pitch: number; // -1..1 (W/S)
  roll: number; // -1..1 (A/D)
  yawRate: number; // -1..1 (Q/E)
  throttle: number; // -1..1 (R/F or Shift/Ctrl)
}

export class FlightController {
  readonly cfg: DroneConfig;
  mode: FlightMode = 'autonomous';
  readonly navigator: WaypointNavigator;
  private attitude: AttitudeController;
  private position: PositionController;
  private manualAltPID: PID;
  private manualTilt: number;
  private manualHoldZ = 1;
  private manualHadInput = false;

  constructor(cfg: DroneConfig = DRONE_PRESETS[DEFAULT_DRONE_ID]!) {
    this.cfg = cfg;
    this.navigator = new WaypointNavigator(cfg.mission);
    this.attitude = new AttitudeController(cfg);
    this.position = new PositionController(cfg);
    const ma = cfg.control.manualAlt;
    this.manualAltPID = new PID(ma.gains, ma.out, ma.int);
    this.manualTilt = (cfg.control.manualTiltDeg * Math.PI) / 180;
  }

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
      const landed = s.position.z < 0.1 && s.velocity.length() < 0.5;
      if ((this.navigator.done || !wp) && landed) {
        // mission complete: motors off, controllers idle until reset
        model.setMotorCommands([0, 0, 0, 0]);
        this.attitude.reset();
        this.position.reset();
        return;
      }
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
          {
            roll: demand.roll,
            pitch: demand.pitch,
            yawRate: this.yawRateTo(wp, s),
            thrust: demand.thrust,
          },
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
          vz + input.throttle * this.cfg.control.manualClimb,
          vz,
          dt,
        );
        thrust = (this.cfg.mass * (GRAVITY + az)) / Math.max(0.3, Math.cos(s.euler.roll) * Math.cos(s.euler.pitch));
      } else {
        const az = this.manualAltPID.update(this.manualHoldZ, s.position.z, dt);
        thrust = (this.cfg.mass * (GRAVITY + az)) / Math.max(0.3, Math.cos(s.euler.roll) * Math.cos(s.euler.pitch));
      }

      const cmds = this.attitude.update(
        {
          roll: input.roll * this.manualTilt,
          pitch: input.pitch * this.manualTilt,
          yawRate: input.yawRate * this.cfg.control.manualYawRate,
          thrust,
        },
        meas,
        dt,
      );
      model.setMotorCommands(cmds);
    }
  }

  /** Yaw target the tracker is currently steering to (rad). */
  private yawTarget = 0;
  private yawInit = false;
  /** Waypoint index whose heading is latched for the terminal approach (-1 = tracking). */
  private yawHoldIdx = -1;

  /**
   * Point the nose at the active waypoint: yaw-rate setpoint proportional to
   * the wrapped bearing error, clamped to the airframe's tracking rate.
   *
   * Within ~1 m of the waypoint the bearing is hypersensitive to position
   * jitter (the drone can sit almost directly over the target), so the
   * approach heading is latched for the rest of that leg — tracking resumes
   * on the next waypoint, or if the drone drifts back out beyond 2 m.
   * This keeps takeoff/dwell/landing headings stable too.
   */
  private yawRateTo(
    wp: Waypoint,
    s: { position: { x: number; y: number }; euler: { yaw: number } },
  ): number {
    if (!this.yawInit) {
      this.yawTarget = s.euler.yaw;
      this.yawInit = true;
    }
    const dxy = Math.hypot(wp.x - s.position.x, wp.y - s.position.y);
    const idx = this.navigator.index;
    if (this.yawHoldIdx !== idx) {
      if (dxy < 1.0) {
        this.yawHoldIdx = idx;
      } else {
        this.yawTarget = Math.atan2(wp.y - s.position.y, wp.x - s.position.x);
      }
    } else if (dxy > 2.0) {
      this.yawHoldIdx = -1; // drifted out of the terminal cone — resume tracking
    }
    const e = wrapAngle(this.yawTarget - s.euler.yaw);
    const { kp, maxRate } = this.cfg.control.yawTrack;
    return Math.max(-maxRate, Math.min(maxRate, e * kp));
  }

  setMode(mode: FlightMode): void {
    this.mode = mode;
    this.manualHadInput = false;
    this.attitude.reset();
    this.position.reset();
    this.manualAltPID.reset();
    this.yawInit = false;
    this.yawHoldIdx = -1;
  }

  hoverOmega(): number {
    return Math.sqrt((this.cfg.mass * GRAVITY) / (4 * this.cfg.kT));
  }

  reset(): void {
    this.navigator.reset();
    this.attitude.reset();
    this.position.reset();
    this.manualAltPID.reset();
    this.manualHadInput = false;
    this.manualHoldZ = 1;
    this.yawInit = false;
    this.yawHoldIdx = -1;
  }
}
