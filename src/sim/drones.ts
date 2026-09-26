import type { PIDGains } from '../control/PID';
import type { Waypoint } from '../control/WaypointNavigator';
import { DEFAULT_MISSION } from '../control/WaypointNavigator';

/** PID gains plus output and integrator clamps for one loop. */
export interface PIDSpec {
  gains: PIDGains;
  /** symmetric output clamp */
  out: number;
  /** integrator clamp */
  int: number;
}

/**
 * Purely cosmetic appearance for an airframe: which silhouette the renderer
 * builds plus its livery colors (hex RGB). No effect on physics.
 */
export interface DroneVisual {
  /** frame builder to use in the scene */
  style: 'quad' | 'recon';
  /** arm/boom wire color */
  frame: number;
  /** body shell fill color */
  body: number;
  /** edges, nose, and detail accent color */
  accent: number;
  /** rotor ring tint for CCW rotors */
  rotorCCW: number;
  /** rotor ring tint for CW rotors */
  rotorCW: number;
}

/**
 * Complete description of a flyable airframe: rigid-body physics,
 * cascaded-controller tuning, and the mission it flies in autonomous mode.
 */
export interface DroneConfig {
  id: string;
  label: string;
  /** one-line role summary shown in telemetry */
  role: string;
  /** total mass, kg */
  mass: number;
  /** rotor distance from center, m (X layout, rotors on the diagonals) */
  arm: number;
  /** thrust coefficient: T = kT·ω² per rotor, N */
  kT: number;
  /** yaw reaction coefficient: Q = kQ·ω², N·m */
  kQ: number;
  /** motor saturation, rad/s */
  omegaMax: number;
  /** first-order rotor lag time constant, s */
  tauMotor: number;
  /** linear drag, N per (m/s) */
  linDrag: number;
  /** angular drag, N·m per (rad/s) */
  angDrag: number;
  /** collision box half extents, m */
  bodyHalfExtents: [number, number, number];
  /** prop disc radius for rendering, m */
  rotorRadius: number;
  /** rendering-only appearance */
  visual: DroneVisual;
  control: {
    /** roll & pitch angle loops (torque out, N·m) */
    attitude: PIDSpec;
    /** yaw rate loop (torque out, N·m) */
    yawRate: PIDSpec;
    /** autonomous yaw tracking: bearing-error gain -> yaw-rate setpoint (rad/s) */
    yawTrack: { kp: number; maxRate: number };
    /** horizontal position loops (accel out, m/s²) */
    posXY: PIDSpec;
    /** vertical position loop (accel out, m/s²) */
    posZ: PIDSpec;
    /** manual-mode altitude hold / climb-rate loop */
    manualAlt: PIDSpec;
    /** autonomous tilt clamp, degrees */
    maxTiltDeg: number;
    /** manual stick tilt at full deflection, degrees */
    manualTiltDeg: number;
    /** yaw rate at full stick, rad/s */
    manualYawRate: number;
    /** climb rate at full throttle stick, m/s */
    manualClimb: number;
  };
  mission: Waypoint[];
}

/** The original airframe: 1.2 kg X-quad, 0.25 m arms. */
export const QUAD_X250: DroneConfig = {
  id: 'quad-x250',
  label: 'QUAD X250',
  role: 'Baseline multirotor',
  mass: 1.2,
  arm: 0.25,
  kT: 8.0e-6,
  kQ: 1.4e-7,
  omegaMax: 1100,
  tauMotor: 0.02,
  linDrag: 0.25,
  angDrag: 0.012,
  bodyHalfExtents: [0.09, 0.09, 0.03],
  rotorRadius: 0.11,
  visual: {
    style: 'quad',
    frame: 0xe8e8e8,
    body: 0x141414,
    accent: 0xd0d0d0,
    rotorCCW: 0xe0e0e0,
    rotorCW: 0x6e6e6e,
  },
  control: {
    attitude: { gains: { kp: 5.5, ki: 0.8, kd: 1.6 }, out: 6, int: 2 },
    yawRate: { gains: { kp: 0.7, ki: 0.15, kd: 0.05 }, out: 1.5, int: 0.8 },
    yawTrack: { kp: 1.4, maxRate: 1.0 },
    posXY: { gains: { kp: 1.6, ki: 0.25, kd: 1.9 }, out: 8, int: 2 },
    posZ: { gains: { kp: 4.0, ki: 0.6, kd: 2.6 }, out: 8, int: 2 },
    manualAlt: { gains: { kp: 4.0, ki: 0.6, kd: 2.4 }, out: 6, int: 2 },
    maxTiltDeg: 30,
    manualTiltDeg: 25,
    manualYawRate: 1.6,
    manualClimb: 2.5,
  },
  mission: DEFAULT_MISSION,
};

/**
 * Reconnaissance drone: lighter airframe swinging larger, slower props.
 * High efficiency and low disc loading give a low-throttle hover and a
 * gentle, camera-friendly ride — softer tilt/yaw limits, wide-area
 * perimeter-sweep mission with a high overwatch point.
 */
export const RECON_R320: DroneConfig = {
  id: 'recon-r320',
  label: 'RECON R320',
  role: 'Recon / survey — perimeter sweep',
  mass: 0.65,
  arm: 0.32,
  kT: 1.6e-5,
  kQ: 2.4e-7,
  omegaMax: 640,
  tauMotor: 0.03,
  linDrag: 0.16,
  angDrag: 0.018,
  bodyHalfExtents: [0.075, 0.075, 0.035],
  rotorRadius: 0.14,
  // survey livery: cyan airframe, dark slate fuselage, amber sensor accents
  visual: {
    style: 'recon',
    frame: 0x3ec6d8,
    body: 0x101d24,
    accent: 0xff9a1f,
    rotorCCW: 0xcfeff5,
    rotorCW: 0x2a6b78,
  },
  control: {
    attitude: { gains: { kp: 5.0, ki: 0.7, kd: 1.5 }, out: 6, int: 2 },
    yawRate: { gains: { kp: 0.7, ki: 0.15, kd: 0.05 }, out: 1.5, int: 0.8 },
    yawTrack: { kp: 1.1, maxRate: 0.7 },
    posXY: { gains: { kp: 1.6, ki: 0.25, kd: 1.9 }, out: 8, int: 2 },
    posZ: { gains: { kp: 4.0, ki: 0.6, kd: 2.6 }, out: 8, int: 2 },
    manualAlt: { gains: { kp: 4.0, ki: 0.6, kd: 2.4 }, out: 6, int: 2 },
    maxTiltDeg: 24,
    manualTiltDeg: 20,
    manualYawRate: 1.0,
    manualClimb: 3.0,
  },
  mission: [
    { x: 0, y: 0, z: 6, name: 'Takeoff 6m' },
    { x: 9, y: 0, z: 8, name: 'Perimeter E' },
    { x: 9, y: 9, z: 8, name: 'Perimeter NE' },
    { x: 0, y: 9, z: 9, name: 'Overwatch N' },
    { x: -9, y: 9, z: 8, name: 'Perimeter NW' },
    { x: -9, y: -9, z: 8, name: 'Perimeter SW' },
    { x: 9, y: -9, z: 7, name: 'Perimeter SE' },
    { x: 0, y: 0, z: 6, name: 'Home alt' },
    { x: 0, y: 0, z: 0.05, name: 'Land' },
  ],
};

export const DRONE_PRESETS: Record<string, DroneConfig> = {
  [QUAD_X250.id]: QUAD_X250,
  [RECON_R320.id]: RECON_R320,
};

export const DEFAULT_DRONE_ID = QUAD_X250.id;
