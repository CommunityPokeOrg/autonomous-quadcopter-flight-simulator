// Shared physical constants and angle helpers. Per-airframe values live in sim/drones.ts.
export const GRAVITY = 9.81;

/** Normalize an angle to [-π, π) so errors always take the short way around. */
export function wrapAngle(a: number): number {
  return a - 2 * Math.PI * Math.round(a / (2 * Math.PI));
}
