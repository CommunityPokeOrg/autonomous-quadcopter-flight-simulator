export interface Waypoint {
  x: number;
  y: number;
  z: number;
  name: string;
}

export const DEFAULT_MISSION: Waypoint[] = [
  { x: 0, y: 0, z: 3, name: 'Takeoff 3m' },
  { x: 6, y: 0, z: 4, name: 'WP1 East' },
  { x: 6, y: 6, z: 5, name: 'WP2 NE' },
  { x: -6, y: 6, z: 4, name: 'WP3 NW' },
  { x: -6, y: -6, z: 6, name: 'WP4 SW high' },
  { x: 6, y: -6, z: 3, name: 'WP5 SE' },
  { x: 0, y: 0, z: 3, name: 'WP6 Home alt' },
  { x: 0, y: 0, z: 0.05, name: 'Land' },
];

const ACCEPT_RADIUS = 0.4;
const DWELL_TIME = 1.0;

export class WaypointNavigator {
  waypoints: Waypoint[] = DEFAULT_MISSION.map((w) => ({ ...w }));
  loop = false;
  index = 0;
  private dwellTimer = 0;
  done = false;

  get current(): Waypoint | null {
    if (this.done) return this.waypoints[this.waypoints.length - 1] ?? null;
    return this.waypoints[this.index] ?? null;
  }

  get progress(): string {
    return `${Math.min(this.index + 1, this.waypoints.length)}/${this.waypoints.length}`;
  }

  visitedCount(): number {
    return this.index;
  }

  /** Advance the mission state machine. Call once per control tick. */
  update(px: number, py: number, pz: number, dt: number): void {
    if (this.done) return;
    const wp = this.waypoints[this.index];
    if (!wp) {
      this.done = true;
      return;
    }
    const d = Math.hypot(px - wp.x, py - wp.y, pz - wp.z);
    if (d < ACCEPT_RADIUS) {
      this.dwellTimer += dt;
      if (this.dwellTimer >= DWELL_TIME) {
        this.dwellTimer = 0;
        this.index++;
        if (this.index >= this.waypoints.length) {
          if (this.loop) {
            this.index = 0;
          } else {
            this.done = true;
            this.index = this.waypoints.length - 1;
          }
        }
      }
    } else {
      this.dwellTimer = 0;
    }
  }

  distanceTo(px: number, py: number, pz: number): number {
    const wp = this.current;
    if (!wp) return 0;
    return Math.hypot(px - wp.x, py - wp.y, pz - wp.z);
  }

  reset(): void {
    this.index = 0;
    this.dwellTimer = 0;
    this.done = false;
  }
}
