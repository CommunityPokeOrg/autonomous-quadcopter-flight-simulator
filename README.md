# Autonomous Quadcopter Flight Simulator

A browser-based 6-DOF quadcopter simulator with real physics, PID flight control,
and autonomous waypoint navigation — built with Three.js and cannon-es.

**Live demo:** https://communitypokeorg.github.io/autonomous-quadcopter-flight-simulator/

## Features

- **6-DOF rigid-body physics** via cannon-es: per-rotor thrust (`T = kT·ω²`),
  reaction torques, first-order motor lag, linear/angular drag, and ground contact.
- **Cascaded PID flight controller**: position loop → attitude loop → X-config
  motor mixer, with motor saturation and tilt compensation.
- **Autonomous missions**: waypoint navigation with acceptance radius and dwell
  time; default mission takes off, flies a 6-waypoint 3-D circuit, returns home
  and lands. Optional mission looping.
- **Manual mode**: keyboard flying with attitude stabilization and altitude hold.
- **Live HUD**: telemetry, per-motor thrust bars, artificial horizon, heading
  readout, start/pause/reset, camera and speed controls.
- **3-D visualization**: wireframe quad with color-coded CW/CCW rotors, spin-blur
  discs, trajectory trail, waypoint markers, dashed mission path, ground shadow,
  orbit and chase cameras.

## Controls

| Mode | Input |
|---|---|
| Manual | `W`/`S` pitch, `A`/`D` roll, `Q`/`E` yaw, `R`/`F` (or `Shift`/`Ctrl`) throttle |
| Camera | Orbit: drag to rotate, scroll to zoom. Toggle Orbit/Chase in the panel. |
| Sim | Start / Pause / Reset buttons; speed slider 0.25×–2×; Loop mission checkbox. |

## Run locally

```bash
npm install
npm run dev        # dev server
npm run typecheck  # tsc --noEmit
npm run build      # typecheck + production build to dist/
npm run preview    # serve the built app at http://localhost:4173
```

## Deployment

Pushes to `main` trigger `.github/workflows/deploy.yml`, which builds `dist/` and
deploys it to GitHub Pages. The Vite `base` is set to
`/autonomous-quadcopter-flight-simulator/`, matching the project-site path.

## Architecture

```
src/
├── main.ts                       bootstrap + rAF loop, fixed-timestep physics (1/240 s)
├── sim/
│   ├── QuadcopterModel.ts        cannon-es body, 4 rotors, thrust/drag model
│   └── Simulation.ts             world, ground plane, stepping, trajectory buffer
├── control/
│   ├── PID.ts                    generic PID (integrator clamp, derivative on meas.)
│   ├── AttitudeController.ts     angle-mode PID + X-config motor mixer
│   ├── PositionController.ts     position PID → tilt demand + collective thrust
│   ├── WaypointNavigator.ts      mission sequencing, dwell, loop
│   ├── FlightController.ts       autonomous/manual modes, altitude hold
│   └── constants.ts              shared physical constants
├── render/
│   └── Scene.ts                  Three.js scene, drone mesh, trails, cameras
└── ui/
    └── HUD.ts                    telemetry/control DOM overlay + attitude indicator
```

### Control loop

```
waypoints → WaypointNavigator → PositionController (PID)
   → desired roll/pitch + total thrust
   → AttitudeController (PID) → mixer → per-motor ω commands
   → QuadcopterModel (rotor lag, saturation, kT·ω² forces)
   → cannon-es rigid body → world step
```

### Physics model

- Mass 1.2 kg, arm length 0.25 m, box inertia.
- Rotor thrust `kT·ω²` (kT = 8e-6), yaw reaction `kQ·ω²` (kQ = 1.4e-7),
  alternating CW/CCW spin directions, motor lag τ ≈ 20 ms, ω ∈ [0, 1100] rad/s.
- Gravity −9.81 m/s² on +z-up world; linear drag 0.25 N/(m/s), angular drag 0.012.
- Fixed timestep 1/240 s with an accumulator; frame dt clamped to 0.1 s so
  background tabs can't explode the sim.

## Tuning notes

- Attitude angle PIDs: kp≈5.5, ki≈0.8, kd≈1.6 (torque-limited to ±6 N·m).
- Position PIDs: horizontal kp≈1.6/kd≈1.9, vertical kp≈4.0/kd≈2.6 (accel out).
- Tilt is clamped to 30° and thrust is tilt-compensated:
  `T = m(g + a_z)/(cos φ cos θ)`.
- Waypoint acceptance radius 0.4 m, dwell 1 s.
- `scripts/sim-check.ts` (`npx tsx scripts/sim-check.ts`) runs the full mission
  headlessly and verifies the drone reaches every waypoint and lands.
