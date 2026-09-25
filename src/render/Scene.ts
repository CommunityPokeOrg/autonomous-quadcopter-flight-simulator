import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { Simulation } from '../sim/Simulation';
import { ROTOR_POSITIONS, ROTOR_SPIN, MOTOR_OMEGA_MAX } from '../sim/QuadcopterModel';

const TRAJ_MAX_VERTS = 2000;

export class Renderer3D {
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private drone = new THREE.Group();
  private rotorDiscs: THREE.Mesh[] = [];
  private rotorRings: THREE.LineSegments[] = [];
  private trajLine: THREE.Line;
  private trajGeom: THREE.BufferGeometry;
  private waypointGroup = new THREE.Group();
  private waypointMarkers: THREE.Object3D[] = [];
  private pathLine: THREE.Line | null = null;
  private shadowRing: THREE.Mesh;
  chaseCam = false;
  private sim: Simulation;
  // Chase cam mount: fixed body-frame offset behind the drone and a fixed
  // body-frame aim point ahead of it — the camera is bolted to the quad.
  private chaseMount = new THREE.Vector3(-2.2, 0, 1.0);
  private chaseAim = new THREE.Vector3(3.5, 0, 0.4);

  constructor(canvas: HTMLCanvasElement, sim: Simulation) {
    this.sim = sim;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setSize(innerWidth, innerHeight);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x060606);
    this.scene.fog = new THREE.Fog(0x060606, 40, 120);

    this.camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.05, 400);
    this.camera.position.set(7, -9, 6);
    this.camera.up.set(0, 0, 1);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.target.set(0, 0, 1.5);

    const grid = new THREE.GridHelper(60, 60, 0x8a8a8a, 0x2a2a2a);
    grid.rotation.x = Math.PI / 2; // GridHelper is XZ by default; rotate to XY (z-up)
    (grid.material as THREE.Material).transparent = true;
    (grid.material as THREE.Material).opacity = 0.55;
    this.scene.add(grid);

    const axes = new THREE.AxesHelper(1.2);
    axes.rotation.x = 0;
    this.scene.add(axes);

    this.buildDrone();
    this.scene.add(this.drone);
    this.scene.add(this.waypointGroup);

    // trajectory line
    this.trajGeom = new THREE.BufferGeometry();
    const trajPos = new Float32Array(TRAJ_MAX_VERTS * 3);
    const trajCol = new Float32Array(TRAJ_MAX_VERTS * 3);
    this.trajGeom.setAttribute('position', new THREE.BufferAttribute(trajPos, 3));
    this.trajGeom.setAttribute('color', new THREE.BufferAttribute(trajCol, 3));
    this.trajGeom.setDrawRange(0, 0);
    this.trajLine = new THREE.Line(
      this.trajGeom,
      new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9 }),
    );
    this.trajLine.frustumCulled = false;
    this.scene.add(this.trajLine);

    // ground shadow marker
    this.shadowRing = new THREE.Mesh(
      new THREE.RingGeometry(0.22, 0.3, 32),
      new THREE.MeshBasicMaterial({ color: 0x9a9a9a, transparent: true, opacity: 0.35, side: THREE.DoubleSide }),
    );
    this.shadowRing.rotation.x = -Math.PI / 2;
    this.shadowRing.position.z = 0.012;
    this.scene.add(this.shadowRing);

    window.addEventListener('resize', () => this.onResize());
  }

  private buildDrone(): void {
    const arm = 0.25;
    const mat = new THREE.LineBasicMaterial({ color: 0xe8e8e8 });
    const pts: THREE.Vector3[] = [];
    // X arms
    const a = arm * Math.SQRT1_2;
    const corners = [
      new THREE.Vector3(a, a, 0),
      new THREE.Vector3(a, -a, 0),
      new THREE.Vector3(-a, -a, 0),
      new THREE.Vector3(-a, a, 0),
    ];
    for (const c of corners) {
      pts.push(new THREE.Vector3(0, 0, 0), c);
    }
    const armGeom = new THREE.BufferGeometry().setFromPoints(pts);
    this.drone.add(new THREE.LineSegments(armGeom, mat));

    // body box
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.14, 0.14, 0.05),
      new THREE.MeshBasicMaterial({ color: 0x141414 }),
    );
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(body.geometry),
      new THREE.LineBasicMaterial({ color: 0xd0d0d0 }),
    );
    this.drone.add(body, edges);

    // rotor rings + discs, color-coded by spin
    for (let i = 0; i < 4; i++) {
      const off = ROTOR_POSITIONS[i]!;
      const color = ROTOR_SPIN[i]! > 0 ? 0xe0e0e0 : 0x6e6e6e; // CCW light, CW dark
      const ringGeom = new THREE.EdgesGeometry(new THREE.CircleGeometry(0.11, 24));
      const ring = new THREE.LineSegments(ringGeom, new THREE.LineBasicMaterial({ color }));
      ring.position.set(off.x, off.y, 0.02);
      this.drone.add(ring);
      this.rotorRings.push(ring);

      const disc = new THREE.Mesh(
        new THREE.CircleGeometry(0.11, 24),
        new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.0, side: THREE.DoubleSide }),
      );
      disc.position.set(off.x, off.y, 0.018);
      this.drone.add(disc);
      this.rotorDiscs.push(disc);
    }

    // nose indicator (forward +x)
    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(0.03, 0.08, 8),
      new THREE.MeshBasicMaterial({ color: 0xf2f2f2 }),
    );
    nose.rotation.z = -Math.PI / 2;
    nose.position.set(0.12, 0, 0.03);
    this.drone.add(nose);

    // body axes triad
    const triad = new THREE.AxesHelper(0.35);
    this.drone.add(triad);
  }

  /** Rebuild waypoint markers + dashed path from the navigator. */
  syncWaypoints(): void {
    this.waypointGroup.clear();
    this.waypointMarkers = [];
    const wps = this.sim.controller.navigator.waypoints;
    const pathPts: THREE.Vector3[] = [];
    for (const wp of wps) {
      const marker = new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.OctahedronGeometry(0.22)),
        new THREE.LineBasicMaterial({ color: 0x7a7a7a }),
      );
      marker.position.set(wp.x, wp.y, wp.z);
      this.waypointGroup.add(marker);
      this.waypointMarkers.push(marker);
      pathPts.push(new THREE.Vector3(wp.x, wp.y, wp.z));
    }
    if (pathPts.length > 1) {
      const geom = new THREE.BufferGeometry().setFromPoints(pathPts);
      this.pathLine = new THREE.Line(
        geom,
        new THREE.LineDashedMaterial({ color: 0x4a4a4a, dashSize: 0.35, gapSize: 0.2 }),
      );
      this.pathLine.computeLineDistances();
      this.waypointGroup.add(this.pathLine);
    }
  }

  private updateWaypointVisuals(t: number): void {
    const nav = this.sim.controller.navigator;
    const active = nav.done ? -1 : nav.index;
    this.waypointMarkers.forEach((m, i) => {
      const s = m as THREE.LineSegments;
      const mat = s.material as THREE.LineBasicMaterial;
      if (i === active) {
        mat.color.setHex(0xf0f0f0);
        const pulse = 1 + 0.25 * Math.sin(t * 5);
        s.scale.setScalar(pulse);
      } else if (i < nav.index || nav.done) {
        mat.color.setHex(0x3a3a3a);
        s.scale.setScalar(0.8);
      } else {
        mat.color.setHex(0x7a7a7a);
        s.scale.setScalar(1);
      }
    });
  }

  private updateTrajectory(): void {
    const traj = this.sim.trajectory;
    const pos = this.trajGeom.getAttribute('position') as THREE.BufferAttribute;
    const col = this.trajGeom.getAttribute('color') as THREE.BufferAttribute;
    const n = Math.min(traj.length, TRAJ_MAX_VERTS);
    for (let i = 0; i < n; i++) {
      const s = traj[i]!;
      pos.setXYZ(i, s.x, s.y, s.z);
      const f = n > 1 ? i / (n - 1) : 1; // older = dimmer
      const g = 0.08 + 0.62 * f;
      col.setXYZ(i, g, g, g);
    }
    pos.needsUpdate = true;
    col.needsUpdate = true;
    this.trajGeom.setDrawRange(0, n);
    this.trajGeom.computeBoundingSphere();
  }

  render(dt: number, elapsed: number): void {
    const st = this.sim.model.state;
    this.drone.position.set(st.position.x, st.position.y, st.position.z);
    this.drone.quaternion.set(st.quaternion.x, st.quaternion.y, st.quaternion.z, st.quaternion.w);

    // spin rotors proportional to omega
    for (let i = 0; i < 4; i++) {
      const w = st.motorOmegas[i] ?? 0;
      const disc = this.rotorDiscs[i]!;
      const ring = this.rotorRings[i]!;
      ring.rotation.z += w * dt * ROTOR_SPIN[i]!;
      (disc.material as THREE.MeshBasicMaterial).opacity = Math.min(
        0.35,
        (w / MOTOR_OMEGA_MAX) * 0.45,
      );
    }

    this.shadowRing.position.set(st.position.x, st.position.y, 0.012);
    const sh = this.shadowRing.material as THREE.MeshBasicMaterial;
    sh.opacity = Math.max(0.06, 0.4 - st.position.z * 0.04);

    this.updateTrajectory();
    this.updateWaypointVisuals(elapsed);

    if (this.chaseCam) {
      const mount = this.chaseMount.clone().applyQuaternion(this.drone.quaternion).add(this.drone.position);
      const aim = this.chaseAim.clone().applyQuaternion(this.drone.quaternion).add(this.drone.position);
      this.camera.up.set(0, 0, 1).applyQuaternion(this.drone.quaternion);
      this.camera.position.copy(mount);
      this.camera.lookAt(aim);
      this.controls.target.copy(this.drone.position);
    } else {
      // keep orbit target drifting gently toward drone so it stays in view
      this.camera.up.set(0, 0, 1);
      this.controls.target.lerp(
        new THREE.Vector3(st.position.x, st.position.y, Math.max(1, st.position.z)),
        0.02,
      );
      this.controls.update();
    }
    this.renderer.render(this.scene, this.camera);
  }

  setChase(on: boolean): void {
    this.chaseCam = on;
    this.controls.enabled = !on;
    if (!on) {
      this.camera.up.set(0, 0, 1);
      this.camera.position.set(7, -9, 6);
    }
  }

  private onResize(): void {
    this.camera.aspect = innerWidth / innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(innerWidth, innerHeight);
  }
}
