import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { Simulation } from '../sim/Simulation';

const TRAJ_MAX_VERTS = 2000;

export class Renderer3D {
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private drone = new THREE.Group();
  private rotorDiscs: THREE.Mesh[] = [];
  private rotorRings: THREE.LineSegments[] = [];
  private scanner: THREE.Object3D | null = null;
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

  /** Rebuild the drone mesh from the active airframe config. */
  syncDrone(): void {
    const dispose = (obj: THREE.Object3D): void => {
      for (const child of [...obj.children]) {
        obj.remove(child);
        dispose(child);
      }
      const c = obj as THREE.Mesh;
      (c.geometry as THREE.BufferGeometry | undefined)?.dispose();
      const m = c.material as THREE.Material | THREE.Material[] | undefined;
      if (Array.isArray(m)) m.forEach((x) => x.dispose());
      else m?.dispose();
    };
    for (const child of [...this.drone.children]) {
      this.drone.remove(child);
      dispose(child);
    }
    this.rotorDiscs = [];
    this.rotorRings = [];
    this.scanner = null;
    this.buildDrone();
  }

  private buildDrone(): void {
    const cfg = this.sim.model.config;
    if (cfg.visual.style === 'recon') this.buildReconFrame();
    else this.buildQuadFrame();

    // rotor rings + discs, color-coded by spin
    const model = this.sim.model;
    for (let i = 0; i < 4; i++) {
      const off = model.rotorPositions[i]!;
      const color = model.rotorSpin[i]! > 0 ? cfg.visual.rotorCCW : cfg.visual.rotorCW;
      const ringGeom = new THREE.EdgesGeometry(new THREE.CircleGeometry(cfg.rotorRadius, 24));
      const ring = new THREE.LineSegments(ringGeom, new THREE.LineBasicMaterial({ color }));
      ring.position.set(off.x, off.y, 0.02);
      this.drone.add(ring);
      this.rotorRings.push(ring);

      const disc = new THREE.Mesh(
        new THREE.CircleGeometry(cfg.rotorRadius, 24),
        new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.0, side: THREE.DoubleSide }),
      );
      disc.position.set(off.x, off.y, 0.018);
      this.drone.add(disc);
      this.rotorDiscs.push(disc);
    }

    // body axes triad
    const triad = new THREE.AxesHelper(0.35);
    this.drone.add(triad);
  }

  /** Stock racing-quad silhouette: X arms + central box. */
  private buildQuadFrame(): void {
    const cfg = this.sim.model.config;
    const arm = cfg.arm;
    const mat = new THREE.LineBasicMaterial({ color: cfg.visual.frame });
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

    // body box (visual box slightly inset from the collision half extents)
    const [hx, hy, hz] = cfg.bodyHalfExtents;
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(hx! * 1.6, hy! * 1.6, hz! * 1.6),
      new THREE.MeshBasicMaterial({ color: cfg.visual.body }),
    );
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(body.geometry),
      new THREE.LineBasicMaterial({ color: cfg.visual.accent }),
    );
    this.drone.add(body, edges);

    // nose indicator (forward +x)
    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(0.03, 0.08, 8),
      new THREE.MeshBasicMaterial({ color: 0xf2f2f2 }),
    );
    nose.rotation.z = -Math.PI / 2;
    nose.position.set(0.12, 0, 0.03);
    this.drone.add(nose);
  }

  /**
   * Recon silhouette: twin-boom H-frame with a capsule fuselage pod, a chin
   * camera gimbal, raked whip antennas, and a spinning scanner puck on a mast.
   * Rotor positions are unchanged — this is geometry only.
   */
  private buildReconFrame(): void {
    const cfg = this.sim.model.config;
    const v = cfg.visual;
    const a = cfg.arm * Math.SQRT1_2;
    const frameMat = new THREE.LineBasicMaterial({ color: v.frame });
    const accentMat = new THREE.LineBasicMaterial({ color: v.accent });
    const accentFill = new THREE.MeshBasicMaterial({ color: v.accent });
    const bodyFill = new THREE.MeshBasicMaterial({ color: v.body });

    // twin longitudinal booms + front/rear crossbars (H-frame)
    const boomGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-a, a, 0), new THREE.Vector3(a, a, 0),
      new THREE.Vector3(-a, -a, 0), new THREE.Vector3(a, -a, 0),
      new THREE.Vector3(a, -a, 0), new THREE.Vector3(a, a, 0),
      new THREE.Vector3(-a, -a, 0), new THREE.Vector3(-a, a, 0),
      // midship struts tying the fuselage pod to the booms
      new THREE.Vector3(0, -a, 0), new THREE.Vector3(0, a, 0),
    ]);
    this.drone.add(new THREE.LineSegments(boomGeom, frameMat));

    // fuselage pod (capsule axis +y by default; rotate to +x)
    const fusGeom = new THREE.CapsuleGeometry(0.045, 0.16, 4, 10);
    const fuselage = new THREE.Mesh(fusGeom, bodyFill);
    fuselage.rotation.z = Math.PI / 2;
    const fusWire = new THREE.LineSegments(
      new THREE.WireframeGeometry(fusGeom),
      new THREE.LineBasicMaterial({ color: v.frame, transparent: true, opacity: 0.45 }),
    );
    fusWire.rotation.z = Math.PI / 2;
    this.drone.add(fuselage, fusWire);

    // chin gimbal: mount strut, ball housing, forward-pointing lens barrel
    const strut = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.02, 0.05), bodyFill);
    strut.position.set(0.09, 0, -0.045);
    const ballGeom = new THREE.SphereGeometry(0.038, 12, 10);
    const ball = new THREE.Mesh(ballGeom, bodyFill);
    ball.position.set(0.1, 0, -0.078);
    const ballWire = new THREE.LineSegments(new THREE.WireframeGeometry(ballGeom), accentMat);
    ballWire.position.copy(ball.position);
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.02, 0.035, 10), accentFill);
    lens.rotation.z = -Math.PI / 2; // cylinder +y → +x
    lens.position.set(0.135, 0, -0.078);
    this.drone.add(strut, ball, ballWire, lens);

    // raked whip antennas off the tail + accent tips
    const antPts: THREE.Vector3[] = [];
    const tipGeom = new THREE.SphereGeometry(0.008, 6, 6);
    for (const side of [-1, 1]) {
      const base = new THREE.Vector3(-0.08, 0.025 * side, 0.04);
      const tip = new THREE.Vector3(-0.17, 0.05 * side, 0.15);
      antPts.push(base, tip);
      const tipMesh = new THREE.Mesh(tipGeom, accentFill);
      tipMesh.position.copy(tip);
      this.drone.add(tipMesh);
    }
    this.drone.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(antPts),
      frameMat,
    ));

    // mast + rotating scanner puck with a sweep bar
    const mastGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0.02, 0, 0.045), new THREE.Vector3(0.02, 0, 0.09),
    ]);
    this.drone.add(new THREE.LineSegments(mastGeom, frameMat));
    const scanner = new THREE.Group();
    scanner.position.set(0.02, 0, 0.095);
    const puck = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.014, 12), bodyFill);
    const puckWire = new THREE.LineSegments(
      new THREE.EdgesGeometry(puck.geometry),
      frameMat,
    );
    const sweep = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.008, 0.006), accentFill);
    sweep.position.z = 0.01;
    scanner.add(puck, puckWire, sweep);
    this.drone.add(scanner);
    this.scanner = scanner;

    // nose indicator (forward +x) in livery accent
    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(0.026, 0.07, 8),
      accentFill,
    );
    nose.rotation.z = -Math.PI / 2;
    nose.position.set(0.15, 0, 0.01);
    this.drone.add(nose);
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
    const omegaMax = this.sim.model.config.omegaMax;
    for (let i = 0; i < 4; i++) {
      const w = st.motorOmegas[i] ?? 0;
      const disc = this.rotorDiscs[i]!;
      const ring = this.rotorRings[i]!;
      ring.rotation.z += w * dt * this.sim.model.rotorSpin[i]!;
      (disc.material as THREE.MeshBasicMaterial).opacity = Math.min(
        0.35,
        (w / omegaMax) * 0.45,
      );
    }

    // spin the recon scanner puck, when the airframe has one
    if (this.scanner) this.scanner.rotation.z += dt * 2.4;

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
