import type { Simulation } from '../sim/Simulation';
import { MOTOR_OMEGA_MAX } from '../sim/QuadcopterModel';
import type { FlightMode } from '../control/FlightController';

export interface HUDCallbacks {
  onStartPause(): void;
  onReset(): void;
  onModeChange(mode: FlightMode): void;
  onCameraChange(chase: boolean): void;
  onSpeedChange(speed: number): void;
  onLoopChange(loop: boolean): void;
}

const RAD2DEG = 180 / Math.PI;

export class HUD {
  private root: HTMLElement;
  private sim: Simulation;
  private cb: HUDCallbacks;
  private els: Record<string, HTMLElement> = {};
  private horizon!: HTMLCanvasElement;
  private startBtn!: HTMLButtonElement;
  private motorBars: HTMLElement[] = [];

  constructor(root: HTMLElement, sim: Simulation, cb: HUDCallbacks) {
    this.root = root;
    this.sim = sim;
    this.cb = cb;
    this.build();
  }

  private el(tag: string, cls: string, parent: HTMLElement, text = ''): HTMLElement {
    const e = document.createElement(tag);
    e.className = cls;
    if (text) e.textContent = text;
    parent.appendChild(e);
    return e;
  }

  private build(): void {
    // telemetry panel (top-left)
    const tele = this.el('div', 'panel telemetry', this.root);
    this.el('div', 'panel-title', tele, 'TELEMETRY');
    const rows: [string, string][] = [
      ['time', 'TIME'],
      ['pos', 'POS'],
      ['vel', 'VEL'],
      ['alt', 'ALT'],
      ['rpy', 'R/P/Y'],
      ['rates', 'RATES'],
      ['thrust', 'THRUST'],
      ['mode', 'MODE'],
      ['wp', 'WAYPOINT'],
      ['dist', 'DIST'],
      ['state', 'STATE'],
    ];
    for (const [key, label] of rows) {
      const row = this.el('div', 'row', tele);
      this.el('span', 'label', row, label);
      this.els[key] = this.el('span', 'value', row, '—');
    }
    const motors = this.el('div', 'motors', tele);
    for (let i = 0; i < 4; i++) {
      const wrap = this.el('div', 'motor', motors);
      this.el('span', 'motor-label', wrap, `M${i + 1}`);
      const bar = this.el('div', 'bar', wrap);
      const fill = this.el('div', 'bar-fill', bar);
      this.motorBars.push(fill);
    }

    // control panel (top-right)
    const ctrl = this.el('div', 'panel controls', this.root);
    this.el('div', 'panel-title', ctrl, 'CONTROL');
    const btnRow = this.el('div', 'btn-row', ctrl);
    this.startBtn = this.el('button', 'btn primary', btnRow, 'Start') as HTMLButtonElement;
    this.startBtn.onclick = () => this.cb.onStartPause();
    const resetBtn = this.el('button', 'btn', btnRow, 'Reset') as HTMLButtonElement;
    resetBtn.onclick = () => this.cb.onReset();

    const modeRow = this.el('div', 'btn-row', ctrl);
    this.el('span', 'label', modeRow, 'MODE');
    const modeSel = this.el('select', 'sel', modeRow) as HTMLSelectElement;
    for (const m of ['autonomous', 'manual']) {
      const o = document.createElement('option');
      o.value = m;
      o.textContent = m;
      modeSel.appendChild(o);
    }
    modeSel.onchange = () => this.cb.onModeChange(modeSel.value as FlightMode);

    const camRow = this.el('div', 'btn-row', ctrl);
    this.el('span', 'label', camRow, 'CAMERA');
    const camSel = this.el('select', 'sel', camRow) as HTMLSelectElement;
    for (const c of ['orbit', 'chase']) {
      const o = document.createElement('option');
      o.value = c;
      o.textContent = c;
      camSel.appendChild(o);
    }
    camSel.onchange = () => this.cb.onCameraChange(camSel.value === 'chase');

    const spdRow = this.el('div', 'btn-row', ctrl);
    this.el('span', 'label', spdRow, 'SPEED');
    const spd = this.el('input', 'slider', spdRow) as HTMLInputElement;
    spd.type = 'range';
    spd.min = '0.25';
    spd.max = '2';
    spd.step = '0.25';
    spd.value = '1';
    this.els['speedVal'] = this.el('span', 'value', spdRow, '1.00×');
    spd.oninput = () => {
      this.cb.onSpeedChange(parseFloat(spd.value));
      this.els['speedVal']!.textContent = `${parseFloat(spd.value).toFixed(2)}×`;
    };

    const loopRow = this.el('div', 'btn-row', ctrl);
    const loopLbl = this.el('label', 'chk', loopRow);
    const loop = document.createElement('input');
    loop.type = 'checkbox';
    loop.onchange = () => this.cb.onLoopChange(loop.checked);
    loopLbl.appendChild(loop);
    loopLbl.appendChild(document.createTextNode(' Loop mission'));

    const help = this.el('div', 'help', ctrl);
    help.innerHTML =
      '<div class="panel-title">MANUAL KEYS</div>' +
      'W/S pitch · A/D roll<br/>Q/E yaw · R/F throttle';

    // bottom instruments
    const bottom = this.el('div', 'panel instruments', this.root);
    this.horizon = this.el('canvas', 'horizon', bottom) as HTMLCanvasElement;
    this.horizon.width = 120;
    this.horizon.height = 120;
    const hdg = this.el('div', 'heading', bottom);
    this.els['hdg'] = this.el('span', 'value big', hdg, '000°');
  }

  update(): void {
    const s = this.sim.model.state;
    const nav = this.sim.controller.navigator;
    const e = this.els;
    e['time']!.textContent = `${this.sim.time.toFixed(1)} s`;
    e['pos']!.textContent =
      `${s.position.x.toFixed(2)} ${s.position.y.toFixed(2)} ${s.position.z.toFixed(2)}`;
    const spd = s.velocity.length();
    e['vel']!.textContent =
      `${s.velocity.x.toFixed(1)} ${s.velocity.y.toFixed(1)} ${s.velocity.z.toFixed(1)} | ${spd.toFixed(1)}`;
    e['alt']!.textContent = `${s.position.z.toFixed(2)} m`;
    e['rpy']!.textContent =
      `${(s.euler.roll * RAD2DEG).toFixed(0)}° ${(s.euler.pitch * RAD2DEG).toFixed(0)}° ${(s.euler.yaw * RAD2DEG).toFixed(0)}°`;
    e['rates']!.textContent =
      `${s.angularVelocity.x.toFixed(2)} ${(-s.angularVelocity.y).toFixed(2)} ${s.angularVelocity.z.toFixed(2)}`;
    e['thrust']!.textContent = `${s.totalThrust.toFixed(1)} N`;
    e['mode']!.textContent = this.sim.mode.toUpperCase();
    const wp = nav.current;
    e['wp']!.textContent = nav.done
      ? 'MISSION COMPLETE'
      : wp ? `${nav.progress} ${wp.name}` : '—';
    e['dist']!.textContent = `${nav.distanceTo(s.position.x, s.position.y, s.position.z).toFixed(1)} m`;
    e['state']!.textContent = this.sim.running
      ? this.sim.landed ? 'LANDED' : 'FLYING'
      : 'PAUSED';
    this.startBtn.textContent = this.sim.running ? 'Pause' : 'Start';

    for (let i = 0; i < 4; i++) {
      const w = s.motorOmegas[i] ?? 0;
      const f = this.motorBars[i]!;
      f.style.width = `${Math.min(100, (w / MOTOR_OMEGA_MAX) * 100).toFixed(0)}%`;
    }

    // round first, then wrap so 359.6° displays as 0° rather than "360°"
    const hdgDeg = ((Math.round(s.euler.yaw * RAD2DEG) % 360) + 360) % 360;
    e['hdg']!.textContent = `${String(hdgDeg).padStart(3, '0')}°`;

    this.drawHorizon(s.euler.roll, s.euler.pitch);
  }

  private drawHorizon(roll: number, pitch: number): void {
    const c = this.horizon;
    const ctx = c.getContext('2d')!;
    const w = c.width;
    const h = c.height;
    const cx = w / 2;
    const cy = h / 2;
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, w / 2 - 2, 0, Math.PI * 2);
    ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate(-roll);
    const pitchPx = (pitch * RAD2DEG) * 1.6;
    ctx.translate(0, pitchPx);
    ctx.fillStyle = '#10364a';
    ctx.fillRect(-w, -h * 2, w * 2, h * 2); // sky
    ctx.fillStyle = '#241a10';
    ctx.fillRect(-w, 0, w * 2, h * 2); // ground
    ctx.strokeStyle = '#1adfd8';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-w, 0);
    ctx.lineTo(w, 0);
    ctx.stroke();
    ctx.restore();
    // fixed aircraft symbol
    ctx.strokeStyle = '#ffe14d';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx - 18, cy);
    ctx.lineTo(cx - 6, cy);
    ctx.lineTo(cx, cy + 5);
    ctx.lineTo(cx + 6, cy);
    ctx.lineTo(cx + 18, cy);
    ctx.stroke();
  }
}
