import type { FlightMode, ManualInput } from '../control/FlightController';

/**
 * A virtual joystick pad driven by Pointer Events. One active pointer per pad;
 * additional touches are ignored so two pads can be used simultaneously.
 */
class VirtualStick {
  x = 0; // -1..1, right positive
  y = 0; // -1..1, up positive
  private pad: HTMLElement;
  private knob: HTMLElement;
  private pointerId: number | null = null;

  constructor(
    parent: HTMLElement,
    cls: string,
    labels: { top: string; bottom: string; left: string; right: string },
  ) {
    const wrap = document.createElement('div');
    wrap.className = `stick ${cls}`;
    for (const [side, text] of Object.entries(labels)) {
      const s = document.createElement('span');
      s.className = `stick-label ${side}`;
      s.textContent = text;
      wrap.appendChild(s);
    }
    this.pad = document.createElement('div');
    this.pad.className = 'stick-pad';
    this.knob = document.createElement('div');
    this.knob.className = 'stick-knob';
    this.pad.appendChild(this.knob);
    wrap.appendChild(this.pad);
    parent.appendChild(wrap);

    this.pad.addEventListener('pointerdown', (e) => this.onDown(e));
    this.pad.addEventListener('pointermove', (e) => this.onMove(e));
    this.pad.addEventListener('pointerup', (e) => this.onUp(e));
    this.pad.addEventListener('pointercancel', (e) => this.onUp(e));
    this.pad.addEventListener('lostpointercapture', () => this.reset());
    this.pad.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  private onDown(e: PointerEvent): void {
    if (this.pointerId !== null) return;
    e.preventDefault();
    this.pointerId = e.pointerId;
    this.pad.setPointerCapture(e.pointerId);
    this.track(e);
  }

  private onMove(e: PointerEvent): void {
    if (e.pointerId !== this.pointerId) return;
    e.preventDefault();
    this.track(e);
  }

  private onUp(e: PointerEvent): void {
    if (e.pointerId !== this.pointerId) return;
    e.preventDefault();
    this.reset();
  }

  private track(e: PointerEvent): void {
    const r = this.pad.getBoundingClientRect();
    const max = r.width / 2;
    let dx = (e.clientX - (r.left + max)) / max;
    let dy = (e.clientY - (r.top + r.height / 2)) / max;
    const mag = Math.hypot(dx, dy);
    if (mag > 1) {
      dx /= mag;
      dy /= mag;
    }
    this.x = dx;
    this.y = -dy; // screen-down positive → stick-up positive
    const travel = max - this.knob.offsetWidth / 2;
    this.knob.style.transform = `translate(${dx * travel}px, ${dy * travel}px)`;
  }

  reset(): void {
    this.pointerId = null;
    this.x = 0;
    this.y = 0;
    this.knob.style.transform = '';
  }
}

/**
 * On-screen twin-stick controls for manual flight on touch devices.
 * Left stick: yaw (x) + throttle/climb (y). Right stick: pitch (y) + roll (x).
 * Visible only in manual mode on touch-capable devices.
 */
export class TouchControls {
  private el: HTMLElement;
  private left: VirtualStick;
  private right: VirtualStick;
  private visible = false;

  constructor(parent: HTMLElement) {
    this.el = document.createElement('div');
    this.el.className = 'touch-controls';
    this.el.hidden = true;
    this.left = new VirtualStick(this.el, 'stick-left', {
      top: 'THR +',
      bottom: 'THR −',
      left: 'YAW −',
      right: 'YAW +',
    });
    this.right = new VirtualStick(this.el, 'stick-right', {
      top: 'PITCH +',
      bottom: 'PITCH −',
      left: 'ROLL −',
      right: 'ROLL +',
    });
    parent.appendChild(this.el);

    const resetAll = () => this.reset();
    window.addEventListener('blur', resetAll);
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) resetAll();
    });
  }

  static isTouchDevice(): boolean {
    return navigator.maxTouchPoints > 0 || 'ontouchstart' in window;
  }

  /** Current analog input from both sticks, each axis -1..1. */
  get input(): ManualInput {
    return {
      pitch: this.right.y,
      roll: this.right.x,
      yawRate: this.left.x,
      throttle: this.left.y,
    };
  }

  /** Show sticks only while in manual mode; resets when hidden. */
  update(mode: FlightMode): void {
    const show = mode === 'manual';
    if (show !== this.visible) {
      this.visible = show;
      this.el.hidden = !show;
      if (!show) this.reset();
    }
  }

  reset(): void {
    this.left.reset();
    this.right.reset();
  }
}
