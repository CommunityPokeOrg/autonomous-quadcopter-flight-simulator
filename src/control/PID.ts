export interface PIDGains {
  kp: number;
  ki: number;
  kd: number;
}

/** Generic PID with integrator clamp, derivative-on-measurement and output limits. */
export class PID {
  private integral = 0;
  private prevMeasurement = 0;
  private hasPrev = false;

  constructor(
    private gains: PIDGains,
    private outputLimit = Infinity,
    private integralLimit = Infinity,
  ) {}

  update(setpoint: number, measurement: number, dt: number): number {
    const error = setpoint - measurement;
    this.integral += error * dt;
    if (this.integral > this.integralLimit) this.integral = this.integralLimit;
    if (this.integral < -this.integralLimit) this.integral = -this.integralLimit;

    let derivative = 0;
    if (this.hasPrev && dt > 0) {
      derivative = (measurement - this.prevMeasurement) / dt;
    }
    this.prevMeasurement = measurement;
    this.hasPrev = true;

    let out =
      this.gains.kp * error + this.gains.ki * this.integral - this.gains.kd * derivative;
    if (out > this.outputLimit) out = this.outputLimit;
    if (out < -this.outputLimit) out = -this.outputLimit;
    return out;
  }

  reset(): void {
    this.integral = 0;
    this.prevMeasurement = 0;
    this.hasPrev = false;
  }

  setGains(g: Partial<PIDGains>): void {
    this.gains = { ...this.gains, ...g };
  }
}
