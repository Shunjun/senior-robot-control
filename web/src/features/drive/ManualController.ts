import { IntentBackpressureError, type Authority, type Intent, type VehiclePort } from '../../api/vehicleState';
export type ManualPhase = 'none' | 'preparing' | 'prepared' | 'active';
/** A prepared receipt is permission, never a queued press. One active latest slot. */
export class ManualController {
  phase: ManualPhase = 'none';
  private authority: Authority | null = null;
  private generation = 0;
  private gesture = 0;
  private sequence = 0;
  private busy = false;
  private current: Intent | null = null;
  private latest: Intent | null = null;
  private timer: ReturnType<typeof setInterval> | null = null;
  private unsubscribe: (() => void) | null = null;
  constructor(private readonly port: VehiclePort, private readonly changed: (phase: ManualPhase, error?: string) => void) {
  }
  attach(): void {
    this.unsubscribe ??= this.port.onManualFenced(() => this.release());
  }
  dispose(): void {
    this.unsubscribe?.();
    this.unsubscribe = null;
    this.release();
  }
  private publish(phase: ManualPhase, error?: string): void {
    this.phase = phase;
    this.changed(phase, error);
  }
  async prepare(): Promise<void> {
    this.attach();
    this.release();
    const generation = this.generation;
    this.publish('preparing');
    try {
      const receipt = await this.port.acquire();
      if (generation !== this.generation) {
        void this.port.release(receipt.authority_id).catch(() => {
        });
        return;
      }
      this.authority = receipt;
      this.publish('prepared');
    } catch (error) {
      if (generation === this.generation) this.publish('none', String(error));
    }
  }
  begin(linear: number, turn: number, gear: Intent['gear']): boolean {
    if (this.phase !== 'prepared' || this.authority === null) return false;
    this.gesture++;
    this.publish('active');
    this.timer = setInterval(() => {
      if (this.current) {
        this.latest = { ...this.current, sequence: ++this.sequence };
        this.flush();
      }
    }, 100);
    this.update(linear, turn, gear);
    return true;
  }
  update(linear: number, turn: number, gear: Intent['gear']): void {
    if (this.phase !== 'active' || this.authority === null) return;
    if (![linear, turn].every(v => Number.isFinite(v) && Math.abs(v) <= 1)) {
      this.release();
      return;
    }
    this.current = {
      authority_id: this.authority.authority_id,
      gesture: this.gesture,
      sequence: ++this.sequence,
      linear,
      turn,
      gear
    };
    this.latest = this.current;
    this.flush();
  }
  private flush(): void {
    if (this.busy || this.phase !== 'active' || !this.latest) return;
    const generation = this.generation, intent = this.latest;
    this.latest = null;
    this.busy = true;
    let backpressured = false;
    void this.port.intent(intent).catch(error => {
      if (generation !== this.generation) return;
      if (error instanceof IntentBackpressureError) {
        // Discard queued samples. The held-input heartbeat generates a fresh
        // sample next tick; release/epoch retirement clears current as well.
        backpressured = true;
        this.latest = null;
        return;
      }
      this.release();
      this.publish('none', `驾驶意图未确认：${String(error)}`);
    }).finally(() => {
      this.busy = false;
      if (!backpressured && generation === this.generation) this.flush();
    });
  }
  observeClosed(authorityId: string): void {
    if (authorityId === this.authority?.authority_id) this.release();
  }
  release(): void {
    const generation = ++this.generation;
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
    this.current = this.latest = null;
    const authority = this.authority;
    this.authority = null;
    this.publish('none');
    if (authority) void this.port.release(authority.authority_id).catch(error => {
      if (this.generation === generation && this.phase === 'none') this.changed('none', `释放结果待确认：${String(error)}`);
    });
  }
}
