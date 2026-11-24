/** Versioned vehicle UI contract. New routes require an explicit car capability. */
export type Mode = 'unknown' | 'idle' | 'manual' | 'task' | 'braking' | 'obstacle_hold' | 'feedback_stop' | 'execution_fault' | 'estop';
export type Capability = 'runtime.v1' | 'motion.intent.v1' | 'settings.v1' | 'task.execution.v1' | 'logs.v1' | 'learning.v1' | 'task.names.v1';
export type Action = 'acquire' | 'estop_reset' | 'fault_recover';
export type TaskAction = 'pause' | 'resume' | 'cancel' | 'takeover';
export type Level = 'DEBUG' | 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';
export interface BodyReference {
  average: {
    vx: number;
    vy: number;
    wz: number;
  };
  source: string;
  age_seconds: number;
  stale: boolean;
  kind: 'independent_body';
}
export interface TaskExecution {
  name?: string;
  definition_task_id?: string;
  task_id: string;
  version: number;
  phase: 'starting' | 'running' | 'revoking' | 'paused' | 'succeeded' | 'canceled' | 'failed';
  completed: number;
  total: number;
  stopped_for: string | null;
  allowed_actions: readonly TaskAction[];
}
export interface ManualAuthorityView {
  authority_id: string;
  state: 'prepared' | 'active' | 'closed';
}
export interface RuntimeObservation {
  status: 'current' | 'missing' | 'revoked' | 'ipc_unavailable';
  last_sampled_steady: number | null;
  age_seconds: number | null;
}
export interface VehicleFrame {
  //Compatible: previous agents omit observation metadata.
  observation?: RuntimeObservation;
  manual_authority: ManualAuthorityView | null;
  schema: 1;
  sequence: number;
  mode: Mode;
  reason: string;
  estop_revision: number;
  fault_revision: number;
  confirmed_estop_status: 'unknown' | 'clear' | 'software_latched';
  body_reference: BodyReference | null;
  wheel_linear_mps: number | null;
  task: TaskExecution | null;
  allowed_actions: readonly Action[];
  capabilities: readonly Capability[];
}
export interface VehicleView {
  manual_connected: boolean;
  available: boolean;
  received_at_ms: number | null;
  frame: VehicleFrame | null;
  connected: boolean;
  rtt_ms: number | null;
}
export interface Authority {
  authority_id: string;
  state: 'prepared';
}
export interface Intent {
  authority_id: string;
  gesture: number;
  sequence: number;
  linear: number;
  turn: number;
  gear: 'low' | 'high';
}
export interface DrivingSettings {
  low_linear_mps: number;
  high_linear_mps: number;
  low_angular_rps: number;
  high_angular_rps: number;
  reverse_linear_mps: number;
  degraded_linear_mps: number;
}
export interface LoggingSettings {
  level: Level;
  retention_seconds: number;
  debug_enabled: boolean;
  debug_restore_seconds: number;
}
export interface SettingsSnapshot {
  driving: DrivingSettings;
  logging: LoggingSettings;
  driving_version: number;
  logging_version: number;
  debug_original_level: Level | null;
  debug_deadline_utc: number | null;
}
export interface SettingsCapabilities {
  driving_defaults: DrivingSettings;
  driving_minimum: DrivingSettings;
  driving_maximum: DrivingSettings;
  logging_defaults: LoggingSettings;
  retention_min_seconds: number;
  retention_max_seconds: number;
  debug_restore_min_seconds: number;
  debug_restore_max_seconds: number;
}
export interface SettingsData {
  snapshot: SettingsSnapshot;
  capabilities: SettingsCapabilities;
  can_apply_driving: boolean;
}
export type SettingsGroup = 'driving' | 'logging';
export interface SettingsRequest {
  request_id: string;
  expected_version: number;
  group: SettingsGroup;
  values: DrivingSettings | LoggingSettings;
  restore_defaults: boolean;
}
export interface SettingsReceipt {
  request_id: string;
  group: SettingsGroup;
  reason: 'applied' | 'version_conflict' | 'stop_required' | 'invalid_values' | 'request_conflict';
  snapshot: SettingsSnapshot;
  capabilities: SettingsCapabilities;
}
export interface LogGap {
  reason: string;
  count: number | null;
  after: number | null;
  through: number | null;
  durable: boolean;
}
export interface LogEntry {
  sequence: number;
  kind: 'event' | 'gap';
  level: Level;
  module: string;
  summary: string;
  detail: string;
  fields_json: string;
  source_utc: number;
  received_utc: number;
  request_id: string | null;
  task_id: string | null;
  loss_reason: string | null;
  loss_count: number | null;
}
export interface LogPage {
  entries: readonly LogEntry[];
  gaps: readonly LogGap[];
  next_cursor: number;
  high_water: number;
  has_more: boolean;
  scanned: number;
}
export interface LogQuery {
  after: number;
  limit: number;
  levels: readonly Level[];
  module: string | null;
  request_id: string | null;
  task_id: string | null;
  since_utc: number | null;
  until_utc: number | null;
}
export interface LogDownload {
  data: string;
  next_cursor: number;
  has_more: boolean;
  gaps: readonly LogGap[];
}
/** Independent authenticated log route: never multiplex its outbox onto control. */
export interface VehicleLogPort {
  query(query: LogQuery): Promise<LogPage>;
  follow(query: LogQuery, onPage: (page: LogPage) => void, onError: (error: Error) => void): () => void;
  download(query: LogQuery): Promise<LogDownload>;
}
/** Local transport backpressure: no sample was sent and the held input may retry fresh. */
export class IntentBackpressureError extends Error {
  override name = 'IntentBackpressureError';
  constructor() {
    super('motion socket is buffered');
  }
}
export interface LearningStart {
  start_request_id: string;
  attempt_id: string;
  state: 'accepted' | 'busy';
}
export interface LearningStatus {
  run_id: string;
  active_revision: number;
  active_model: { attempt_id: string; model_id: string; point_count: number } | null;
  attempt_id: string;
  phase: string;
  reason: string;
  motion_complete: boolean;
  capture_complete: boolean;
  compute_complete: boolean;
  durable: boolean;
  can_start: boolean;
  cannot_start_reason: string | null;
}
function learningId(value: unknown): string {
  const id = text(value, 32);
  if (!/^[0-9a-f]{32}$/.test(id)) throw new VehicleDecodeError();
  return id;
}
export function decodeLearningStart(value: unknown): LearningStart {
  const r = record(value, ['start_request_id', 'attempt_id', 'state']);
  const id = text(r.start_request_id);
  if (!id) throw new VehicleDecodeError();
  return { start_request_id: id, attempt_id: learningId(r.attempt_id), state: oneOf(r.state, ['accepted', 'busy']) };
}
export function decodeLearningStatus(value: unknown): LearningStatus {
  const r = record(value, ['run_id', 'active_revision', 'active_model', 'attempt_id', 'phase', 'reason', 'motion_complete', 'capture_complete', 'compute_complete', 'durable', 'can_start', 'cannot_start_reason']);
  return {
    run_id: learningId(r.run_id), active_revision: count(r.active_revision),
    active_model: nullable(r.active_model, value => {
      const model = record(value, ['attempt_id', 'model_id', 'point_count']);
      return { attempt_id: learningId(model.attempt_id), model_id: learningId(model.model_id), point_count: count(model.point_count, 2048) };
    }),
    attempt_id: learningId(r.attempt_id), phase: text(r.phase, 64), reason: text(r.reason, 256),
    motion_complete: boolean(r.motion_complete), capture_complete: boolean(r.capture_complete),
    compute_complete: boolean(r.compute_complete), durable: boolean(r.durable),
    can_start: boolean(r.can_start), cannot_start_reason: nullable(r.cannot_start_reason, v => text(v, 256))
  };
}
export interface VehiclePort {
  getSnapshot(): VehicleView;
  subscribe(listener: () => void): () => void;
  fenceManual(): void;
  onManualFenced(listener: () => void): () => void;
  acquire(): Promise<Authority>;
  release(authorityId: string): Promise<void>;
  stop(authorityId: string, gesture: number): Promise<void>;
  /** Resolves after local send; vehicle status arrives independently. */
  intent(value: Intent): Promise<void>;
  recover(kind: 'estop_reset' | 'fault_recover'): Promise<void>;
  taskCommand(action: TaskAction): Promise<void>;
  learningStart(requestId: string, activeRevision: number): Promise<LearningStart>;
  learningStatus(query?: { attempt_id?: string; start_request_id?: string }): Promise<LearningStatus>;
  settingsRead(): Promise<SettingsData>;
  settingsApply(request: SettingsRequest): Promise<SettingsReceipt>;
  settingsResult(id: string): Promise<SettingsReceipt | null>;
  readonly logs: VehicleLogPort | null;
}
export const EMPTY_VIEW: VehicleView = Object.freeze({
  manual_connected: false,
  available: false,
  received_at_ms: null,
  frame: null,
  connected: false,
  rtt_ms: null
});
const MODES: readonly Mode[] = ['unknown', 'idle', 'manual', 'task', 'braking', 'obstacle_hold', 'feedback_stop', 'execution_fault', 'estop'];
const CAPS: readonly Capability[] = ['runtime.v1', 'motion.intent.v1', 'settings.v1', 'task.execution.v1', 'logs.v1', 'learning.v1', 'task.names.v1'];
const LEVELS: readonly Level[] = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'];
const ACTIONS: readonly Action[] = ['acquire', 'estop_reset', 'fault_recover'];
const TASK_ACTIONS: readonly TaskAction[] = ['pause', 'resume', 'cancel', 'takeover'];
const DRIVING = ['low_linear_mps', 'high_linear_mps', 'low_angular_rps', 'high_angular_rps', 'reverse_linear_mps', 'degraded_linear_mps'] as const;
export class VehicleDecodeError extends Error {
  constructor() {
    super('车端数据格式不合法');
  }
}
export function record(value: unknown, keys?: readonly string[]): Record<string, unknown> {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) throw new VehicleDecodeError();
  const r = value as Record<string, unknown>;
  if (keys && (Object.keys(r).length !== keys.length || keys.some(k => !(k in r)))) throw new VehicleDecodeError();
  return r;
}
export function text(value: unknown, bytes = 128): string {
  if (typeof value !== 'string' || new TextEncoder().encode(value).length > bytes) throw new VehicleDecodeError();
  return value;
}
export function number(value: unknown, min = -Infinity, max = Infinity): number {
  if (typeof value !== 'number' || !Number.isFinite(value) || value < min || value > max) throw new VehicleDecodeError();
  return value;
}
export function count(value: unknown, max = Number.MAX_SAFE_INTEGER): number {
  const n = number(value, 0, max);
  if (!Number.isSafeInteger(n)) throw new VehicleDecodeError();
  return n;
}
export function boolean(value: unknown): boolean {
  if (typeof value !== 'boolean') throw new VehicleDecodeError();
  return value;
}
export function oneOf<T extends string>(value: unknown, values: readonly T[]): T {
  if (typeof value !== 'string' || !values.includes(value as T)) throw new VehicleDecodeError();
  return value as T;
}
export function array<T>(value: unknown, parse: (item: unknown) => T, max: number): T[] {
  if (!Array.isArray(value) || value.length > max) throw new VehicleDecodeError();
  return value.map(parse);
}
function nullable<T>(value: unknown, parse: (item: unknown) => T): T | null {
  return value === null ? null : parse(value);
}
function body(value: unknown): BodyReference {
  const r = record(value, ['kind', 'average', 'source', 'age_seconds', 'stale']);
  const a = record(r.average, ['vx', 'vy', 'wz']);
  return {
    kind: oneOf(r.kind, ['independent_body']),
    average: { vx: number(a.vx), vy: number(a.vy), wz: number(a.wz) },
    source: text(r.source),
    age_seconds: number(r.age_seconds, 0),
    stale: boolean(r.stale)
  };
}
/** Unicode character count matches the task store's Python validation. */
export function isTaskName(value: unknown): value is string {
  return typeof value === 'string' && value.trim().length > 0 && Array.from(value).length <= 128;
}
function nonemptyId(value: unknown): string {
  const id = text(value);
  if (!id.trim()) throw new VehicleDecodeError();
  return id;
}
function taskName(value: unknown): string {
  if (!isTaskName(value)) throw new VehicleDecodeError();
  return value;
}
function task(value: unknown): TaskExecution {
  const raw = record(value);
  //Compatible: metadata is optional on prior agents and direct navigation executions.
  const keys = ['task_id', 'version', 'phase', 'completed', 'total', 'stopped_for', 'allowed_actions'];
  const r = record(raw, [...keys, ...['name', 'definition_task_id'].filter(key => key in raw)]);
  const total = count(r.total);
  const completed = count(r.completed, total);
  return {
    ...('name' in r ? { name: taskName(r.name) } : {}),
    ...('definition_task_id' in r ? { definition_task_id: nonemptyId(r.definition_task_id) } : {}),
    task_id: text(r.task_id),
    version: count(r.version),
    phase: oneOf(r.phase, ['starting', 'running', 'revoking', 'paused', 'succeeded', 'canceled', 'failed']),
    completed,
    total,
    stopped_for: nullable(r.stopped_for, text),
    allowed_actions: array(r.allowed_actions, v => oneOf(v, TASK_ACTIONS), 4)
  };
}
export function decodeVehicleFrame(value: unknown): VehicleFrame {
  const keys = ['manual_authority', 'schema', 'sequence', 'mode', 'reason', 'estop_revision', 'fault_revision', 'confirmed_estop_status', 'body_reference', 'wheel_linear_mps', 'task', 'allowed_actions', 'capabilities'];
  const raw = record(value);
  //Compatible: accept the prior exact frame shape during agent rollout.
  const r = record(raw, 'observation' in raw ? [...keys, 'observation'] : keys);
  if (r.schema !== 1) throw new VehicleDecodeError();
  const authority = nullable(r.manual_authority, value => {
    const a = record(value, ['authority_id', 'state']);
    return { authority_id: text(a.authority_id), state: oneOf(a.state, ['prepared', 'active', 'closed'] as const) };
  });
  const result: VehicleFrame = {
    manual_authority: authority,
    schema: 1,
    sequence: count(r.sequence),
    mode: oneOf(r.mode, MODES),
    reason: text(r.reason, 512),
    estop_revision: count(r.estop_revision),
    fault_revision: count(r.fault_revision),
    confirmed_estop_status: oneOf(r.confirmed_estop_status, ['unknown', 'clear', 'software_latched']),
    body_reference: nullable(r.body_reference, body),
    wheel_linear_mps: nullable(r.wheel_linear_mps, number),
    task: nullable(r.task, task),
    allowed_actions: array(r.allowed_actions, v => oneOf(v, ACTIONS), 3),
    capabilities: array(r.capabilities, v => oneOf(v, CAPS), CAPS.length)
  };
  if ('observation' in r) {
    const o = record(r.observation, ['status', 'last_sampled_steady', 'age_seconds']);
    result.observation = {
      status: oneOf(o.status, ['current', 'missing', 'revoked', 'ipc_unavailable']),
      last_sampled_steady: nullable(o.last_sampled_steady, number),
      age_seconds: nullable(o.age_seconds, v => number(v, 0))
    };
  }
  if (result.mode === 'unknown') {
    const claimsCurrentState = result.confirmed_estop_status !== 'unknown'
      || result.allowed_actions.length > 0 || result.manual_authority !== null
      || result.body_reference !== null || result.wheel_linear_mps !== null;
    if (!result.observation || result.observation.status === 'current' || claimsCurrentState) {
      throw new VehicleDecodeError();
    }
    return result;
  }
  if (result.confirmed_estop_status === 'unknown' || (result.observation && result.observation.status !== 'current')) throw new VehicleDecodeError();
  if ((result.mode === 'estop') !== (result.confirmed_estop_status !== 'clear')) throw new VehicleDecodeError();
  return result;
}
export function decodeDriving(value: unknown): DrivingSettings {
  const r = record(value, DRIVING);
  const d = Object.fromEntries(DRIVING.map(k => [k, number(r[k], 0)])) as unknown as DrivingSettings;
  if (d.low_linear_mps > d.high_linear_mps) throw new VehicleDecodeError();
  return d;
}
function logging(value: unknown): LoggingSettings {
  const r = record(value, ['level', 'retention_seconds', 'debug_enabled', 'debug_restore_seconds']);
  return {
    level: oneOf(r.level, LEVELS),
    retention_seconds: count(number(r.retention_seconds, 3600, 2592000)),
    debug_enabled: boolean(r.debug_enabled),
    debug_restore_seconds: count(number(r.debug_restore_seconds, 60, 3600))
  };
}
export function decodeSettingsSnapshot(value: unknown): SettingsSnapshot {
  const r = record(value, ['driving', 'logging', 'driving_version', 'logging_version', 'debug_original_level', 'debug_deadline_utc']);
  const s = {
    driving: decodeDriving(r.driving),
    logging: logging(r.logging),
    driving_version: count(r.driving_version),
    logging_version: count(r.logging_version),
    debug_original_level: nullable(r.debug_original_level, v => oneOf(v, LEVELS)),
    debug_deadline_utc: nullable(r.debug_deadline_utc, number)
  };
  if (s.logging.debug_enabled !== (s.debug_original_level !== null && s.debug_deadline_utc !== null) || (!s.logging.debug_enabled && (s.debug_original_level !== null || s.debug_deadline_utc !== null))) throw new VehicleDecodeError();
  return s;
}
function capabilities(value: unknown): SettingsCapabilities {
  const r = record(value, ['driving_defaults', 'driving_minimum', 'driving_maximum', 'logging_defaults', 'retention_min_seconds', 'retention_max_seconds', 'debug_restore_min_seconds', 'debug_restore_max_seconds']);
  return {
    driving_defaults: decodeDriving(r.driving_defaults),
    driving_minimum: decodeDriving(r.driving_minimum),
    driving_maximum: decodeDriving(r.driving_maximum),
    logging_defaults: logging(r.logging_defaults),
    retention_min_seconds: count(r.retention_min_seconds),
    retention_max_seconds: count(r.retention_max_seconds),
    debug_restore_min_seconds: count(r.debug_restore_min_seconds),
    debug_restore_max_seconds: count(r.debug_restore_max_seconds)
  };
}
function settingsData(value: unknown): SettingsData {
  const r = record(value, ['snapshot', 'capabilities', 'can_apply_driving']);
  return { snapshot: decodeSettingsSnapshot(r.snapshot), capabilities: capabilities(r.capabilities), can_apply_driving: boolean(r.can_apply_driving) };
}
function settingsReceipt(value: unknown): SettingsReceipt {
  const r = record(value, ['request_id', 'group', 'reason', 'snapshot', 'capabilities']);
  return {
    request_id: text(r.request_id),
    group: oneOf(r.group, ['driving', 'logging']),
    reason: oneOf(r.reason, ['applied', 'version_conflict', 'stop_required', 'invalid_values', 'request_conflict']),
    snapshot: decodeSettingsSnapshot(r.snapshot),
    capabilities: capabilities(r.capabilities)
  };
}
export function decodeLogGap(value: unknown): LogGap {
  const r = record(value, ['reason', 'count', 'after', 'through', 'durable']);
  return {
    reason: text(r.reason),
    count: nullable(r.count, count),
    after: nullable(r.after, count),
    through: nullable(r.through, count),
    durable: boolean(r.durable)
  };
}
function logEntry(value: unknown): LogEntry {
  const r = record(value, ['sequence', 'kind', 'level', 'module', 'summary', 'detail', 'fields_json', 'source_utc', 'received_utc', 'request_id', 'task_id', 'loss_reason', 'loss_count']);
  const fields = text(r.fields_json, 4096);
  JSON.parse(fields);
  return {
    sequence: count(r.sequence),
    kind: oneOf(r.kind, ['event', 'gap']),
    level: oneOf(r.level, LEVELS),
    module: text(r.module),
    summary: text(r.summary, 512),
    detail: text(r.detail, 4096),
    fields_json: fields,
    source_utc: number(r.source_utc),
    received_utc: number(r.received_utc),
    request_id: nullable(r.request_id, text),
    task_id: nullable(r.task_id, text),
    loss_reason: nullable(r.loss_reason, text),
    loss_count: nullable(r.loss_count, count)
  };
}
export function decodeLogPage(value: unknown): LogPage {
  if (new TextEncoder().encode(JSON.stringify(value)).length > 256 * 1024) throw new VehicleDecodeError();
  const r = record(value, ['entries', 'gaps', 'next_cursor', 'high_water', 'has_more', 'scanned']);
  const p = {
    entries: array(r.entries, logEntry, 200),
    gaps: array(r.gaps, decodeLogGap, 16),
    next_cursor: count(r.next_cursor),
    high_water: count(r.high_water),
    has_more: boolean(r.has_more),
    scanned: count(r.scanned, 200)
  };
  if (p.next_cursor > p.high_water || p.entries.some((e, i) => e.sequence > p.next_cursor || (i > 0 && e.sequence <= p.entries[i - 1]!.sequence))) throw new VehicleDecodeError();
  return p;
}
export type VehicleRequestType = 'control.acquire' | 'control.release' | 'drive.stop' | 'safety.estop.reset' | 'safety.fault.recover' | 'task.execution.command' | 'settings.get' | 'settings.apply' | 'settings.result' | 'learning.start' | 'learning.status';
export const VEHICLE_REQUEST_TYPES: readonly string[] = ['safety.estop.reset', 'safety.fault.recover', 'task.execution.command', 'settings.get', 'settings.apply', 'settings.result', 'learning.start', 'learning.status'];
/** Validate locally before marking a request as possibly submitted. */
export function validateSettingsRequest(request: SettingsRequest): void {
  if (!text(request.request_id)) throw new VehicleDecodeError();
  count(request.expected_version);
  boolean(request.restore_defaults);
  oneOf(request.group, ['driving', 'logging']);
  if (request.group === 'driving') decodeDriving(request.values);
  else logging(request.values);
}
export class VehicleTransport implements VehiclePort {
  private view: VehicleView = EMPTY_VIEW;
  private readonly listeners = new Set<() => void>();
  private readonly manualListeners = new Set<() => void>();
  fenceManual(): void {
    this.manualListeners.forEach(listener => listener());
  }
  onManualFenced(listener: () => void): () => void {
    this.manualListeners.add(listener);
    return () => this.manualListeners.delete(listener);
  }
  constructor(private readonly send: (type: VehicleRequestType | 'drive.intent', payload: unknown, requestId?: string) => Promise<unknown>, readonly logs: VehicleLogPort | null = null) {
  }
  getSnapshot = (): VehicleView => this.view;
  subscribe = (listener: () => void): (() => void) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };
  update(value: unknown, connected: boolean): void {
    const frame = decodeVehicleFrame(value), old = this.view.frame;
    if (old && (frame.sequence <= old.sequence || frame.estop_revision < old.estop_revision || frame.fault_revision < old.fault_revision)) {
      this.view = { ...this.view, connected, available: this.view.available && connected };
      this.publish();
      return;
    }
    this.view = {
      ...this.view,
      frame,
      connected,
      available: connected,
      received_at_ms: performance.now()
    };
    this.publish();
  }
  reset(): void {
    this.view = EMPTY_VIEW;
    this.publish();
  }
  setConnection(connected: boolean, rtt = this.view.rtt_ms): void {
    this.view = {
      ...this.view,
      connected,
      manual_connected: connected && this.view.manual_connected,
      available: connected && this.view.available,
      rtt_ms: rtt
    };
    if (!connected) this.fenceManual();
    this.publish();
  }
  setUnsupportedConnection(connected: boolean): void {
    this.view = { ...this.view, connected, available: false };
    this.publish();
  }
  setRtt(rtt: number): void {
    this.view = { ...this.view, rtt_ms: rtt };
    this.publish();
  }
  setManualConnection(connected: boolean): void {
    this.view = { ...this.view, manual_connected: connected };
    if (!connected) this.fenceManual();
    this.publish();
  }
  markReportStale(): void {
    this.view = { ...this.view, connected: false, available: false };
    this.publish();
  }
  private publish(): void {
    this.listeners.forEach(l => l());
  }
  private require(cap: Capability): VehicleFrame {
    const frame = this.view.frame;
    if (!this.view.connected || !this.view.available || !frame?.capabilities.includes(cap)) throw new Error('车端能力未接通或车辆离线');
    return frame;
  }
  async acquire(): Promise<Authority> {
    const r = record(await this.send('control.acquire', {}));
    if (r.state !== 'prepared') throw new VehicleDecodeError();
    const id = text(r.authority_id);
    if (!id) throw new VehicleDecodeError();
    return { authority_id: id, state: 'prepared' };
  }
  async release(id: string): Promise<void> {
    await this.send('control.release', { authority_id: id });
  }
  async stop(authorityId: string, gesture: number): Promise<void> {
    if (!text(authorityId) || count(gesture) < 1) throw new VehicleDecodeError();
    await this.send('drive.stop', { authority_id: authorityId, gesture });
  }
  async intent(value: Intent): Promise<void> {
    number(value.linear, -1, 1);
    number(value.turn, -1, 1);
    count(value.gesture);
    count(value.sequence);
    oneOf(value.gear, ['low', 'high']);
    if (!text(value.authority_id)) throw new VehicleDecodeError();
    await this.send('drive.intent', value);
  }
  async recover(kind: 'estop_reset' | 'fault_recover'): Promise<void> {
    const f = this.require('runtime.v1');
    if (!f.allowed_actions.includes(kind)) throw new Error('车端未允许恢复');
    await this.send(kind === 'estop_reset' ? 'safety.estop.reset' : 'safety.fault.recover', { revision: kind === 'estop_reset' ? f.estop_revision : f.fault_revision });
  }
  async taskCommand(action: TaskAction): Promise<void> {
    const frame = this.require('task.execution.v1');
    const task = frame.task;
    if (!task?.allowed_actions.includes(action)) throw new Error('车端未允许该任务动作');
    const requestId = crypto.randomUUID();
    const response = record(await this.send('task.execution.command', {
      request_id: requestId,
      expected_version: task.version,
      task_id: task.task_id,
      action
    }), ['request_id', 'task_id', 'accepted', 'reason', 'version']);
    if (text(response.request_id) !== requestId || text(response.task_id) !== task.task_id) throw new VehicleDecodeError();
    count(response.version);
    const accepted = boolean(response.accepted), reason = text(response.reason, 512);
    if (!accepted) throw new Error(`车端未受理：${reason}`);
  }
  async learningStart(requestId: string, activeRevision: number): Promise<LearningStart> {
    const frame = this.require('learning.v1');
    if (!text(requestId)) throw new VehicleDecodeError();
    count(activeRevision);
    const response = decodeLearningStart(await this.send('learning.start', {
      expected_active_revision: activeRevision,
      expected_authority_id: frame.manual_authority?.state === 'prepared' ? frame.manual_authority.authority_id : null
    }, requestId));
    if (response.start_request_id !== requestId) throw new VehicleDecodeError();
    return response;
  }
  async learningStatus(query: { attempt_id?: string; start_request_id?: string } = {}): Promise<LearningStatus> {
    this.require('learning.v1');
    if (query.attempt_id !== undefined) learningId(query.attempt_id);
    if (query.start_request_id !== undefined && !text(query.start_request_id)) throw new VehicleDecodeError();
    if (query.attempt_id !== undefined && query.start_request_id !== undefined) throw new VehicleDecodeError();
    return decodeLearningStatus(await this.send('learning.status', query));
  }
  async settingsRead(): Promise<SettingsData> {
    this.require('settings.v1');
    return settingsData(await this.send('settings.get', {}));
  }
  async settingsApply(request: SettingsRequest): Promise<SettingsReceipt> {
    this.require('settings.v1');
    validateSettingsRequest(request);
    const r = settingsReceipt(await this.send('settings.apply', request));
    if (r.request_id !== request.request_id || r.group !== request.group) throw new VehicleDecodeError();
    return r;
  }
  async settingsResult(id: string): Promise<SettingsReceipt | null> {
    this.require('settings.v1');
    const data = await this.send('settings.result', { request_id: id });
    if (data === null) return null;
    const r = settingsReceipt(data);
    if (r.request_id !== id) throw new VehicleDecodeError();
    return r;
  }
}
export function decodeLogDownload(value: unknown): LogDownload {
  const r = record(value, ['data', 'next_cursor', 'has_more', 'gaps']);
  return {
    data: text(r.data, 2 * 1024 * 1024),
    next_cursor: count(r.next_cursor),
    has_more: boolean(r.has_more),
    gaps: array(r.gaps, decodeLogGap, 16)
  };
}
