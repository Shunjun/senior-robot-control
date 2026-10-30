import { CommandError, type ControlApi, type RobotLink } from "./client";
import type { LinkStatus, RobotSnapshot, RobotSummary } from "./protocol";

// Demo data for looking at the console without a cloud or a robot. Every name carries the demo
// mark, the shell shows a permanent demo label, and no command ever leaves this module.

const DEMO_ROBOTS: readonly RobotSummary[] = [
  { id: "demo-alpha", name: "演示 · 仓库车 A", online: true },
  { id: "demo-beta", name: "演示 · 装卸车 B", online: false },
];

function demoSnapshot(robot: RobotSummary): RobotSnapshot {
  const now = Date.now() / 1000;
  return {
    robot_id: robot.id,
    connected: robot.online,
    localized: robot.online,
    map_ref: { map_id: "demo-map", reference_revision: 1 },
    home: null,
    task_revision: 0,
    runnable_task_ids: [],
    execution_status: robot.online ? "idle" : "unknown",
    collected_at: now,
    last_synced_at: robot.online ? now : now - 3600,
    stale: !robot.online,
  };
}

class DemoApi implements ControlApi {
  async login(): Promise<void> {}
  async logout(): Promise<void> {}
  async listRobots(): Promise<RobotSummary[]> {
    return DEMO_ROBOTS.map((robot) => ({ ...robot }));
  }
}

class DemoLink implements RobotLink {
  private readonly snapshotListeners = new Set<(snapshot: RobotSnapshot) => void>();
  private readonly statusListeners = new Set<(status: LinkStatus) => void>();
  private generation = 0;

  async connect(robotId: string): Promise<void> {
    const generation = ++this.generation;
    this.statusListeners.forEach((l) => l({ state: "open", robotId, closeCode: null }));
    const robot = DEMO_ROBOTS.find((item) => item.id === robotId);
    if (robot === undefined) return;
    queueMicrotask(() => {
      if (generation === this.generation) {
        this.snapshotListeners.forEach((listener) => listener(demoSnapshot(robot)));
      }
    });
  }

  disconnect(): void {
    this.generation += 1;
  }

  request<T>(): Promise<T> {
    return Promise.reject(new CommandError("safety_rejected", "演示模式不驱动底盘"));
  }

  subscribe(listener: (snapshot: RobotSnapshot) => void): () => void {
    this.snapshotListeners.add(listener);
    return () => this.snapshotListeners.delete(listener);
  }

  subscribeStatus(listener: (status: LinkStatus) => void): () => void {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }
}

export function createDemoBackend(): { api: ControlApi; link: RobotLink } {
  return { api: new DemoApi(), link: new DemoLink() };
}
