import { useEffect, useRef, useState } from "react";
import type { MapPoint, RobotSnapshot } from "../../api/protocol";

interface Lease {
  robot: string;
  order?: NonNullable<RobotSnapshot["display_order"]>;
  scope?: string;
  sequence: number;
  deadline: number;
}

function lease(robot: string): Lease {
  return { robot, sequence: -1, deadline: 0 };
}

/** Display-only residence lease, after the original client fences transport and publication. */
export function useCurrentScan(snapshot: RobotSnapshot): readonly MapPoint[] {
  const current = useRef<Lease>(lease(snapshot.robot_id));
  const [, tick] = useState(0);
  if (current.current.robot !== snapshot.robot_id) current.current = lease(snapshot.robot_id);
  const state = current.current;
  const layer = snapshot.scan_layer;
  const order = snapshot.display_order;
  const previous = state.order;
  const now = performance.now();
  let visible = false;
  const older = order !== undefined && previous !== undefined && order.client === previous.client && (
    order.generation < previous.generation || (order.generation === previous.generation && (
      order.epoch < previous.epoch || (order.epoch === previous.epoch && order.sequence < previous.sequence)
    ))
  );
  if (order !== undefined && !older && order.sequence > 0) {
    const samePublication = previous !== undefined && order.client === previous.client &&
      order.generation === previous.generation && order.epoch === previous.epoch &&
      order.sequence === previous.sequence;
    const scope = layer === undefined ? undefined : JSON.stringify([
      layer.map_ref.map_id, layer.map_ref.reference_revision, layer.map_generation,
      layer.backend_boot, layer.source_id,
    ]);
    const same = scope === state.scope;
    const rejected = layer !== undefined && (
      (same && layer.sequence < state.sequence) ||
      (samePublication && (!same || layer.sequence !== state.sequence))
    );
    state.order = order;
    if (!rejected) {
      if (layer === undefined || !snapshot.connected || snapshot.stale || !snapshot.localized ||
          snapshot.map_ref === null || snapshot.map_generation !== layer.map_generation ||
          snapshot.map_ref.map_id !== layer.map_ref.map_id ||
          snapshot.map_ref.reference_revision !== layer.map_ref.reference_revision) {
        state.deadline = Math.min(state.deadline, now);
      } else {
        const deadline = layer.valid ? now + layer.remaining_s * 1000 : now;
        state.deadline = same && layer.sequence === state.sequence
          ? Math.min(state.deadline, deadline) : deadline;
        state.scope = scope!;
        state.sequence = layer.sequence;
        visible = layer.valid && state.deadline > now;
      }
    }
  }
  // //Compatible: legacy snapshots have no trusted display order and cannot grant Q.
  const deadline = state.deadline;
  useEffect(() => {
    if (deadline <= performance.now()) return;
    const timer = setTimeout(() => tick(value => value + 1), Math.ceil(deadline - performance.now()));
    return () => clearTimeout(timer);
  }, [deadline]);
  return visible ? (snapshot.scan_points ?? []) : [];
}
