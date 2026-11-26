import { useEffect, useRef, useState } from "react";
import type { MapPoint, RobotSnapshot } from "../../api/protocol";

interface Lease {
  robot: string;
  reference?: string;
  mapReference?: string;
  source?: string;
  boot?: string;
  sequence: number;
  deadline: number;
  retiredReferences: Set<string>;
  retiredSources: Set<string>;
  retiredBoots: Set<string>;
}

function lease(robot: string): Lease {
  return { robot, sequence: -1, deadline: 0, retiredReferences: new Set(),
    retiredSources: new Set(), retiredBoots: new Set() };
}

/** Display-only, local residence deadline. Vehicle monotonic timestamps are identities here. */
export function useCurrentScan(snapshot: RobotSnapshot): readonly MapPoint[] {
  const current = useRef<Lease>(lease(snapshot.robot_id));
  const [, tick] = useState(0);
  if (current.current.robot !== snapshot.robot_id) current.current = lease(snapshot.robot_id);
  const state = current.current;
  const layer = snapshot.scan_layer;
  const now = performance.now();
  let visible = false;
  const displayedReference = snapshot.map_ref === null || snapshot.map_generation === undefined
    ? undefined : JSON.stringify([snapshot.map_ref.map_id, snapshot.map_ref.reference_revision,
      snapshot.map_generation]);
  const retiredReference = displayedReference !== undefined && state.retiredReferences.has(displayedReference);
  // Reference retirement is observable even while the new reference has no Q yet.
  if (displayedReference !== undefined && !retiredReference) {
    if (state.mapReference !== undefined && state.mapReference !== displayedReference)
      state.retiredReferences.add(state.mapReference);
    state.mapReference = displayedReference;
  }
  if (retiredReference || layer === undefined || !snapshot.connected || snapshot.stale || !snapshot.localized ||
      snapshot.map_ref === null || snapshot.map_generation !== layer.map_generation ||
      snapshot.map_ref.map_id !== layer.map_ref.map_id ||
      snapshot.map_ref.reference_revision !== layer.map_ref.reference_revision) {
    if (!retiredReference) state.deadline = Math.min(state.deadline, now);
  } else {
    const reference = JSON.stringify([layer.map_ref.map_id, layer.map_ref.reference_revision,
      layer.map_generation]);
    const source = JSON.stringify([layer.backend_boot, layer.source_id]);
    const same = reference === state.reference && source === state.source;
    const rejected = state.retiredReferences.has(reference) || state.retiredSources.has(source) ||
      state.retiredBoots.has(layer.backend_boot) || (same && layer.sequence < state.sequence);
    if (!rejected) {
      const deadline = layer.valid ? now + layer.remaining_s * 1000 : now;
      if (same && layer.sequence === state.sequence) {
        state.deadline = Math.min(state.deadline, deadline);
      } else {
        if (state.reference !== undefined && state.reference !== reference)
          state.retiredReferences.add(state.reference);
        if (state.source !== undefined && state.source !== source) state.retiredSources.add(state.source);
        if (state.boot !== undefined && state.boot !== layer.backend_boot) state.retiredBoots.add(state.boot);
        state.reference = reference;
        state.source = source;
        state.boot = layer.backend_boot;
        state.sequence = layer.sequence;
        state.deadline = deadline;
      }
      visible = layer.valid && state.deadline > now;
    }
  }
  const deadline = state.deadline;
  useEffect(() => {
    if (deadline <= performance.now()) return;
    const timer = setTimeout(() => tick(value => value + 1), Math.ceil(deadline - performance.now()));
    return () => clearTimeout(timer);
  }, [deadline]);
  return visible ? (snapshot.scan_points ?? []) : [];
}
