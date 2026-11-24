import { useState } from 'react';
import type { SpeedUnit } from '../../display/speedUnits';
export type WorkspacePage = 'control' | 'logs' | 'settings';
export function useWorkspaceView() {
  const [page, setPage] = useState<WorkspacePage>('control');
  const [unit, setUnit] = useState<SpeedUnit>('m/s');
  const [statusOpen, setStatusOpen] = useState(false);
  return { page, setPage, unit, setUnit, statusOpen, setStatusOpen };
}
