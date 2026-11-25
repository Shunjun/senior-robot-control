import type { ReactElement } from 'react';
import type { VehiclePort } from '../../api/vehicleState';
import { VehicleSettings } from '../settings/VehicleSettings';
import { VehicleLogs } from '../logs/VehicleLogs';
import { Button } from '../../components/ui/Button';
import type { WorkspacePage } from './useWorkspaceView';
import { useVehicleView } from './useVehicleView';
interface VehicleToolsProps { port?: VehiclePort; page?: WorkspacePage; onBack?: () => void }
export function VehicleTools({ port, page = "control", onBack }: VehicleToolsProps): ReactElement {
  const view = useVehicleView(port);
  return <><div hidden={page === "control"}>{page !== "control" && <Button onClick={onBack}>返回控制</Button>}</div><div hidden={page !== "settings"}>
    <VehicleSettings {...(port ? { port } : {})} enabled={view.available && view.connected && view.frame?.capabilities.includes('settings.v1') === true} />
    </div><div hidden={page !== "logs"}><VehicleLogs port={port?.logs ?? null} enabled={view.available && view.connected && view.frame?.capabilities.includes('logs.v1') === true} />
  </div></>;
}
