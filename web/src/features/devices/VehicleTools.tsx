import type { ReactElement } from 'react';
import type { VehiclePort } from '../../api/vehicleState';
import { VehicleSettings } from '../settings/VehicleSettings';
import { VehicleLogs } from '../logs/VehicleLogs';
import { useVehicleView } from './useVehicleView';
export function VehicleTools({ port }: {
  port?: VehiclePort;
}): ReactElement {
  const view = useVehicleView(port);
  return <>
    <VehicleSettings {...(port ? { port } : {})} enabled={view.available && view.connected && view.frame?.capabilities.includes('settings.v1') === true} />
    <VehicleLogs port={port?.logs ?? null} enabled={view.available && view.connected && view.frame?.capabilities.includes('logs.v1') === true} />
  </>;
}
