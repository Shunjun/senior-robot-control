import { useSyncExternalStore } from 'react';
import { EMPTY_VIEW, type VehiclePort, type VehicleView } from '../../api/vehicleState';
const none = () => () => {
};
const empty = () => EMPTY_VIEW;
export function useVehicleView(port: VehiclePort | undefined): VehicleView {
  return useSyncExternalStore(port?.subscribe ?? none, port?.getSnapshot ?? empty, empty);
}
