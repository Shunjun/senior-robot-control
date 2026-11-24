import { useEffect, useState, useSyncExternalStore } from 'react';
import { EMPTY_VIEW, type VehiclePort, type VehicleView } from '../../api/vehicleState';
const none = () => () => {
};
const empty = () => EMPTY_VIEW;
export function useVehicleView(port: VehiclePort | undefined): VehicleView {
  return useSyncExternalStore(port?.subscribe ?? none, port?.getSnapshot ?? empty, empty);
}

/** Display freshness follows the existing relay's 5s report deadline, never RTT or render time.
 * It does not affect the independently fenced manual transport/authority. */
export function useMeasuredSpeed(view: VehicleView): number | null {
  const [expiredReceipt,setExpiredReceipt]=useState<number|null>(null);
  useEffect(()=>{
    const receipt=view.received_at_ms;
    if(receipt===null) return;
    const remaining=5000-(performance.now()-receipt);
    const timer=setTimeout(()=>setExpiredReceipt(receipt),Math.max(0,remaining));
    return ()=>clearTimeout(timer);
  },[view.received_at_ms,view.frame?.sequence]);
  const receipt=view.received_at_ms;
  const expired=receipt===null || expiredReceipt===receipt || performance.now()-receipt>=5000;
  if(!view.connected || !view.available || expired) return null;
  const body=view.frame?.body_reference;
  return body && !body.stale ? body.average.vx : view.frame?.wheel_linear_mps ?? null;
}
