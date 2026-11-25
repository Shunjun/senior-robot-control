import type { ReactElement } from "react";
import type { VehiclePort } from "../../api/vehicleState";
import { VehicleSettings } from "../settings/VehicleSettings";
import { VehicleLogs } from "../logs/VehicleLogs";

import type { WorkspacePage } from "./useWorkspaceView";
import { useVehicleView } from "./useVehicleView";
interface VehicleToolsProps {
  // Compatible: standalone consumers can omit session actions.
  onLogout?: (() => void) | undefined;
  loggingOut?: boolean | undefined;
  port?: VehiclePort;
  page?: WorkspacePage;
  onBack?: () => void;
  cameraOn?: boolean;
  onCameraChange?: (value: boolean) => void;
}
export function VehicleTools({
  onLogout,
  loggingOut,
  port,
  page = "control",
  onBack,
  cameraOn = false,
  onCameraChange,
}: VehicleToolsProps): ReactElement {
  const view = useVehicleView(port);
  return (
    <>
      <div hidden={page !== "settings"}>
        <VehicleSettings
          onBack={onBack}
          onLogout={onLogout}
          loggingOut={loggingOut}
          cameraOn={cameraOn}
          onCameraChange={onCameraChange}
          {...(port ? { port } : {})}
          enabled={
            view.available &&
            view.connected &&
            view.frame?.capabilities.includes("settings.v1") === true
          }
        />
      </div>
      <div hidden={page !== "logs"}>
        <VehicleLogs
          onBack={onBack}
          port={port?.logs ?? null}
          enabled={
            view.available &&
            view.connected &&
            view.frame?.capabilities.includes("logs.v1") === true
          }
        />
      </div>
    </>
  );
}
