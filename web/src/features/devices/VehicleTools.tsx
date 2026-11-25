import type { ReactElement } from "react";
import type { VehiclePort } from "../../api/vehicleState";
import { VehicleSettings } from "../settings/VehicleSettings";
import { VehicleLogs } from "../logs/VehicleLogs";

import type { WorkspacePage } from "./useWorkspaceView";
import { useVehicleView } from "./useVehicleView";
interface VehicleToolsProps {
  port?: VehiclePort;
  page?: WorkspacePage;
  onBack?: () => void;
  cameraOn?: boolean;
  onCameraChange?: (value: boolean) => void;
}
export function VehicleTools({
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
