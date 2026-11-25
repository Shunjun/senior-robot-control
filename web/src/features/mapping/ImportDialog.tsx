import type { ReactElement } from "react";
import { Button } from "../../components/ui/Button";

export function ImportDialog({
  disabled,
  onImport,
}: {
  disabled: boolean;
  onImport: () => void;
}): ReactElement {
  return (
    <Button variant="primary" disabled={disabled} onClick={onImport}>
      导入地图包
    </Button>
  );
}
