import type { ReactElement } from "react";

export function ImportDialog({ disabled, onImport }: {
  disabled: boolean;
  onImport: () => void;
}): ReactElement {
  return (
    <button type="button" disabled={disabled} onClick={onImport}>
      导入地图包
    </button>
  );
}
