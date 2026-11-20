"""Explicit cloud-only settings. Importing this module reads no environment."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, SecretStr, model_validator

from cloud.diagnostics import DiagnosticsSettings


class CloudSettings(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, hide_input_in_errors=True)
    diagnostics: DiagnosticsSettings = Field(default_factory=DiagnosticsSettings)
    secret: SecretStr
    role: Literal["cloud"] = "cloud"

    @model_validator(mode="after")
    def nonempty_secret(self) -> "CloudSettings":
        if not self.secret.get_secret_value().strip():
            raise ValueError("secret is required")
        return self
