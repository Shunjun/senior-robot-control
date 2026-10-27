"""Small JSON log formatter with explicit, non-sensitive fields."""

import json
import logging
import re


class JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        message = re.sub(
            r"(?i)(token|password|cookie|pairing_code)\s*[:=]\s*[^\s,;]+",
            r"\1=[REDACTED]",
            record.getMessage(),
        )
        data = {"level": record.levelname, "message": message}
        for key in ("robot_id", "request_id", "operation_id", "execution_id"):
            value = getattr(record, key, None)
            if isinstance(value, str):
                data[key] = value
        return json.dumps(data, ensure_ascii=False)
