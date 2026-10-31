"""Fail-closed freshness tracking across ROS and steady clocks."""
import math


class SafetyFreshness:
    def __init__(self, max_age: float) -> None:
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError("max_age must be positive")
        self.max_age = max_age
        self.last_ros = None
        self.scan_stamp = None
        self.measured_stamp = None
        self.scan_received = None
        self.measured_received = None

    def _clock(self, ros: float) -> bool:
        if not math.isfinite(ros) or (self.last_ros is not None and ros < self.last_ros):
            self.scan_received = self.measured_received = None
            self.scan_stamp = self.measured_stamp = None
            self.last_ros = ros if math.isfinite(ros) else None
            return False
        self.last_ros = ros
        return True

    def _accept(self, kind: str, stamp: float, steady: float, ros: float) -> bool:
        if not self._clock(ros):
            return False
        previous = getattr(self, kind + "_stamp")
        valid = (math.isfinite(stamp) and math.isfinite(steady) and stamp > 0
                 and 0 <= ros - stamp <= self.max_age
                 and (previous is None or stamp > previous))
        if not valid:
            setattr(self, kind + "_received", None)
            return False
        setattr(self, kind + "_stamp", stamp)
        setattr(self, kind + "_received", steady)
        return True

    def scan(self, stamp: float, steady: float, ros: float) -> bool:
        return self._accept("scan", stamp, steady, ros)

    def measured(self, stamp: float, steady: float, ros: float) -> bool:
        return self._accept("measured", stamp, steady, ros)

    def usable(self, ros: float, steady: float) -> bool:
        if not self._clock(ros) or not math.isfinite(steady):
            return False
        return all(received is not None and 0 <= steady - received <= self.max_age
                   and stamp is not None and 0 <= ros - stamp <= self.max_age
                   for received, stamp in ((self.scan_received, self.scan_stamp),
                                           (self.measured_received, self.measured_stamp)))
