"""Voltage filtering and a latched low-voltage stop, independent of the browser."""
from collections import deque
import math
import statistics
from follow_settings import SETTINGS


class BatteryState:
    def __init__(self, settings=SETTINGS):
        self.settings = settings
        self.samples = deque(maxlen=100)
        self.low_since = None
        self.recovery_since = None
        self.latched = False

    def observe(self, voltage, now):
        if not math.isfinite(voltage) or not 0 < voltage < 30:
            return
        if self.samples and now - self.samples[-1][0] > 1:
            self.low_since = self.recovery_since = None
            self.samples.clear()
        self.samples.append((now, voltage))
        while self.samples and now - self.samples[0][0] > 1:
            self.samples.popleft()
        filtered = statistics.median(value for _, value in self.samples)
        if filtered < self.settings.battery_stop_v:
            if self.low_since is None:
                self.low_since = now
            if now - self.low_since >= self.settings.battery_stop_s:
                self.latched = True
        else:
            self.low_since = None
        if filtered >= self.settings.battery_slow_v:
            if self.recovery_since is None:
                self.recovery_since = now
        else:
            self.recovery_since = None

    def status(self, now):
        fresh = bool(self.samples) and now - self.samples[-1][0] <= 5
        voltage = statistics.median(v for _, v in self.samples) if fresh else None
        recoverable = (fresh and self.recovery_since is not None
                       and self.samples[-1][0] - self.recovery_since >= self.settings.battery_recover_s)
        if not fresh:
            limit, reason = 0.0, "等待电池电压数据"
        elif self.latched:
            limit, reason = 0.0, "低电压保护，请充电后重新启动"
        elif voltage < self.settings.battery_slow_v:
            limit, reason = self.settings.fallback_speed_mps, "电池电压偏低"
        else:
            limit, reason = self.settings.max_speed_mps, "正常"
        percent = None if voltage is None else round(100 * max(0, min(1, (voltage - 10.5) / 2.1)))
        return dict(voltage=round(voltage, 2) if voltage is not None else None,
                    percent=percent, estimated=True, latched=self.latched,
                    recoverable=bool(recoverable), limit_mps=limit, reason=reason)

    def arm(self, now):
        state = self.status(now)
        if self.latched and state["recoverable"]:
            self.latched = False
        return self.status(now)["limit_mps"] > 0
