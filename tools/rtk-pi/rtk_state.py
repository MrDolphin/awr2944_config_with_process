"""Position-state tracking with fixed-solution hysteresis."""

import time
from enum import Enum


class PositionState(Enum):
    NO_FIX = "NO FIX"
    SINGLE = "SINGLE"
    DGPS = "DGPS"
    RTK_FLOAT = "RTK FLOAT"
    RTK_FIXED = "RTK FIXED"
    UNKNOWN = "UNKNOWN"


_RANK = {
    PositionState.NO_FIX: 0,
    PositionState.SINGLE: 1,
    PositionState.DGPS: 2,
    PositionState.RTK_FLOAT: 3,
    PositionState.RTK_FIXED: 4,
}


class RtkState:
    def __init__(self, min_fixed_epochs=8, fix_timeout_s=25.0):
        self.min_fixed_epochs = int(min_fixed_epochs)
        self.fix_timeout_s = float(fix_timeout_s)
        self.position = PositionState.NO_FIX
        self.best_position = PositionState.NO_FIX
        self.heading_available = False
        self.heading_deg = 0.0
        self._fixed_streak = 0
        self._last_fixed_ts = None

    def _update_best(self, state):
        if _RANK.get(state, 0) > _RANK.get(self.best_position, 0):
            self.best_position = state

    def update_quality(self, quality, now=None):
        now = time.time() if now is None else now
        quality = int(quality)
        if quality == 4:
            self._fixed_streak += 1
            self._last_fixed_ts = now
            state = PositionState.RTK_FIXED
            if self._fixed_streak >= self.min_fixed_epochs:
                self.position = state
                self._update_best(state)
            return state

        self._fixed_streak = 0
        mapping = {
            0: PositionState.NO_FIX,
            1: PositionState.SINGLE,
            2: PositionState.DGPS,
            5: PositionState.RTK_FLOAT,
        }
        raw = mapping.get(quality, PositionState.UNKNOWN)
        hold_fixed = (
            self.position == PositionState.RTK_FIXED
            and self._last_fixed_ts is not None
            and now - self._last_fixed_ts <= self.fix_timeout_s
        )
        if not hold_fixed:
            self.position = raw
        self._update_best(raw)
        return self.position

    def update_heading(self, sol_stat, heading):
        if sol_stat != "SOL_COMPUTED":
            self.heading_available = False
            return False
        self.heading_available = True
        self.heading_deg = float(heading)
        return True