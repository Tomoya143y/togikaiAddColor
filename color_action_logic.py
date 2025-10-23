
# color_action_logic.py
# Drop-in helper to trigger actions after N consecutive same-color reads.
# Usage in your run.py:
#   from color_action_logic import ColorActionManager
#   color_mgr = ColorActionManager(
#       n_required=getattr(config, "COLOR_CONSECUTIVE_N", 10),
#       actions=getattr(config, "COLOR_TRIGGER_ACTIONS", {}),
#       ignore_colors=set(getattr(config, "COLOR_IGNORE_SET", {"unknown"})),
#       cooldown=getattr(config, "COLOR_TRIGGER_COOLDOWN", 0),
#       verbose=getattr(config, "PRINT_COLOR_TRIGGER_LOG", True),
#   )
#   ...
#   detected = <your color string from color_runtime_detector>  # e.g., "pink" | "green" | "white" | "unknown"
#   color_mgr.process(detected)  # calls the mapped action once the same color is seen N times in a row
#
# This module is self-contained and does not depend on GPIO or sensor code.

from typing import Callable, Dict, Optional, Set

class ColorActionManager:
    def __init__(
        self,
        n_required: int,
        actions: Dict[str, Callable],
        ignore_colors: Optional[Set[str]] = None,
        cooldown: int = 0,
        verbose: bool = True,
    ):
        if n_required < 1:
            raise ValueError("n_required must be >= 1")
        self.n_required = n_required
        self.actions = dict(actions or {})
        self.ignore_colors = set(ignore_colors or set())
        self.cooldown = max(0, int(cooldown))
        self.verbose = verbose

        self._current_color: Optional[str] = None
        self._count: int = 0
        self._cooldown_left: int = 0

    @property
    def count(self) -> int:
        return self._count

    @property
    def current_color(self) -> Optional[str]:
        return self._current_color

    def reset(self):
        self._current_color = None
        self._count = 0
        self._cooldown_left = 0

    def _log(self, *args):
        if self.verbose:
            print(*args)

    def process(self, color: Optional[str]) -> Optional[str]:
        """
        Feed one color label (string).
        Returns the color that triggered (when N consecutive reads reached), otherwise None.
        Also executes the mapped action if available.
        """
        # Cooldown handling
        if self._cooldown_left > 0:
            self._cooldown_left -= 1

        if color is None:
            # treat None like a break in the sequence
            if self._current_color is not None:
                self._log(f"[color] break: {self._current_color} x{self._count} -> None")
            self._current_color = None
            self._count = 0
            return None

        if color in self.ignore_colors:
            # ignore but also treat like a break unless the ignored color equals current
            if color != self._current_color:
                if self._current_color is not None:
                    self._log(f"[color] break: {self._current_color} x{self._count} -> {color}(ignored)")
                self._current_color = None
                self._count = 0
            return None

        # Same color streak
        if color == self._current_color:
            self._count += 1
        else:
            # new color begins
            if self._current_color is not None:
                self._log(f"[color] switch: {self._current_color} x{self._count} -> {color}")
            self._current_color = color
            self._count = 1

        # Not yet reached the threshold
        if self._count < self.n_required:
            if self.verbose:
                self._log(f"[color] streak {self._current_color}: {self._count}/{self.n_required}")
            return None

        # Reached threshold: trigger (only if not in cooldown)
        if self._cooldown_left > 0:
            # We reached N again but still in cooldown; don't fire action
            self._log(f"[color] {self._current_color} reached {self.n_required} but in cooldown ({self._cooldown_left} frames left)")
            return None

        decided = self._current_color
        self._log(f"[color] DECIDED: {decided} (x{self._count}) -> trigger")

        # Execute corresponding action if exists
        action = self.actions.get(decided)
        if callable(action):
            try:
                action()
            except Exception as e:
                self._log(f"[color] action for '{decided}' raised: {e}")

        # start cooldown (optional) and keep current streak running so we won't retrigger until streak breaks or cooldown elapses
        if self.cooldown > 0:
            self._cooldown_left = self.cooldown

        # To avoid immediate retriggers if the same color keeps coming without cooldown,
        # you can reset the counter right after firing. We keep the streak so that logs remain meaningful.
        # If you prefer reset-on-fire behavior, uncomment the next two lines:
        # self._current_color = None
        # self._count = 0

        return decided
