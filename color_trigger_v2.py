
# color_trigger_v2.py
# Two-stage color decision & trigger FSM for TCS34725 pipelines.
# Stage A: Raw -> "True color" after N consecutive *same* raw readings.
# Stage B: Count "true color" transitions (increment only when previous true color is different).
#          When per-color count >= threshold, ask caller to execute the mapped action.

from collections import defaultdict

class ColorTriggerFSM:
    def __init__(
        self,
        target_colors,
        raw_same_n=3,                 # A: N consecutive same raw readings → accept as true color
        true_transition_n=3,          # B: N true-color transitions per color → fire action
        include_unknown=True,
        per_color_thresholds=None,    # dict like {"pink":3, "white":5}
        default_threshold=3           # fallback when a color not in per_color_thresholds
    ):
        self.target_colors = list(target_colors or [])
        self.raw_same_n = int(raw_same_n)
        self.true_transition_n = int(true_transition_n)
        self.include_unknown = bool(include_unknown)
        self.per_color_thresholds = dict(per_color_thresholds or {})
        self.default_threshold = int(default_threshold)

        # Raw stage
        self.prev_raw = None
        self.raw_streak = 0

        # True-color stage
        self.current_true = "unknown"
        self.prev_true = None
        self.true_counts = defaultdict(int)

        # Triggered flags (avoid firing twice)
        self._trig_done = {c: False for c in self.target_colors}

    def _need_for(self, label):
        return int(self.per_color_thresholds.get(label, self.default_threshold))

    def update(self, raw_label):
        """
        Feed one raw label. Returns a dict:
        {
          "accepted_true_changed": bool,  # True when Stage A just accepted a new true color
          "true_color": str,              # Current true color after Stage A
          "raw_streak": int,              # Current raw consecutive streak for raw_label
          "action_label": str|None,       # Color whose action should be fired now (Stage B)
          "true_counts": dict,            # Snapshot of per-color true transition counts
        }
        """
        action_label = None

        # --- Stage A: same-as-previous RAW → count up; otherwise reset ---
        if raw_label == self.prev_raw:
            self.raw_streak += 1
        else:
            self.raw_streak = 1
            self.prev_raw = raw_label

        accepted_true_changed = False
        if self.raw_streak >= max(1, self.raw_same_n):
            # Accept raw as the "true color"
            if raw_label != self.current_true:
                self.prev_true = self.current_true
                self.current_true = raw_label
                accepted_true_changed = True

                # --- Stage B: increment only when previous TRUE color is different ---
                if (self.include_unknown or self.current_true != "unknown"):
                    if self.current_true != self.prev_true:
                        self.true_counts[self.current_true] += 1

                # Evaluate trigger for the new true color (if target & not fired yet)
                if self.current_true in self.target_colors and not self._trig_done.get(self.current_true, False):
                    need = self._need_for(self.current_true if self.current_true in self.per_color_thresholds else self.current_true)
                    # Prefer per-color thresholds if set; fallback to true_transition_n when not provided
                    threshold = self.per_color_thresholds.get(self.current_true, self.true_transition_n)
                    if self.true_counts[self.current_true] >= max(1, int(threshold)):
                        self._trig_done[self.current_true] = True
                        action_label = self.current_true

        # Snapshot of counts for display (convert defaultdict to dict)
        counts_snapshot = dict(self.true_counts)
        return {
            "accepted_true_changed": accepted_true_changed,
            "true_color": self.current_true,
            "raw_streak": self.raw_streak,
            "action_label": action_label,
            "true_counts": counts_snapshot,
        }
