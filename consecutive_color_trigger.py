# coding:utf-8
from typing import Callable, Dict, Optional, Tuple
import sys

try:
    import myparam_run  # myparam_run.py から起動する場合の config 参照
    config = myparam_run.config
except Exception:
    import config  # 通常実行時

class ConsecutiveColorTrigger:
    """
    同じ色を閾値回『連続』して観測したら、その色に紐づくアクションを実行するトリガ。
    """
    def __init__(
        self,
        threshold: int = None,
        actions: Dict[str, Callable] = None,
        enable: bool = None,
        ignore_labels: Optional[set] = None,
        verbose: bool = None,
    ) -> None:
        self.threshold = threshold if threshold is not None else getattr(config, "COLOR_CONSECUTIVE_THRESHOLD", 10)
        self.actions = actions if actions is not None else getattr(config, "COLOR_TRIGGER_ACTIONS", {})
        self.enable = getattr(config, "ENABLE_COLOR_TRIGGER", True) if enable is None else enable
        self.ignore_labels = ignore_labels if ignore_labels is not None else set(getattr(config, "COLOR_IGNORE_LABELS", {"unknown"}))
        self.verbose = getattr(config, "PRINT_COLOR_TRIGGER_LOG", True) if verbose is None else verbose

        # 連続カウント用の状態
        self._last_color: Optional[str] = None
        self._streak: int = 0
        # 同色の連続トリガ多重発火を抑止するための状態
        self._cooldown_color: Optional[str] = None

        if self.verbose:
            print(f"[ColorTrigger] enable={self.enable}, threshold={self.threshold}, ignore={self.ignore_labels}", file=sys.stderr)

    def reset(self) -> None:
        """外部から全状態をリセットしたい場合に使用。"""
        self._last_color = None
        self._streak = 0
        self._cooldown_color = None

    def update(self, color_label: Optional[str]) -> Tuple[bool, Optional[str]]:
        """
        ループ毎に現在の色ラベルを渡すと、必要に応じてアクションを実行。
        Returns:
            fired (bool): この呼び出しで新規にアクションが発火したか
            fired_color (Optional[str]): 発火した色（なければ None）
        """
        if not self.enable:
            return False, None

        if color_label is None:
            # センサー未読などは無視してステートは維持
            return False, None

        # 無視対象（例: "unknown"）は streak をリセット
        if color_label in self.ignore_labels:
            if self.verbose and self._streak:
                print(f"[ColorTrigger] reset by ignore label: {color_label} (streak was {self._streak})", file=sys.stderr)
            self._last_color = None
            self._streak = 0
            # ignore は cooldown 解除もしがちだが、誤検知ノイズ抑制のため保持
            return False, None

        # 連続カウント更新
        if color_label == self._last_color:
            self._streak += 1
        else:
            self._last_color = color_label
            self._streak = 1

        if self.verbose:
            print(f"[ColorTrigger] color={color_label}, streak={self._streak}/{self.threshold}, cooldown={self._cooldown_color}", file=sys.stderr)

        # クールダウン中（同じ色が続く限り再発火しない）
        if self._cooldown_color == color_label:
            return False, None

        # 閾値到達で発火
        if self._streak >= self.threshold:
            action = self.actions.get(color_label)
            if callable(action):
                try:
                    if self.verbose:
                        print(f"[ColorTrigger] FIRE action for '{color_label}'", file=sys.stderr)
                    action()
                except Exception as e:
                    print(f"[ColorTrigger] Action for '{color_label}' raised: {e}", file=sys.stderr)
            else:
                if self.verbose:
                    print(f"[ColorTrigger] No action bound for '{color_label}'", file=sys.stderr)

            # 多重発火防止：同色のままは抑止、色が変わったら解除
            self._cooldown_color = color_label
            # 次の確定までのカウントは継続してもいいが、誤動作防止でリセット
            self._streak = 0
            self._last_color = None
            return True, color_label

        return False, None
