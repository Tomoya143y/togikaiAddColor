# -*- coding: utf-8 -*-
"""
color_runtime_detector.py — 4次元（RGBn + Clear）対応ランタイム分類器
- color_profiles.json の mu/std が 4次元なら 4D の z-score 距離、3次元のみなら従来通り 3D。
- 端末表示は既存形式互換: [pink] RGBC=(r,g,b,c) | RGBn=(r,g,b) | d=.. | dist={...}
- 既存コードとの親和性を重視し、APIは ColorRuntimeDetector(...).read_and_classify() のみでOK。
"""
from __future__ import annotations
import json, math, time, os
from typing import Dict, Any, Tuple, List, Optional

# ---- Sensor import (Raspberry Pi) ----
try:
    import board, busio, adafruit_tcs34725
except Exception:
    board = busio = adafruit_tcs34725 = None

def _safe_div(a: float, b: float, eps: float = 1e-9) -> float:
    return a / b if abs(b) > eps else 0.0

def _normalize_rgbc(r: float, g: float, b: float, c: float) -> Tuple[float,float,float]:
    return _safe_div(r,c), _safe_div(g,c), _safe_div(b,c)

class ColorRuntimeDetector:
    def __init__(self,
                 profiles_path: str = "color_profiles.json",
                 method: str = "std_weighted",          # "std_weighted" | "euclidean"
                 unknown_threshold: float = 35.0,        # d がこの値を超えたら unknown
                 read_interval: float = 0.0,             # 秒。0なら毎回読む
                 moving_avg_n: int = 1,                  # nRGB の移動平均サンプル数
                 distance_scale: float = 1.0,            # 表示のdを既存値に近づけたい時に調整
                 i2c=None):
        self.method = method
        self.unknown_threshold = float(unknown_threshold)
        self.read_interval = float(read_interval)
        self.moving_avg_n = int(moving_avg_n)
        self.distance_scale = float(distance_scale)

        self._hist_r: List[float] = []
        self._hist_g: List[float] = []
        self._hist_b: List[float] = []
        self._last_ts = 0.0

        self.profiles_path = profiles_path
        self.labels, self.mu, self.std = self._load_profiles(profiles_path)

        # Sensor init
        if adafruit_tcs34725 is not None:
            if i2c is None and busio is not None and board is not None:
                i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_tcs34725.TCS34725(i2c) if i2c else None
        else:
            self.sensor = None

    # ---------- Profiles ----------
    def _load_profiles(self, path: str) -> Tuple[List[str], Dict[str, List[float]], Dict[str, List[float]]]:
        if not os.path.exists(path):
            raise FileNotFoundError(f"color profiles not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            prof = json.load(f)

        labels = []
        mu: Dict[str, List[float]] = {}
        std: Dict[str, List[float]] = {}

        labdict = prof.get("labels", prof)  # 後方互換（古い単純辞書形式もサポート）
        for lab, rec in labdict.items():
            # 優先: 4D (mu/std 長さ4), 次点: 3D (mu3/std3 or mean/std)
            arr_mu = rec.get("mu") or rec.get("mean") or rec.get("mu3")
            arr_std = rec.get("std") or rec.get("sigma") or rec.get("std3")
            if not arr_mu or not arr_std:
                continue
            # 長さ3なら3D、4なら4D
            if len(arr_mu) == 3:
                mu[lab]  = [float(arr_mu[0]), float(arr_mu[1]), float(arr_mu[2])]
                std[lab] = [max(float(arr_std[0]),1e-6), max(float(arr_std[1]),1e-6), max(float(arr_std[2]),1e-6)]
            else:
                mu[lab]  = [float(arr_mu[0]), float(arr_mu[1]), float(arr_mu[2]), float(arr_mu[3])]
                std[lab] = [max(float(arr_std[0]),1e-6), max(float(arr_std[1]),1e-6), max(float(arr_std[2]),1e-6), max(float(arr_std[3]),1e-6)]
            labels.append(lab)

        if not labels:
            raise RuntimeError("No valid labels found in profiles.")
        return labels, mu, std

    # ---------- Moving Average ----------
    def _push_ma(self, rn: float, gn: float, bn: float) -> Tuple[float,float,float]:
        if self.moving_avg_n <= 1:
            return rn, gn, bn
        self._hist_r.append(rn); self._hist_g.append(gn); self._hist_b.append(bn)
        if len(self._hist_r) > self.moving_avg_n:
            self._hist_r.pop(0); self._hist_g.pop(0); self._hist_b.pop(0)
        return sum(self._hist_r)/len(self._hist_r), sum(self._hist_g)/len(self._hist_g), sum(self._hist_b)/len(self._hist_b)

    # ---------- Distance ----------
    def _dist_euclidean(self, v: List[float], mu: List[float]) -> float:
        s = 0.0
        for i in range(min(len(v), len(mu))):
            d = v[i] - mu[i]
            s += d*d
        return math.sqrt(s) * self.distance_scale

    def _dist_std_weighted(self, v: List[float], mu: List[float], std: List[float]) -> float:
        s = 0.0
        for i in range(min(len(v), len(mu), len(std))):
            z = (v[i]-mu[i]) / max(std[i], 1e-6)
            s += z*z
        return math.sqrt(s) * self.distance_scale

    # ---------- Read & Classify ----------
    def read_and_classify(self) -> Dict[str, Any]:
        now = time.perf_counter()
        if self.read_interval > 0 and (now - self._last_ts) < self.read_interval:
            time.sleep(self.read_interval - (now - self._last_ts))
        self._last_ts = time.perf_counter()

        if self.sensor is None:
            raise RuntimeError("TCS34725 is not available in this environment.")

        # RGBC 取得
        try:
            r, g, b, c = self.sensor.color_raw  # type: ignore
        except Exception:
            r, g, b = self.sensor.red, self.sensor.green, self.sensor.blue  # type: ignore
            c = getattr(self.sensor, "clear", r+g+b)

        rn, gn, bn = _normalize_rgbc(r, g, b, c)
        rn, gn, bn = self._push_ma(rn, gn, bn)

        # ベクトル v: 4D or 3D をラベルごとに合わせる
        # - プロファイルが4Dなら [rn,gn,bn,C] を使用、3Dなら [rn,gn,bn]。
        distances: Dict[str, float] = {}
        best_label: Optional[str] = None
        best_d = float("inf")

        for lab in self.labels:
            mu = self.mu[lab]
            std = self.std[lab]
            if len(mu) == 4:
                v = [rn, gn, bn, float(c)]
            else:
                v = [rn, gn, bn]

            if self.method == "euclidean":
                d = self._dist_euclidean(v, mu)
            else:
                d = self._dist_std_weighted(v, mu, std)

            distances[lab] = d
            if d < best_d:
                best_d, best_label = d, lab

        label = "unknown" if best_d > self.unknown_threshold else (best_label or "unknown")

        # 表示行（従来互換）
        dist_str = " ".join([f"{k}:{distances[k]:.1f}" for k in sorted(distances, key=distances.get)])
        line = f"[{label}] RGBC=({int(r)},{int(g)},{int(b)},{int(c)}) | RGBn=({rn:.3f},{gn:.3f},{bn:.3f}) | d={best_d:.1f} | dist={{ {dist_str} }}"

        return {
            "label": label,
            "d": best_d,
            "distances": distances,
            "rgbc": (int(r), int(g), int(b), int(c)),
            "rgbn": (rn, gn, bn),
            "line": line,
        }

# 使い方（例）:
# from color_runtime_detector import ColorRuntimeDetector
# det = ColorRuntimeDetector(profiles_path="color_profiles.json",
#                            method="std_weighted",
#                            unknown_threshold=35.0,
#                            read_interval=0.02,
#                            moving_avg_n=1,
#                            distance_scale=1.0)
# while True:
#     res = det.read_and_classify()
#     print(res["line"])
