#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
color_learn.py — 4次元（RGBn + Clear）学習スクリプト
- 各ラベルの nRGB (R/C, G/C, B/C) と Clear(C) の平均・標準偏差を color_profiles.json に保存
- 旧フォーマット（3次元）との互換性あり：mu3/std3 も同時保存
使い方（例：同一の it/gain/高さ/速度 条件で各ラベルを個別に学習してください）:
  python3 color_learn.py --label pink  --samples 400 --it 24  --gain 4
  python3 color_learn.py --label green --samples 400 --it 24  --gain 4
  python3 color_learn.py --label white --samples 400 --it 24  --gain 4
  python3 color_learn.py --label gray  --samples 400 --it 24  --gain 4
  # ラベル一覧
  python3 color_learn.py --list
  # ラベル削除
  python3 color_learn.py --clear pink
"""
from __future__ import annotations
import argparse, json, time, sys, math, os
from typing import Dict, Any, List, Tuple

# ---- Sensor import (Raspberry Pi 実機で動作) ----
try:
    import board, busio, adafruit_tcs34725
except Exception:
    board = busio = adafruit_tcs34725 = None

PROFILES_PATH = "color_profiles.json"

def _safe_div(a: float, b: float, eps: float = 1e-9) -> float:
    return a / b if abs(b) > eps else 0.0

def normalize_rgbc(r: float, g: float, b: float, c: float) -> Tuple[float, float, float]:
    rn = _safe_div(r, c)
    gn = _safe_div(g, c)
    bn = _safe_div(b, c)
    return rn, gn, bn

def load_profiles(path: str = PROFILES_PATH) -> Dict[str, Any]:
    if not os.path.exists(path):
        return {"meta": {"format": "rgbn+c_v2", "created": time.time()}, "labels": {}}
    with open(path, "r", encoding="utf-8") as f:
        try:
            return json.load(f)
        except Exception:
            return {"meta": {"format": "rgbn+c_v2", "created": time.time()}, "labels": {}}

def save_profiles(obj: Dict[str, Any], path: str = PROFILES_PATH) -> None:
    obj["meta"]["updated"] = time.time()
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)

def learn_once(label: str, samples: int, it_ms: int, gain: int) -> None:
    if adafruit_tcs34725 is None:
        print("この環境ではセンサーが利用できません。Raspberry Pi 上で実行してください。", file=sys.stderr)
        sys.exit(1)

    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_tcs34725.TCS34725(i2c)

    # 設定（実験条件を固定化すること）
    sensor.integration_time = it_ms
    sensor.gain = gain

    buf_rgbn: List[Tuple[float,float,float]] = []
    buf_c: List[float] = []

    t0 = time.perf_counter()
    for i in range(samples):
        # 一部のライブラリでは .color_raw / .raw / .color_rgb_bytes など実装差があります
        try:
            # adafruit_tcs34725 では .color_raw が (R,G,B,C) を返す実装が多い
            r, g, b, c = sensor.color_raw  # type: ignore
        except Exception:
            # フォールバック：個別属性がある実装向け
            r, g, b = sensor.red, sensor.green, sensor.blue  # type: ignore
            c = getattr(sensor, "clear", r+g+b)

        rn, gn, bn = normalize_rgbc(r, g, b, c)
        buf_rgbn.append((rn, gn, bn))
        buf_c.append(float(c))

        if (i+1) % max(1, samples//10) == 0:
            elapsed = time.perf_counter() - t0
            hz = (i+1) / max(elapsed, 1e-9)
            print(f"[{i+1}/{samples}] ~{hz:.1f} Hz  RGBC=({r},{g},{b},{c})")

    # 統計
    n = max(1, len(buf_rgbn))
    mean3 = [sum(x[j] for x in buf_rgbn)/n for j in range(3)]
    var3  = [sum((x[j]-mean3[j])**2 for x in buf_rgbn)/max(1, (n-1)) for j in range(3)]
    std3  = [max(math.sqrt(var3[j]), 1e-6) for j in range(3)]

    meanC = sum(buf_c)/n
    varC  = sum((x-meanC)**2 for x in buf_c)/max(1, (n-1))
    stdC  = max(math.sqrt(varC), 1e-6)

    # 4次元ベクトル [rn, gn, bn, C] として保存（旧互換のため 3D も保持）
    profiles = load_profiles(PROFILES_PATH)
    labels = profiles.setdefault("labels", {})
    labels[label] = {
        "mu":   [mean3[0], mean3[1], mean3[2], meanC],
        "std":  [std3[0],  std3[1],  std3[2],  stdC ],
        "mu3":  mean3,
        "std3": std3,
        "count": n,
        "it_ms": it_ms,
        "gain": gain,
        "timestamp": time.time(),
    }
    # 記録しておくと便利な参考情報
    profiles["meta"]["format"] = "rgbn+c_v2"
    profiles["meta"]["note"] = "mu/std は [rn,gn,bn,C] の4次元。mu3/std3 は互換用3次元。"

    save_profiles(profiles)
    print(f"✔ 学習完了: {label}  samples={n} it={it_ms}ms gain={gain}")
    print(f"   mu3={mean3}, std3={std3}")
    print(f"   C_mean={meanC:.1f}, C_std={stdC:.1f}")
    print(f"   4D mu={labels[label]['mu']}, 4D std={labels[label]['std']}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", type=str, help="学習するラベル名（例: pink, white, green, gray）")
    ap.add_argument("--samples", type=int, default=300, help="サンプル数")
    ap.add_argument("--it", type=int, default=24, help="integration_time [ms]: 2/24/50/101/154/700 など")
    ap.add_argument("--gain", type=int, default=4, help="gain: 1/4/16/60")
    ap.add_argument("--list", action="store_true", help="保存済みラベル一覧を表示")
    ap.add_argument("--clear", type=str, help="指定ラベルを削除")
    args = ap.parse_args()

    if args.list:
        prof = load_profiles()
        print(json.dumps(prof, ensure_ascii=False, indent=2))
        return

    if args.clear:
        prof = load_profiles()
        if args.clear in prof.get("labels", {}):
            prof["labels"].pop(args.clear, None)
            save_profiles(prof)
            print(f"✔ 削除: {args.clear}")
        else:
            print(f"指定ラベルが見つかりません: {args.clear}")
        return

    if not args.label:
        print("エラー: --label を指定してください（例: --label pink）")
        sys.exit(1)

    learn_once(args.label, args.samples, args.it, args.gain)

if __name__ == "__main__":
    main()
