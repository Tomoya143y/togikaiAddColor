# color_detect_change.py（detect.py と同じ関数/計算 + ラベル変化時のみ出力）
# すべての調整値は config.py を参照（未定義なら安全なデフォルト）

import time
import json
import math
from pathlib import Path
import board
import busio
import adafruit_tcs34725
import config  # ★ 追加：調整値はここから取得

PROFILE_PATH = Path(__file__).with_name("color_profiles.json")

def load_profiles(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    profiles = {}
    for label, entry in data.items():
        mean = entry.get("mean") or entry.get("mu")
        std  = entry.get("std")  or entry.get("sigma")
        if (isinstance(mean, (list, tuple)) and isinstance(std, (list, tuple))
                and len(mean) == 3 and len(std) == 3):
            profiles[label] = {"mean": tuple(mean), "std": tuple(std)}
    return profiles

def normalize_rgb(r, g, b, c):
    # C で正規化（照度影響を緩和）/ C=0 は 1 扱い
    denom = c if c > 0 else 1
    return (r/denom, g/denom, b/denom)

def color_distance(rgbn, mean, std):
    # detect.py と同じ：z スコアのユークリッド距離（sqrt あり）
    z0 = (rgbn[0] - mean[0]) / std[0]
    z1 = (rgbn[1] - mean[1]) / std[1]
    z2 = (rgbn[2] - mean[2]) / std[2]
    return math.sqrt(z0*z0 + z1*z1 + z2*z2)

def compute_distances(rgbn, profiles):
    dists = {}
    for label, p in profiles.items():
        dists[label] = color_distance(rgbn, p["mean"], p["std"])
    return dists

def best_label_from_distances(dists, unknown_thresh):
    label, dmin = min(dists.items(), key=lambda kv: kv[1])
    if dmin > unknown_thresh:
        return "unknown", dmin
    return label, dmin

def format_distances(dists):
    # "pink:12.3 green:45.6 white:7.8" のように簡潔表示
    return " ".join(f"{k}:{v:.1f}" for k, v in dists.items())

def read_raw_rgbc(sensor):
    # adafruit_tcs34725 の版差吸収：color_raw / raw の両対応
    try:
        return sensor.color_raw
    except AttributeError:
        return sensor.raw

def main():
    if not PROFILE_PATH.exists():
        print(f"Profile not found: {PROFILE_PATH}")
        return
    profiles = load_profiles(PROFILE_PATH)
    if not profiles:
        print("No valid profiles in JSON.")
        return

    # ---- config.py から調整値を取得（未定義なら安全なデフォルト）----
    integration_ms = float(getattr(config, "TCS34725_INTEGRATION_MS", 24))
    gain           = int(getattr(config, "TCS34725_GAIN", 4))
    unknown_th     = float(getattr(config, "UNKNOWN_THRESH", 5.0))
    read_interval  = float(getattr(config, "READ_INTERVAL", 0.02))

    # I2C / センサー初期化
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_tcs34725.TCS34725(i2c)
    sensor.integration_time = integration_ms
    sensor.gain = gain

    # print("TCS34725 ready.")
    # print(f"integration_time={sensor.integration_time} ms, gain={sensor.gain}x")
    # print(f"UNKNOWN_THRESH={unknown_th}, READ_INTERVAL={read_interval}")
    # print("Output only when label changes.\n")

    last_label = None

    try:
        while True:
            r, g, b, c = read_raw_rgbc(sensor)
            rgbn = normalize_rgb(r, g, b, c)

            dists = compute_distances(rgbn, profiles)
            label, best_d = best_label_from_distances(dists, unknown_th)

            # ★ ラベルが変わった時だけ出力
            if label != last_label:
                rn, gn, bn = rgbn
                print(
                    f"[{label}] RGBC=({r}, {g}, {b}, {c}) | "
                    f"RGBn=({rn:.3f},{gn:.3f},{bn:.3f}) | "
                    f"d={best_d:.1f} | dist={format_distances(dists)}"
                )
                last_label = label

            time.sleep(read_interval)

    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
