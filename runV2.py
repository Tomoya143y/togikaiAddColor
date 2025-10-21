# coding:utf-8
if __name__ == "__main__":
    # myparam_run.pyで２重にimportされるのを防ぐ
    import config
else:
    # myparam_run.pyから起動したときにmyparam_run.pyのconfigを使う
    import myparam_run
    config = myparam_run.config

# -------------------------
# 標準/外部ライブラリ
# -------------------------
import os
import time
import sys
import json
import numpy as np
from collections import defaultdict
import multiprocessing
from multiprocessing import Process
import RPi.GPIO as GPIO
GPIO.setwarnings(False)

print("ライブラリの初期化に数秒かかります...")

# =========================
# 1) GPIO を「先に BOARD で固定」してからピンをセットアップ
#    → ultrasonic.py / motor.py が BOARD 前提のため
# =========================
try:
    GPIO.cleanup()  # 以前の実行状態を完全リセット
except Exception:
    pass

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)  # ★ ここで最初に BOARD を確定させる
# あなたの config の BOARD 配列をそのまま使う
if hasattr(config, "e_list") and config.e_list:
    GPIO.setup(config.e_list, GPIO.IN)
if hasattr(config, "t_list") and config.t_list:
    GPIO.setup(config.t_list, GPIO.OUT, initial=GPIO.LOW)
print("[GPIO] Mode set to BOARD. e:", getattr(config, "e_list", []), " t:", getattr(config, "t_list", []))

# =========================
# 2) 色センサー（TCS34725）: smbus 直叩き版（Blinka 不使用）
# =========================
COLOR_DET_AVAILABLE = True
tcs_sensor = None
color_det = None
current_color = "unknown"

# --- 最小TCS34725ドライバ（GPIOモードに非依存：I2Cのみ） ---
class _MinimalTCS34725:
    I2C_ADDR = 0x29
    CMD      = 0x80
    REG_ENABLE  = 0x00
    REG_ATIME   = 0x01
    REG_CONTROL = 0x0F
    REG_ID      = 0x12
    REG_STATUS  = 0x13
    REG_CDATA   = 0x14  # CDATAL/CDATAH, R/G/B follow

    # integration time(ms) → ATIME レジスタ値
    _ATIME_LUT = {2.4:0xFF, 24:0xF6, 50:0xEB, 101:0xD5, 154:0xC0, 700:0x00}
    # gain → CONTROL 値
    _GAIN_LUT  = {1:0x00, 4:0x01, 16:0x02, 60:0x03}

    def __init__(self, bus, integ_ms=154, gain=4):
        self.bus = bus
        # 電源ON→ADC有効
        self._write8(self.REG_ENABLE, 0x01)      # PON
        time.sleep(0.003)
        self._write8(self.REG_ENABLE, 0x03)      # PON|AEN
        # 積分時間とゲイン（最も近い値に丸め）
        self.integration_time = self._closest_atime(integ_ms)
        self.gain             = self._closest_gain(gain)

    def _closest_atime(self, integ_ms):
        closest = min(self._ATIME_LUT, key=lambda x: abs(x - float(integ_ms)))
        self._write8(self.REG_ATIME, self._ATIME_LUT[closest])
        return closest

    def _closest_gain(self, gain):
        valid = min(self._GAIN_LUT, key=lambda x: abs(x - int(gain)))
        self._write8(self.REG_CONTROL, self._GAIN_LUT[valid])
        return valid

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.I2C_ADDR, self.CMD | reg, val & 0xFF)

    def _read8(self, reg):
        return self.bus.read_byte_data(self.I2C_ADDR, self.CMD | reg)

    def _read16(self, reg_l):
        lo = self.bus.read_byte_data(self.I2C_ADDR, self.CMD | reg_l)
        hi = self.bus.read_byte_data(self.I2C_ADDR, self.CMD | (reg_l + 1))
        return (hi << 8) | lo

    @property
    def color_raw(self):
        # 積分時間ぶん待機（最小2.4ms）
        time.sleep(max(self.integration_time / 1000.0, 0.003))
        c = self._read16(self.REG_CDATA + 0)  # C
        r = self._read16(self.REG_CDATA + 2)  # R
        g = self._read16(self.REG_CDATA + 4)  # G
        b = self._read16(self.REG_CDATA + 6)  # B
        return (r, g, b, c)

# --- smbus 開始 & ドライバ生成 ---
try:
    try:
        from smbus2 import SMBus
    except Exception:
        from smbus import SMBus  # smbus2 が無い場合のフォールバック

    bus = SMBus(1)
    integ_ms = getattr(config, "TCS34725_INTEGRATION_MS", 24)  # 2.4/24/50/101/154/700
    gain_val = getattr(config, "TCS34725_GAIN", 4)              # 1/4/16/60
    tcs_sensor = _MinimalTCS34725(bus, integ_ms=integ_ms, gain=gain_val)
    print(f"[Color] TCS34725 (smbus) ready. integration_time={tcs_sensor.integration_time} ms, gain={tcs_sensor.gain}x")

    # カラーデテクタ（color_runtime_detector.py はそのまま利用）
    from color_runtime_detector import ColorRuntimeDetector
    color_det = ColorRuntimeDetector(
        sensor=tcs_sensor,
        method="std_weighted",   # または "euclidean"（元スクリプトに合わせて）
        unknown_threshold=5.0,  # 調整可（判定ロジックは外部に委譲）
        read_interval=0.02,
        moving_avg_n=1,
        distance_scale=1.0
    )
    try:
        _rdbg = color_det.read_and_classify()
        print("[Color] Probe:", _rdbg["line"])
    except Exception as e_probe:
        print("[Color] 起動時読み取りエラー:", e_probe)
    print("[Color] ColorRuntimeDetector ready. (smbus only)")

except Exception as e:
    COLOR_DET_AVAILABLE = False
    color_det = None
    print("[Color] センサー初期化に失敗したため色判定を無効化します:", e)

# =========================
# 3) 色1行ログのフォールバック関数（この先のループで使用）
# =========================
def _rgb_line_from_raw(sensor, label_hint="unknown"):
    try:
        r, g, b, c = sensor.color_raw
    except Exception:
        r, g, b, c = 0, 0, 0, 1
    rn = r / max(c, 1); gn = g / max(c, 1); bn = b / max(c, 1)
    return f"[{label_hint}] RGBC=({r}, {g}, {b}, {c}) | RGBn=({rn:.3f},{gn:.3f},{bn:.3f}) | d=n/a | dist=n/a"

# -------------------------
# togikaidrive モジュール
# -------------------------
import ultrasonic
import motor
import planner
if config.HAVE_CONTROLLER: import joystick
if config.HAVE_CAMERA: import camera_multiprocess
if config.HAVE_IMU: import gyro
if config.mode_plan in ["NN","CNN"]:
    from train_pytorch import NeuralNetwork, ConvolutionalNeuralNetwork, load_model

# FPV
if config.fpv:
    data_sh = multiprocessing.sharedctypes.RawArray('i', (2,3))
    import fpv
    server = Process(target=fpv.run, args=data_sh, kwargs={'host': 'localhost', 'port': config.port, 'threaded': True})
    server.start()

# 記録配列
d = np.zeros(config.N_ultrasonics)
d_stack = np.zeros(config.N_ultrasonics+3)
recording = True

# カメラ
if config.HAVE_CAMERA and not config.fpv:
    print("Start taking pictures in ", config.image_dir)
    cam = camera_multiprocess.VideoCaptureWrapper(0)
    print("【 ◎*】Capture started! \\n")

# モータ初期化
motor = motor.Motor()
motor.set_throttle_pwm_duty(config.STOP)
motor.set_steer_pwm_duty(config.NUTRAL)

# 超音波初期化
ultrasonics = {name: ultrasonic.Ultrasonic(name=name) for name in config.ultrasonics_list}
print(" 下記の超音波センサを利用")
print(" ", config.ultrasonics_list)

# プランナ初期化
plan = planner.Planner(config.mode_plan)

# NN/CNN
model = None
if config.mode_plan == "NN":
    model = NeuralNetwork(len(config.ultrasonics_list), 2, config.hidden_dim, config.num_hidden_layers)
    print("\\n保存したモデルをロードします: ", config.model_path)
    load_model(model, config.model_path, None, config.model_dir)
    print(model)

if config.mode_plan == "CNN":
    model = ConvolutionalNeuralNetwork()
    print("\\n保存したモデルをロードします: ", config.model_path)
    load_model(model, config.model_path, None, config.model_dir)
    print(model)

# IMU
if config.HAVE_IMU:
    imu = gyro.BNO055()

# コントローラ
if config.HAVE_CONTROLLER:
    joystick = joystick.Joystick()
    mode = joystick.mode[0]
    print("Starting mode: ", mode)

# 一時停止
print('Enterを押して走行開始!')
input()

# 再初期化（モータ）
motor.set_throttle_pwm_duty(config.STOP)

# 開始時間
start_time = time.time()


# --- 色カウンタ（判定・計算には一切影響しない表示用） ---
# 1) color_profiles.json から学習済みの「色ラベル一覧」を取得（存在しない場合は動的に追加）
color_labels = []
_profiles_path_candidates = [
    "color_profiles.json",
    os.path.join(os.path.dirname(__file__), "color_profiles.json"),
    os.path.join(os.getcwd(), "color_profiles.json"),
]
for _p in _profiles_path_candidates:
    try:
        with open(_p, "r", encoding="utf-8") as f:
            prof = json.load(f)
        if isinstance(prof, dict):
            color_labels = [k for k in prof.keys() if isinstance(prof[k], dict)]
        break
    except Exception:
        pass

# 2) カウンタ本体（未知の色が来たら自動でキー追加する設計）
color_counts = defaultdict(int)
prev_color = None
include_unknown = True  # unknown も数える場合は True（表示だけなので判定ロジックには無関係）

# 3) トリガーフラグ（多重実行防止）
_trig_pink_done = False
_trig_green_done = False
_trig_white_done = False

def _fmt_counts(d, labels, include_unknown=True):
    """
    color_profiles.json に載っているラベルをすべて表示・集計する。
    jsonが読めなかった場合でも、d に出てきたキーはすべて表示する（動的追加）。
    ※ 'unknown' は重複表示を避けるため、常に最後に1回だけ付ける。
    """
    # 1) JSON からの既知ラベル（unknown はここでは除外）
    shown_labels = [x for x in (labels or []) if x != "unknown"]

    # 2) 走行中に新規に現れたラベルを追加（unknown はスキップ）
    for k in d.keys():
        if k == "unknown":
            continue
        if k not in shown_labels:
            shown_labels.append(k)

    # 3) 表示文字列を組み立て（unknown は最後に1回だけ）
    parts = [f"{name}={d.get(name,0)}" for name in shown_labels]
    if include_unknown:
        parts.append(f"unknown={d.get('unknown',0)}")
    total = sum(d.values())
    parts.append(f"total={total}")
    return ", ".join(parts)

def _reload_nn_model_to_planB():
    global model
    # model_name と path を差し替え
    config.model_name = "planB"
    config.model_path = os.path.join(config.model_dir, config.model_name)
    if model is None:
        print("[Model] 現在NN/CNNモデル未使用のため、model_nameのみ切替:", config.model_path)
        return True
    try:
        load_model(model, config.model_path, None, config.model_dir)
        print(f"[Model] planBをロードしました -> {config.model_path}")
        return True
    except Exception as e:
        print(f"[Model] planBのロードに失敗しました: {e}")
        return False

def _switch_mode_to_right_left_3():
    global plan
    config.mode_plan = "Right_Left_3"
    plan = planner.Planner(config.mode_plan)
    print("[Mode] mode_plan を Right_Left_3 に切替えました。")

def _safe_stop_and_exit():
    print("[Stop] green×3 を検知。停止します。")
    motor.set_steer_pwm_duty(config.NUTRAL)
    motor.set_throttle_pwm_duty(config.STOP)
    raise SystemExit  # finally によるクリーンアップへ

# 走行ループ
try:
    while True:
        # --- 認知（超音波） ---
        message = ""
        for i, name in enumerate(config.ultrasonics_list):
            d[i] = ultrasonics[name].measure()
            message += name + ":" + "{:>4}".format(round(ultrasonics[name].distance)) + ", "

        # --- 色判定（必ず1行出すフォールバック付き） ---
        def _rgb_line_from_raw(sensor, label_hint="unknown"):
            try:
                r, g, b, c = sensor.color_raw
            except Exception:
                r, g, b, c = 0, 0, 0, 1
            rn = r / max(c, 1)
            gn = g / max(c, 1)
            bn = b / max(c, 1)
            return f"[{label_hint}] RGBC=({r}, {g}, {b}, {c}) | RGBn=({rn:.3f},{gn:.3f},{bn:.3f}) | d=n/a | dist=n/a"

        color_line_to_print = None
        if color_det is not None:
            try:
                # ★ 判定ロジックには一切触れない：結果のみ受け取る
                color_res = color_det.read_and_classify()
                current_color = color_res["label"]
                color_line_to_print = color_res["line"]
            except Exception:
                current_color = "unknown"
                color_line_to_print = _rgb_line_from_raw(tcs_sensor, "unknown")
        else:
            current_color = "unknown"
            color_line_to_print = _rgb_line_from_raw(tcs_sensor, "unknown")

        # --- 連続同色はカウントしない（表示用のみ、判定ロジック非干渉） ---
        try:
            _cc = current_color
        except NameError:
            _cc = None
        if _cc is not None:
            if _cc != prev_color:
                if include_unknown or _cc != "unknown":
                    color_counts[_cc] += 1
                prev_color = _cc

        # === ★ カラー・トリガー（希望仕様）===============================
        # ・pinkを3回 → mode_plan="Right_Left_3" に切替
        # ・greenを3回 → 停止（安全に停止して終了）
        # ・whiteを3回 → NNの model_name を "planB" に切替（即時ロードを試みる）
        # ================================================================
        if (not _trig_pink_done) and color_counts.get("green", 0) >= 99:
            _trig_pink_done = True
            _switch_mode_to_right_left_3()

        if (not _trig_green_done) and color_counts.get("white", 0) >= 99:
            _trig_green_done = True
            _safe_stop_and_exit()

        if (not _trig_white_done) and color_counts.get("pink", 0) >= 99:
            _trig_white_done = True
            ok = _reload_nn_model_to_planB()
            if not ok:
                print("[Model] planBのロードは失敗。従来モデルを継続利用します。")

        # --- 判断（プランニング） ---
        if config.mode_plan == "GoStraight":
            steer_pwm_duty, throttle_pwm_duty = 0, config.FORWARD_S

        elif config.mode_plan == "Right_Left_3":
            steer_pwm_duty, throttle_pwm_duty = plan.Right_Left_3(
                ultrasonics["FrLH"].distance, ultrasonics["Fr"].distance, ultrasonics["FrRH"].distance)

        elif config.mode_plan == "Right_Left_3_Records":
            steer_pwm_duty, throttle_pwm_duty = plan.Right_Left_3_Records(
                ultrasonics["FrLH"].distance, ultrasonics["Fr"].distance, ultrasonics["FrRH"].distance)

        elif config.mode_plan == "RightHand":
            steer_pwm_duty, throttle_pwm_duty = plan.RightHand(
                ultrasonics["FrRH"].distance, ultrasonics["RrRH"].distance)

        elif config.mode_plan == "LeftHand":
            steer_pwm_duty, throttle_pwm_duty = plan.LeftHand(
                ultrasonics["FrLH"].distance, ultrasonics["RrLH"].distance)

        elif config.mode_plan == "RightHand_PID":
            steer_pwm_duty, throttle_pwm_duty = plan.RightHand_PID(
                ultrasonics["FrRH"], ultrasonics["RrRH"])

        elif config.mode_plan == "LeftHand_PID":
            steer_pwm_duty, throttle_pwm_duty = plan.LeftHand_PID(
                ultrasonics["FrLH"], ultrasonics["RrLH"])

        elif config.mode_plan == "NN":
            args = [ultrasonics[key].distance for key in config.ultrasonics_list]
            steer_pwm_duty, throttle_pwm_duty = plan.NN(model, *args)

        elif config.mode_plan == "CNN":
            _, img = cam.read()
            steer_pwm_duty, throttle_pwm_duty = plan.CNN(model, img)

        else:
            print("デフォルトの判断モードの選択ではありません, コードを書き換えてオリジナルのモードを実装しよう!")
            break

        # --- 操作（ステアリング、アクセル） ---
        if config.HAVE_CONTROLLER:
            joystick.poll()
            mode = joystick.mode[0]
            if mode == "user":
                steer_pwm_duty = int(joystick.steer * config.JOYSTICK_STEERING_SCALE * 100)
                throttle_pwm_duty = int(joystick.accel * config.JOYSTICK_THROTTLE_SCALE * 100)
                if joystick.accel2:
                    throttle_pwm_duty = int(config.FORWARD_S)
                elif joystick.accel1:
                    throttle_pwm_duty = int(config.FORWARD_C)
                # LB/RBボタンで一定角度ステア
                if getattr(joystick, "lb", 0) and not getattr(joystick, "rb", 0):
                    steer_pwm_duty = int(config.LEFT * config.JOYSTICK_STEER_BTN_RATIO)
                elif getattr(joystick, "rb", 0) and not getattr(joystick, "lb", 0):
                    steer_pwm_duty = int(config.RIGHT * config.JOYSTICK_STEER_BTN_RATIO)
                elif getattr(joystick, "lb", 0) and getattr(joystick, "rb", 0):
                    steer_pwm_duty = int(config.NUTRAL)

            elif mode == "auto_str":
                throttle_pwm_duty = int(joystick.accel * config.JOYSTICK_THROTTLE_SCALE * 100)
                if joystick.accel2:
                    throttle_pwm_duty = int(config.FORWARD_S)
                elif joystick.accel1:
                    throttle_pwm_duty = int(config.FORWARD_C)
                if getattr(joystick, "lb", 0) and not getattr(joystick, "rb", 0):
                    steer_pwm_duty = int(config.LEFT * config.JOYSTICK_STEER_BTN_RATIO)
                elif getattr(joystick, "rb", 0) and not getattr(joystick, "lb", 0):
                    steer_pwm_duty = int(config.RIGHT * config.JOYSTICK_STEER_BTN_RATIO)
                elif getattr(joystick, "lb", 0) and getattr(joystick, "rb", 0):
                    steer_pwm_duty = int(config.NUTRAL)

            if joystick.recording:
                recording = True
            else:
                recording = False

            if joystick.breaking:
                motor.breaking()

        # --- モータ出力 ---
        if config.mode_plan == "GCounter" and config.HAVE_IMU:
            imu.GCounter()
            motor.set_steer_pwm_duty(steer_pwm_duty * (1 - 2 * imu.Gstr))
            motor.set_throttle_pwm_duty(throttle_pwm_duty * (1 - 2 * imu.Gthr))
        else:
            motor.set_steer_pwm_duty(steer_pwm_duty)
            motor.set_throttle_pwm_duty(throttle_pwm_duty)

        # --- 記録 ---
        ts = time.time()
        ts_run = round(ts - start_time, 2)
        if recording:
            d_stack = np.vstack((d_stack, np.insert(d, 0, [ts, steer_pwm_duty, throttle_pwm_duty]),))
            if config.HAVE_CAMERA and not config.fpv:
                if not config.mode_plan == "CNN":
                    _, img = cam.read()
                cam.save(img, "%10.2f" % (ts), steer_pwm_duty, throttle_pwm_duty,
                        config.image_dir, config.IMAGE_W, config.IMAGE_H)

        # --- 出力 ---
        if color_line_to_print:
            out_line = f"{color_line_to_print} | count: {_fmt_counts(color_counts, color_labels, include_unknown)}"
            print(out_line)
        if 'mode' in locals() and mode == 'auto':
            mode = config.mode_plan
        if config.plotter:
            print(message)
        else:
            print("Rec:{0}, Mode:{1}, RunTime:{2:>5}, Str:{3:>4}, Thr:{4:>4}, Color:{5}, Uls:[ {6}]".format(
                recording, ('-' if 'mode' not in locals() else mode), ts_run,
                steer_pwm_duty, throttle_pwm_duty, current_color, message))

        # --- リカバリ ---
        if config.mode_recovery == "None":
            pass

        elif config.mode_recovery == "Back" and (('mode' not in locals()) or mode != "user"):
            plan.Back(ultrasonics["Fr"], ultrasonics["FrRH"], ultrasonics["FrLH"])
            if plan.flag_back is True:
                for _ in range(config.recovery_braking):
                    motor.set_steer_pwm_duty(config.NUTRAL)
                    motor.set_throttle_pwm_duty(config.REVERSE)
                    time.sleep(config.recovery_time)

        elif config.mode_recovery == "Stop" and (('mode' not in locals()) or mode != "user"):
            plan.Stop(ultrasonics["Fr"])
            if plan.flag_stop is True:
                motor.set_steer_pwm_duty(config.recovery_str)
                for _ in range(config.recovery_braking):
                    motor.set_throttle_pwm_duty(config.STOP)
                    time.sleep(0.02)
                    motor.set_throttle_pwm_duty(config.REVERSE)
                    time.sleep(0.1)
                    time.sleep(config.recovery_time / config.recovery_braking)
                motor.set_throttle_pwm_duty(config.STOP)
                plan.flag_stop = False
                print("一時停止、Enterを押して走行再開!")
                input()

finally:
    # 終了処理
    print('\\n停止')
    try:
        motor.set_throttle_pwm_duty(config.STOP)
        motor.set_steer_pwm_duty(config.NUTRAL)
    except Exception:
        pass
    GPIO.cleanup()
    header = "Tstamp,Str,Thr,"
    for name in config.ultrasonics_list:
        header += name + ","
    header = header[:-1]
    try:
        np.savetxt(config.record_filename, d_stack[1:], delimiter=',', fmt='%10.2f', header=header, comments="")
        print('記録停止')
        print("記録保存--> ", config.record_filename)
    except Exception as e:
        print("記録保存に失敗:", e)
    if getattr(config, "HAVE_CAMERA", False):
        print("画像保存--> ", getattr(config, "image_dir", "(未設定)"))

header = "Tstamp, Str, Thr, "
for name in config.ultrasonics_list:
    header += name + ", "
header = header[:-1]
