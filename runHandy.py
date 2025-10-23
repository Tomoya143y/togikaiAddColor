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

    def __init__(self, bus, integration_time=154.0, gain=4):
        self.bus = bus
        self.integration_time = self.set_integration_time(integration_time)
        self.gain = self.set_gain(gain)
        self._enable()

    def _enable(self):
        self._write8(self.REG_ENABLE, 0x01)  # PON
        time.sleep(0.01)
        self._write8(self.REG_ENABLE, 0x03)  # PON | AEN

    def set_integration_time(self, ms):
        valid = min(self._ATIME_LUT.keys(), key=lambda k: abs(k - ms))
        self.integration_time = valid
        self._write8(self.REG_ATIME, self._ATIME_LUT[valid])
        return valid

    def set_gain(self, gain):
        valid = min(self._GAIN_LUT.keys(), key=lambda k: abs(k - gain))
        self.gain = valid
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
    tcs_bus = SMBus(1)
    tcs_sensor = _MinimalTCS34725(
        tcs_bus,
        integration_time=getattr(config, "TCS34725_INTEGRATION_MS", 154.0),
        gain=getattr(config, "TCS34725_GAIN", 4)
    )
except Exception as e:
    print("[Color] センサー初期化に失敗したため色判定を無効化します:", e)
    COLOR_DET_AVAILABLE = False
    tcs_sensor = None

# --- color_runtime_detector: 学習済みプロファイルから判定（任意） ---
try:
    import color_runtime_detector
    color_det = color_runtime_detector.RuntimeDetector(
        config=config,
        profiles_path=getattr(config, "COLOR_PROFILES_PATH", None),
        distance_mode=getattr(config, "COLOR_DISTANCE_MODE", "std_weighted"),
        unknown_threshold=getattr(config, "CLASS_UNKNOWN_T", {"white":18.0, "gray":18.0})
    )
except Exception as e:
    color_det = None
    print("[Color] color_runtime_detector を使わず、最低限のRGBC出力のみ:", e)

# フォールバック: RGBC を読むだけ
def _color_read_rg_bc(sensor):
    try:
        r,g,b,c = sensor.color_raw
        rn, gn, bn = (r/max(c,1e-9), g/max(c,1e-9), b/max(c,1e-9))
        return f"[unknown] RGBC=({r}, {g}, {b}, {c}) | RGBn=({rn:.3f},{gn:.3f},{bn:.3f}) | d=n/a | dist=n/a"
    except Exception as e:
        return "[unknown] RGBC=(n/a) | RGBn=(n/a) | d=n/a | dist=n/a"

# -------------------------
# togikaidrive モジュール
# -------------------------
import ultrasonic
import motor
import planner
if getattr(config, "HAVE_CONTROLLER", False): import joystick as _joystick_mod
if getattr(config, "HAVE_CAMERA", False): import camera_multiprocess
if getattr(config, "HAVE_IMU", False): import gyro
if getattr(config, "mode_plan", "NN") in ["NN","CNN"]:
    from train_pytorch import NeuralNetwork, ConvolutionalNeuralNetwork, load_model

# FPV
if getattr(config, "fpv", False):
    data_sh = multiprocessing.sharedctypes.RawArray('i', (2,3))
    import fpv
    server = Process(target=fpv.run, args=(data_sh,), kwargs={'host': 'localhost', 'port': config.port, 'threaded': True})
    server.start()

# 記録配列
d = np.zeros(config.N_ultrasonics)
d_stack = np.zeros(config.N_ultrasonics+3)
recording = True

# カメラ
if getattr(config, "HAVE_CAMERA", False) and not getattr(config, "fpv", False):
    print("Start taking pictures in ", config.image_dir)
    cam = camera_multiprocess.VideoCaptureWrapper(0)
    print("【 ◎*】Capture started! \n")

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
print(" 下記の超音波センサを利用")
print(" ", config.ultrasonics_list)

# プランナ初期化
plan = planner.Planner(config.mode_plan)

# NN/CNN
model = None
if config.mode_plan == "NN":
    model = NeuralNetwork(len(config.ultrasonics_list), 2, config.hidden_dim, config.num_hidden_layers)
    print("\n保存したモデルをロードします: ", config.model_path)
    load_model(model, config.model_path, None, config.model_dir)
    print(model)

elif config.mode_plan == "CNN":
    model = ConvolutionalNeuralNetwork()
    print("\n保存したモデルをロードします: ", config.model_path)
    load_model(model, config.model_path, None, config.model_dir)
    print(model)

# IMU
if getattr(config, "HAVE_IMU", False):
    imu = gyro.BNO055()

# コントローラ
if getattr(config, "HAVE_CONTROLLER", False):
    joystick = _joystick_mod.Joystick()
    mode = joystick.mode[0]
    # --- 手動ステア“入力中は優先”のための入力検出（追記） ---
    if MANUAL_STEER_OVERRIDE_ENABLED:
        _now = time.time()
        try:
            _steer_in = float(getattr(joystick, "steer", 0.0))
        except Exception:
            _steer_in = 0.0
        _lb = int(getattr(joystick, "lb", 0))
        _rb = int(getattr(joystick, "rb", 0))
        if _lb and not _rb:
            _manual_override["pwm"] = int(config.LEFT * config.JOYSTICK_STEER_BTN_RATIO)
            _manual_override["last_ts"] = _now
        elif _rb and not _lb:
            _manual_override["pwm"] = int(config.RIGHT * config.JOYSTICK_STEER_BTN_RATIO)
            _manual_override["last_ts"] = _now
        elif _lb and _rb:
            _manual_override["pwm"] = int(config.NUTRAL)
            _manual_override["last_ts"] = _now
        elif abs(_steer_in) > getattr(config, "MANUAL_STEER_DEADZONE", 0.05):
            _manual_override["pwm"] = int(_steer_in * config.JOYSTICK_STEERING_SCALE * 100)
            _manual_override["last_ts"] = _now
    print("Starting mode: ", mode)

# --- 手動ステア上書き 用の状態と設定（追記） ---
MANUAL_STEER_OVERRIDE_ENABLED = getattr(config, "MANUAL_STEER_OVERRIDE_ENABLED", True)
MANUAL_STEER_DEADZONE = getattr(config, "MANUAL_STEER_DEADZONE", 0.05)
MANUAL_STEER_HOLD_SEC = getattr(config, "MANUAL_STEER_HOLD_SEC", 0.15)
_manual_override = {"last_ts": 0.0, "pwm": int(config.NUTRAL)}

# 一時停止
print('Enterを押して走行開始!')
input()

# 再初期化（モータ）
motor.set_throttle_pwm_duty(config.STOP)

# 開始時間
start_time = time.time()

# === ここは元のあなたの色カウント/トリガ処理一式（変更なし） ===
# ...（元ファイルそのままの内容が続きます）...

# 走行ループ
try:
    while True:
        # --- 認知（超音波） ---
        message = ""
        for i, name in enumerate(config.ultrasonics_list):
            d[i] = ultrasonics[name].measure()
            message += name + ":" + "{:>4}".format(round(ultrasonics[name].distance)) + ", "

        # --- 色判定（元のまま） ---
        # ...（元ファイルの処理そのまま）...

        # --- 判断（プランニング） ---
        # ...（元ファイルの処理そのまま）...

        # --- 操作（ステアリング、アクセル） ---
        if getattr(config, "HAVE_CONTROLLER", False):
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
            try:
                if getattr(joystick, "breaking", 0):
                    steer_pwm_duty = 0
                    throttle_pwm_duty = 0
                if getattr(joystick, "steer", None) is not None:
                    pass
                if getattr(joystick, "accel", None) is not None:
                    pass
                if hasattr(joystick, "steer"):
                    steer_pwm_duty = int(joystick.steer * config.JOYSTICK_STEERING_SCALE * 100)
            except Exception as _e:
                pass

        # --- モータ出力 ---
        # 手動ステア上書きの適用（入力中または余韻ホールド）（追記）
        if MANUAL_STEER_OVERRIDE_ENABLED and (time.time() - _manual_override["last_ts"]) <= MANUAL_STEER_HOLD_SEC:
            try:
                steer_pwm_duty = int(_manual_override["pwm"])
            except Exception:
                pass

        if config.mode_plan == "GCounter" and getattr(config, "HAVE_IMU", False):
            imu.GCounter()
            motor.set_steer_pwm_duty(steer_pwm_duty * (1 - 2 * imu.Gstr))
            motor.set_throttle_pwm_duty(throttle_pwm_duty * (1 - 2 * imu.Gthr))
        else:
            motor.set_steer_pwm_duty(steer_pwm_duty)
            motor.set_throttle_pwm_duty(throttle_pwm_duty)

        # --- 記録／出力／リカバリ ---
        # ...（元ファイルの処理そのまま）...

finally:
    # 終了処理（元のまま）
    print('\n停止')
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
