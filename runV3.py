
# coding:utf-8
if __name__ == "__main__":
    import config
else:
    import myparam_run
    config = myparam_run.config

import os, time, sys, json, numpy as np
from collections import defaultdict
import multiprocessing
from multiprocessing import Process
import RPi.GPIO as GPIO
GPIO.setwarnings(False)

from color_trigger_v2 import ColorTriggerFSM

print("ライブラリの初期化に数秒かかります...")

try:
    GPIO.cleanup()
except Exception:
    pass
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
if hasattr(config, "e_list") and config.e_list:
    GPIO.setup(config.e_list, GPIO.IN)
if hasattr(config, "t_list") and config.t_list:
    GPIO.setup(config.t_list, GPIO.OUT, initial=GPIO.LOW)
print("[GPIO] Mode set to BOARD. e:", getattr(config, "e_list", []), " t:", getattr(config, "t_list", []))

COLOR_DET_AVAILABLE = True
tcs_sensor = None
color_det = None
current_color = "unknown"

class _MinimalTCS34725:
    I2C_ADDR = 0x29
    CMD      = 0x80
    REG_ENABLE  = 0x00
    REG_ATIME   = 0x01
    REG_CONTROL = 0x0F
    REG_ID      = 0x12
    REG_STATUS  = 0x13
    REG_CDATA   = 0x14
    _ATIME_LUT = {2.4:0xFF, 24:0xF6, 50:0xEB, 101:0xD5, 154:0xC0, 700:0x00}
    _GAIN_LUT  = {1:0x00, 4:0x01, 16:0x02, 60:0x03}
    def __init__(self, bus, integ_ms=154, gain=4):
        self.bus = bus
        self._write8(self.REG_ENABLE, 0x01); time.sleep(0.003)
        self._write8(self.REG_ENABLE, 0x03)
        self.integration_time = self._closest_atime(integ_ms)
        self.gain             = self._closest_gain(gain)
    def _closest_atime(self, integ_ms):
        closest = min(self._ATIME_LUT, key=lambda x: abs(x - float(integ_ms)))
        self._write8(self.REG_ATIME, self._ATIME_LUT[closest]); return closest
    def _closest_gain(self, gain):
        valid = min(self._GAIN_LUT, key=lambda x: abs(x - int(gain)))
        self._write8(self.REG_CONTROL, self._GAIN_LUT[valid]); return valid
    def _write8(self, reg, val): self.bus.write_byte_data(self.I2C_ADDR, self.CMD | reg, val & 0xFF)
    def _read8(self, reg): return self.bus.read_byte_data(self.I2C_ADDR, self.CMD | reg)
    def _read16(self, reg_l):
        lo = self.bus.read_byte_data(self.I2C_ADDR, self.CMD | reg_l)
        hi = self.bus.read_byte_data(self.I2C_ADDR, self.CMD | (reg_l + 1))
        return (hi << 8) | lo
    @property
    def color_raw(self):
        time.sleep(max(self.integration_time / 1000.0, 0.003))
        c = self._read16(self.REG_CDATA + 0); r = self._read16(self.REG_CDATA + 2)
        g = self._read16(self.REG_CDATA + 4); b = self._read16(self.REG_CDATA + 6)
        return (r, g, b, c)

try:
    try:
        from smbus2 import SMBus
    except Exception:
        from smbus import SMBus
    bus = SMBus(1)
    integ_ms = getattr(config, "TCS34725_INTEGRATION_MS", 24)
    gain_val = getattr(config, "TCS34725_GAIN", 4)
    tcs_sensor = _MinimalTCS34725(bus, integ_ms=integ_ms, gain=gain_val)
    print(f"[Color] TCS34725 ready. integration_time={tcs_sensor.integration_time} ms, gain={tcs_sensor.gain}x")
    from color_runtime_detector import ColorRuntimeDetector
    unknown_th    = float(getattr(config, "UNKNOWN_THRESHOLD", getattr(config, "UNKNOWN_THRESH", 30.0)))
    read_interval = float(getattr(config, "READ_INTERVAL", 0.02))
    moving_avg_n  = int(getattr(config, "COLOR_MOVING_AVG_N", getattr(config, "MOVING_AVG_N", 1)))
    dist_scale    = float(getattr(config, "COLOR_DISTANCE_SCALE", getattr(config, "DISTANCE_SCALE", 1.0)))
    _base_dir = os.path.dirname(os.path.abspath(__file__))
    profiles_path = getattr(config, "COLOR_PROFILES_PATH", os.path.join(_base_dir, "color_profiles.json"))
    color_det = ColorRuntimeDetector(
        profiles_path=profiles_path, method="std_weighted",
        unknown_threshold=unknown_th, read_interval=read_interval,
        moving_avg_n=moving_avg_n, distance_scale=dist_scale
    )
    color_det.sensor = tcs_sensor
    try:
        _rdbg = color_det.read_and_classify(); print("[Color] Probe:", _rdbg["line"])
    except Exception as e_probe:
        print("[Color] 起動時読み取りエラー:", e_probe)
    print("[Color] ColorRuntimeDetector ready. profiles:", profiles_path)
    print("[Color] Labels:", getattr(color_det, "labels", []))
except Exception as e:
    COLOR_DET_AVAILABLE = False; color_det = None
    print("[Color] センサー初期化に失敗したため色判定を無効化します:", e)

def _rgb_line_from_raw(sensor, label_hint="unknown"):
    try: r, g, b, c = sensor.color_raw
    except Exception: r, g, b, c = 0,0,0,1
    rn = r/max(c,1); gn = g/max(c,1); bn = b/max(c,1)
    return f"[{label_hint}] RGBC=({r}, {g}, {b}, {c}) | RGBn=({rn:.3f},{gn:.3f},{bn:.3f}) | d=n/a | dist=n/a"

import ultrasonic, motor, planner
if getattr(config, "HAVE_CONTROLLER", False): import joystick as _joystick_mod
if getattr(config, "HAVE_CAMERA", False): import camera_multiprocess
if getattr(config, "HAVE_IMU", False): import gyro
if getattr(config, "mode_plan", "NN") in ["NN","CNN"]:
    from train_pytorch import NeuralNetwork, ConvolutionalNeuralNetwork, load_model

if getattr(config, "fpv", False):
    data_sh = multiprocessing.sharedctypes.RawArray('i', (2,3)); import fpv
    server = Process(target=fpv.run, args=(data_sh,), kwargs={'host':'localhost','port':config.port,'threaded':True}); server.start()

d = np.zeros(config.N_ultrasonics)
d_stack = np.zeros(config.N_ultrasonics+3)
recording = True

if getattr(config, "HAVE_CAMERA", False) and not getattr(config, "fpv", False):
    print("Start taking pictures in ", config.image_dir)
    cam = camera_multiprocess.VideoCaptureWrapper(0)
    print("【 ◎*】Capture started! \n")

motor = motor.Motor()
motor.set_throttle_pwm_duty(config.STOP)
motor.set_steer_pwm_duty(config.NUTRAL)

ultrasonics = {name: ultrasonic.Ultrasonic(name=name) for name in config.ultrasonics_list}
print(" 下記の超音波センサを利用"); print(" ", config.ultrasonics_list)

plan = planner.Planner(config.mode_plan)

model = None
if config.mode_plan == "NN":
    model = NeuralNetwork(len(config.ultrasonics_list), 2, config.hidden_dim, config.num_hidden_layers)
    print("\n保存したモデルをロードします: ", config.model_path); load_model(model, config.model_path, None, config.model_dir); print(model)
if config.mode_plan == "CNN":
    model = ConvolutionalNeuralNetwork()
    print("\n保存したモデルをロードします: ", config.model_path); load_model(model, config.model_path, None, config.model_dir); print(model)

if getattr(config, "HAVE_IMU", False): imu = gyro.BNO055()

if getattr(config, "HAVE_CONTROLLER", False):
    joystick = _joystick_mod.Joystick(); mode = joystick.mode[0]; print("Starting mode: ", mode)

print('Enterを押して走行開始!'); input()
motor.set_throttle_pwm_duty(config.STOP)

start_time = time.time()

color_labels = []
for _p in [getattr(config,"COLOR_PROFILES_PATH",""),"color_profiles.json",
           os.path.join(os.path.dirname(__file__),"color_profiles.json"),
           os.path.join(os.getcwd(),"color_profiles.json")]:
    if not _p: continue
    try:
        with open(_p,"r",encoding="utf-8") as f: prof = json.load(f)
        if isinstance(prof,dict):
            if "labels" in prof and isinstance(prof["labels"],dict): color_labels = list(prof["labels"].keys())
            else: color_labels = [k for k in prof.keys() if isinstance(prof[k],dict)]
        break
    except Exception: pass

COLOR_TRIGGER_ENABLED = getattr(config, "COLOR_TRIGGER_ENABLED", True)
COLOR_TRIGGER_COLORS = getattr(config, "COLOR_TRIGGER_COLORS", ["green"])
COLOR_TRIGGER_THRESHOLD = getattr(config, "COLOR_TRIGGER_THRESHOLD", 3)
COLOR_TRIGGER_THRESHOLDS = getattr(config, "COLOR_TRIGGER_THRESHOLDS", {})
COLOR_COUNT_INCLUDE_UNKNOWN = getattr(config, "COLOR_COUNT_INCLUDE_UNKNOWN", True)
COLOR_RAW_SAME_N  = getattr(config, "COLOR_RAW_SAME_N", 3)
COLOR_TRUE_SAME_N = getattr(config, "COLOR_TRUE_SAME_N", 3)
COLOR_TRIGGER_MILESTONES = getattr(config, "COLOR_TRIGGER_MILESTONES", {})
COLOR_RESET_AFTER_THRESHOLD = getattr(config, "COLOR_RESET_AFTER_THRESHOLD", True)

# ★ デフォルトを要求通りに変更：green=100, white=60、userモードでも上書き
THROTTLE_OVERRIDE_TRUE_COLOR = getattr(config, "THROTTLE_OVERRIDE_TRUE_COLOR",
                                       {"green": 100, "white": 60})
THROTTLE_OVERRIDE_APPLY_IN_USER_MODE = getattr(config, "THROTTLE_OVERRIDE_APPLY_IN_USER_MODE", True)

def _fmt_counts(d, labels, include_unknown=True):
    shown_labels = [x for x in (labels or []) if x != "unknown"]
    for k in d.keys():
        if k == "unknown": continue
        if k not in shown_labels: shown_labels.append(k)
    parts = [f"{name}={d.get(name,0)}" for name in shown_labels]
    if include_unknown: parts.append(f"unknown={d.get('unknown',0)}")
    total = sum(d.values()); parts.append(f"total={total}")
    return ", ".join(parts)

_color_fsm = ColorTriggerFSM(
    target_colors=COLOR_TRIGGER_COLORS,
    raw_same_n=COLOR_RAW_SAME_N,
    true_transition_n=COLOR_TRUE_SAME_N,
    include_unknown=COLOR_COUNT_INCLUDE_UNKNOWN,
    per_color_thresholds=COLOR_TRIGGER_THRESHOLDS,
    default_threshold=COLOR_TRIGGER_THRESHOLD,
)

try:
    while True:
        message = ""
        for i, name in enumerate(config.ultrasonics_list):
            d[i] = ultrasonics[name].measure()
            message += name + ":" + "{:>4}".format(round(ultrasonics[name].distance)) + ", "

        color_line_to_print = None
        if color_det is not None:
            try:
                color_res = color_det.read_and_classify()
                current_color = color_res["label"]
                color_line_to_print = color_res["line"]
            except Exception:
                current_color = "unknown"
                color_line_to_print = _rgb_line_from_raw(tcs_sensor, "unknown")
        else:
            current_color = "unknown"
            color_line_to_print = _rgb_line_from_raw(tcs_sensor, "unknown")

        fsm_info   = _color_fsm.update(current_color)
        true_color = fsm_info["true_color"]
        counts     = fsm_info["true_counts"]

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
            _, img = cam.read(); steer_pwm_duty, throttle_pwm_duty = plan.CNN(model, img)
        else:
            print("デフォルトの判断モードの選択ではありません, コードを書き換えてオリジナルのモードを実装しよう!"); break

        # --- 操作（ステアリング、アクセル） ---
        if getattr(config, "HAVE_CONTROLLER", False):
            joystick.poll(); mode = joystick.mode[0]
            if mode == "user":
                steer_pwm_duty = int(joystick.steer * config.JOYSTICK_STEERING_SCALE * 100)
                throttle_pwm_duty = int(joystick.accel * config.JOYSTICK_THROTTLE_SCALE * 100)
                if joystick.accel2: throttle_pwm_duty = int(config.FORWARD_S)
                elif joystick.accel1: throttle_pwm_duty = int(config.FORWARD_C)
                if getattr(joystick, "lb", 0) and not getattr(joystick, "rb", 0):
                    steer_pwm_duty = int(config.LEFT * config.JOYSTICK_STEER_BTN_RATIO)
                elif getattr(joystick, "rb", 0) and not getattr(joystick, "lb", 0):
                    steer_pwm_duty = int(config.RIGHT * config.JOYSTICK_STEER_BTN_RATIO)
                elif getattr(joystick, "lb", 0) and getattr(joystick, "rb", 0):
                    steer_pwm_duty = int(config.NUTRAL)
            elif mode == "auto_str":
                throttle_pwm_duty = int(joystick.accel * config.JOYSTICK_THROTTLE_SCALE * 100)
                if joystick.accel2: throttle_pwm_duty = int(config.FORWARD_S)
                elif joystick.accel1: throttle_pwm_duty = int(config.FORWARD_C)
                if getattr(joystick, "lb", 0) and not getattr(joystick, "rb", 0):
                    steer_pwm_duty = int(config.LEFT * config.JOYSTICK_STEER_BTN_RATIO)
                elif getattr(joystick, "rb", 0) and not getattr(joystick, "lb", 0):
                    steer_pwm_duty = int(config.RIGHT * config.JOYSTICK_STEER_BTN_RATIO)
                elif getattr(joystick, "lb", 0) and getattr(joystick, "rb", 0):
                    steer_pwm_duty = int(config.NUTRAL)
            recording = True if joystick.recording else False
            if joystick.breaking: motor.breaking()

        # === ★ true_colorによるスロットル常時上書き ===
        try:
            apply_override = THROTTLE_OVERRIDE_APPLY_IN_USER_MODE or (('mode' not in locals()) or mode != "user")
        except Exception:
            apply_override = True
        if true_color in THROTTLE_OVERRIDE_TRUE_COLOR and apply_override:
            throttle_pwm_duty = int(THROTTLE_OVERRIDE_TRUE_COLOR[true_color])

        # --- モータ出力 ---
        if config.mode_plan == "GCounter" and getattr(config, "HAVE_IMU", False):
            imu.GCounter()
            motor.set_steer_pwm_duty(steer_pwm_duty * (1 - 2 * imu.Gstr))
            motor.set_throttle_pwm_duty(throttle_pwm_duty * (1 - 2 * imu.Gthr))
        else:
            motor.set_steer_pwm_duty(steer_pwm_duty)
            motor.set_throttle_pwm_duty(throttle_pwm_duty)

        ts = time.time(); ts_run = round(ts - start_time, 2)
        if recording:
            d_stack = np.vstack((d_stack, np.insert(d, 0, [ts, steer_pwm_duty, throttle_pwm_duty]),))
            if getattr(config, "HAVE_CAMERA", False) and not getattr(config, "fpv", False):
                if not config.mode_plan == "CNN":
                    _, img = cam.read()
                cam.save(img, "%10.2f" % (ts), steer_pwm_duty, throttle_pwm_duty,
                        config.image_dir, config.IMAGE_W, config.IMAGE_H)

        if color_line_to_print:
            out_line = f"{color_line_to_print} | true_color={true_color} | count: {_fmt_counts(fsm_info['true_counts'], color_labels, COLOR_COUNT_INCLUDE_UNKNOWN)}"
            print(out_line)
        if 'mode' in locals() and mode == 'auto': mode = config.mode_plan
        if getattr(config, "plotter", False):
            print(message)
        else:
            print("Rec:{0}, Mode:{1}, RunTime:{2:>5}, Str:{3:>4}, Thr:{4:>4}, Color:{5}, Uls:[ {6}]".format(
                recording, ('-' if 'mode' not in locals() else mode), ts_run,
                steer_pwm_duty, throttle_pwm_duty, current_color, message))

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
                print("一時停止、Enterを押して走行再開!"); input()
finally:
    print('\n停止')
    try:
        motor.set_throttle_pwm_duty(config.STOP)
        motor.set_steer_pwm_duty(config.NUTRAL)
    except Exception: pass
    GPIO.cleanup()
    header = "Tstamp,Str,Thr," + ",".join(config.ultrasonics_list)
    try:
        np.savetxt(config.record_filename, d_stack[1:], delimiter=',', fmt='%10.2f', header=header, comments="")
        print('記録停止'); print("記録保存--> ", config.record_filename)
    except Exception as e:
        print("記録保存に失敗:", e)
    if getattr(config, "HAVE_CAMERA", False):
        print("画像保存--> ", getattr(config, "image_dir", "(未設定)"))
