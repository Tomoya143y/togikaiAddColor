# coding:utf-8
import datetime
import os

# カウント対象の色（color_profiles.json のラベル名 
#アクションはrun.pyのCOLOR_TRIGGER_ACTIONSで決める
COLOR_TRIGGER_COLORS = ["pink", "green", "white","gray"]   # 例: ["pink","white"] などに絞ってもOK

# カウントの既定の閾値（未指定色はこれが使われる）
COLOR_TRIGGER_THRESHOLD = 50

# カウント表示にunknownも含めるか（表示のみの影響）
COLOR_COUNT_INCLUDE_UNKNOWN = True

# true_color に応じたスロットル上書き（常時）
THROTTLE_OVERRIDE_TRUE_COLOR = {"green": 80, "white": 30}
# 手動(user)モードでも上書きを適用
THROTTLE_OVERRIDE_APPLY_IN_USER_MODE = True

# Stage A/B はこれまで通り
COLOR_RAW_SAME_N  = 3    # RAWの同色連続回数で true_color 確定
COLOR_TRUE_SAME_N = 1    # 真の色“遷移”の既定閾値（使わない色はマイルストーンで上書き）

# しきい値を無視して、色ごとに回数別アクションを使いたい場合
COLOR_TRIGGER_MILESTONES = {
    "green": { 1: "RightHand", 2: "LeftHand", 3: "Stop" }
  # 1回目の white で PlanB モデルに切替
    # "pink":  { 2: "RightHand" },  # 例：2回目だけ何かする 等
}
# "RightHand","LeftHand","Stop","PlanB","PlanC","PlanD","Noop"



##色判定パラメーター
TCS34725_INTEGRATION_MS = 24  # 選択肢: 2.4, 24, 50, 101, 154, 700
TCS34725_GAIN = 4 # 1/4/16/60
UNKNOWN_THRESH = 50.0     # d がこの値を超えたら "unknown"
READ_INTERVAL  = 0.02    # 読み取り間隔（秒）

# === Color Trigger settings (for run.py) ===
# 全体のON/OFF
COLOR_TRIGGER_ENABLED = True


# 判断モード選択
model_plan_list = ["GoStraight",
                   "Right_Left_3","Right_Left_3_Records",
                   "RightHand","RightHand_PID","LeftHand","LeftHand_PID",
                   "NN", "CNN"]
mode_plan = "NN"
print("mode_plan:",mode_plan)
if mode_plan not in model_plan_list:
    print("Please set mode_plan as ",model_plan_list)
# 色→アクションの割当てを差し替えたい場合は run.py 内の
# COLOR_TRIGGER_ACTIONS をそのまま使う（高度な入替が必要ならrun.py側で定義）


# モーター出力パラメータ （デューティー比：-100~100で設定）
# スロットル用
FORWARD_S = 80 # B
FORWARD_C = 40 # A
STOP = 0
REVERSE = -100 
# ステアリング用
LEFT = 100 #<=100
NUTRAL = 0 
RIGHT = -100 #<=100

# 超音波センサの検知パラメータ 
## 距離関連、単位はmm
### 前壁の停止/検知距離
DETECTION_DISTANCE_STOP = 250
DETECTION_DISTANCE_BACK = 150
DETECTION_DISTANCE_Fr = 150
### 右左折判定基準距離
DETECTION_DISTANCE_RL = 550
### 右/左手法目標距離
DETECTION_DISTANCE_TARGET = 200 #目標距離
DETECTION_DISTANCE_RANGE = 50/2 #修正認知半径距離

## PIDパラメータ(PDまでを推奨)
K_P = 0.7 #0.7
K_I = 0.0 #0.0
K_D = 0.3 #0.3

# 判断モード関連パラメータ
## 過去の操作値記録回数
motor_Nrecords = 3

# 復帰モード選択
mode_recovery = "Back" #None, Back, Stop
recovery_str = LEFT # 復帰時のステアリング値
recovery_time = 0.1 #総復帰時間
recovery_braking = 1 #ブレーキ回数、ブレーキにはReverseを利用

# 出力系
# 判断結果出力、Thonyのplotterを使うならFalse
print_plan_result = False
# Thonnyのplotterを使う場合
plotter = False

#↑↑↑体験型イベント向けパラメータはここまで↑↑↑～～～～～～～～～～～～～～～～～～～～～～～～～～～～～～
# 車両調整用パラメータ(motor.pyで調整した後値を入れる)
## ステアリングのPWMの値
STEERING_CENTER_PWM = 360 #410 #410:newcar, #340~360:oldcar
STEERING_WIDTH_PWM = 100
# STEERING_RIGHT_PWM = STEERING_CENTER_PWM + STEERING_WIDTH_PWM
# STEERING_LEFT_PWM = STEERING_CENTER_PWM - STEERING_WIDTH_PWM
STEERING_RIGHT_PWM = 470
STEERING_LEFT_PWM = 270
### !!!ステアリングを壊さないための上限下限の値設定  
STEERING_RIGHT_PWM_LIMIT = 550
STEERING_LEFT_PWM_LIMIT = 250

## アクセルのPWM値
## モーターの回転音を聞き、音が変わらないところが最大/最小値とする
THROTTLE_STOPPED_PWM = 390 #めやす：390:newcar, #370~390:oldcar
THROTTLE_FORWARD_PWM = 490
THROTTLE_REVERSE_PWM = 290
THROTTLE_WIDTH_PWM = 100 

# 超音波センサの設定
## 使う超音波センサ位置の指示、計測ループが遅い場合は数を減らす
### 前３つ使う場合はこちらをコメントアウト外す
#ultrasonics_list = ["FrLH","Fr","FrRH"]
### ５つ使う場合はこちらをコメントアウト外す
ultrasonics_list = ["RrLH", "FrLH", "Fr", "FrRH","RrRH"]
### ８つ使う場合ははこちらのコメントアウト外す
#ultrasonics_list.extend(["BackRH", "Back", "BackLH"])
### ほかのファイルで使うためリスト接続名
ultrasonics_list_join = "uls_"+"_".join(ultrasonics_list)


## 超音波センサの最大測定距離(mm)、往復
cutoff_distance = 4000 
## 超音波センサの測定回数、ultrasonic.pyチェック用
sampling_times = 100
## 目標サンプリング周期（何秒に１回）、複数センサ利用の場合は合計値
sampling_cycle = 0.01
## 過去の超音波センサの値記録回数
ultrasonics_Nrecords = 3

# GPIOピン番号:超音波センサの位置の対応とPWMピンのチャンネル
## 新旧ボードの指定
board = "new" #old：~2023年たこ足配線、new：新ボード

## !!!超音波センサとPWMの配線を変えない限り触らない
if board == "old":
    ### Echo -- Fr:26, FrLH:24, RrLH:37, FrRH:31, RrRH:38
    e_list=[26,24,37,31,38]
    ### Triger -- Fr:15, FrLH:13, RrLH:35, FrRH:32, RrRH:36
    t_list=[15,13,35,32,36]
    ultrasonics_dict_trig = {"Fr":t_list[0], "FrLH":t_list[1], "RrLH":t_list[2], "FrRH":t_list[3], "RrRH":t_list[4]} 
    ultrasonics_dict_echo = {"Fr":e_list[0], "FrLH":e_list[1], "RrLH":e_list[2], "FrRH":e_list[3], "RrRH":e_list[4]} 
    CHANNEL_STEERING = 14 #old board
    CHANNEL_THROTTLE = 13 #old board

elif board == "new": #new board
    ### Echo -- Fr:26, FrLH:24, RrLH:37, FrRH:31, RrRH:38
    e_list=[11,13,15,29,31,33,35,37]
    ### Triger -- Fr:15, FrLH:13, RrLH:35, FrRH:32, RrRH:36
    t_list=[12,16,18,22,32,36,38,40]
    ultrasonics_dict_trig = {"Fr":t_list[0], "FrRH":t_list[1], "FrLH":t_list[2], "RrRH":t_list[3], "RrLH":t_list[4]} 
    ultrasonics_dict_echo = {"Fr":e_list[0], "FrRH":e_list[1], "FrLH":e_list[2], "RrRH":e_list[3], "RrLH":e_list[4]} 
    CHANNEL_STEERING = 1 #new board
    CHANNEL_THROTTLE = 0 #new board

else:
    print("Please set board as 'old' or 'new'.")

N_ultrasonics = len(ultrasonics_list)

# NNパラメータ
HAVE_NN = False
if mode_plan in ["NN", "CNN"] : HAVE_NN = True

## 学習済みモデルのパス
model_dir = "models"
model_name = "model_20251022_record_20251022_0848_tumesho_test.pth"
model_path = os.path.join(model_dir, model_name)

## NNモデルのパラメータ
hidden_dim = 64 #（隠れ層のノード数）
num_hidden_layers = 3 #（隠れ層の数）

## 学習のハイパーパラメータ
batch_size = 8
epochs = 30

## モデルの種類
model_type = "linear" #linear, categorical
### categoricalのカテゴリ設定、カテゴリ数は揃える↓　
num_categories = 3
# -100~100の範囲で小さな値→大きな値の順にする（しないとValueError: bins must increase monotonically.）
categories_Str = [RIGHT, NUTRAL, LEFT]
categories_Thr = [FORWARD_C, FORWARD_S, FORWARD_C] #Strのカテゴリに合わせて設定

bins_Str = [-101] # -101は最小値-100を含むため設定、境界の最大値は100
#bins_Thr = [-101]
# 分類の境界：binを設定(pd.cutで使う)
for i in range(num_categories):
    bins_Str.append((categories_Str[i]+categories_Str[min(i+1,num_categories-1)])/2)
bins_Str[-1] = 100
#for i in range(num_categories):
#    bins_Thr.append((categories_Thr[i]+categories_Thr[min(i+1,num_categories-1)])/2)
#bins_Thr[-1] = 100

# コントローラー（ジョイスティックの設定）
HAVE_CONTROLLER = True #True
JOYSTICK_STEERING_SCALE = -1.0
JOYSTICK_THROTTLE_SCALE = -0.6
##追加箇所
# ボタンでステアする時の強さ（±100がフルロック）
JOYSTICK_STEER_BTN_RATIO = 0.3  # 0.4〜0.8で好み調整

#CONTROLLER_TYPE = 'F710'            
JOYSTICK_DEVICE_FILE = "/dev/input/js0" 
## ジョイスティックのボタンとスティック割り当て
# F710の操作設定 #割り当て済み
JOYSTICK_A = 0 #0 #アクセル１
JOYSTICK_B = 1 #1 #アクセル２
JOYSTICK_X = 3 #2 #ブレーキ
JOYSTICK_Y = 4 #3 #記録停止開始
JOYSTICK_LB = 6 #5
JOYSTICK_RB = 7 #5
JOYSTICK_BACK = 10 #6
JOYSTICK_S = 11 #7 #自動/手動走行切り替え
JOYSTICK_Logi = 2 #8
JOYSTICK_LSTICKB = 8 #9
JOYSTICK_RSTICKB = 9 #10
JOYSTICK_AXIS_LEFT = 0 #ステアリング（左右）
JOYSTICK_AXIS_RIGHT = 1 #スロットル（上下）
JOYSTICK_HAT_LR = 0
JOYSTICK_HAT_DU = 4
######  ↓元の配置 ####### Command + / で一斉コメントアウト
# JOYSTICK_A = 0 #アクセル１
# JOYSTICK_B = 1 #アクセル２
# JOYSTICK_X = 2 #ブレーキ
# JOYSTICK_Y = 3 #記録停止開始
# JOYSTICK_LB = 4
# JOYSTICK_RB = 5
# JOYSTICK_BACK = 6
# JOYSTICK_S = 7 #自動/手動走行切り替え
# JOYSTICK_Logi = 8
# JOYSTICK_LSTICKB = 9
# JOYSTICK_RSTICKB = 10
# JOYSTICK_AXIS_LEFT = 0 #ステアリング（左右）
# JOYSTICK_AXIS_RIGHT = 4 #スロットル（上下）
# JOYSTICK_HAT_LR = 0
# JOYSTICK_HAT_DU = 1
############

# カメラの設定
HAVE_CAMERA = False
IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3         # default RGB=3, make 1 for mono
#CAMERA_FRAMERATE = 20 #DRIVE_LOOP_HZ
#CAMERA_VFLIP = False
#CAMERA_HFLIP = False
#IMSHOW = False #　画像を表示するか

#↑↑↑ルールベース/機械学習講座向けパラメータはここまで↑↑↑～～～～～～～～～～～～～～～～～～～～～～～～～～～～～～
# その他
# ジャイロを使った動的制御モード選択
HAVE_IMU = False #True
mode_dynamic_control = "GCounter" #"GCounter", "GVectoring"

# FPV 下記のport番号
## fpvがONの時は画像保存なし
fpv = False #True
port = 8910

# 走行記録
## 測定データ
records = "records"
if not os.path.exists(records):
    # ディレクトリが存在しない場合、ディレクトリを作成する
    os.makedirs(records)
    print("make dir as ",records)
## 記録したcsvファイル名
record_filename = './'+records+'/record_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'

# 画像保存
if HAVE_CAMERA ==True:
    img_size = (IMAGE_W, IMAGE_H, IMAGE_DEPTH)
    images = "images"
    if not os.path.exists(images):
        # ディレクトリが存在しない場合、ディレクトリを作成する
        os.makedirs(images)
        print("make dir as ",images)
    ## 記録するフォルダ名
    image_dir = './'+images+'/image_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(image_dir)
    print("make dir as ",image_dir)
