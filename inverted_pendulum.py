#!/usr/bin/env micropython
# Kondo KRS servo control program (Standalone main.py with Continuous Rotation Align & LED Status)
# 2026.11
# by T.Nishimura of AiRRC
#

from machine import Pin, I2C
import time
import rp2
from krs_driver_v2_0 import *
from bno055 import *

# ==============================================================================
# オンボードLEDの初期化
# ==============================================================================
led = Pin("LED", Pin.OUT)
led.off()

# 電源投入直後の安定化処理 (PC切り離し時の起動対策)
time.sleep(1.0)

# BNO055 (IMU) 設定
i2c = I2C(0, sda=Pin(20), scl=Pin(21)) 
imu = BNO055(i2c)
time.sleep(0.5)

# サーボモーター設定
id_r = 1
id_l = 2
ct = 7500  # 基準位置（目標位置）

krs = KRSdriver()

# ------------------------------------------------------------------------------
# 基本制御関数
# ------------------------------------------------------------------------------
def free():
    """サーボを脱力（フリー）状態にする"""
    krs.read_position_set_free(id_r)
    krs.read_position_set_free(id_l)

def drive(r_sp, l_sp):
    """車輪サーボの出力制御 (±4000制限)"""
    if r_sp > 4000: r_sp = 4000
    elif r_sp < -4000: r_sp = -4000
    if l_sp > 4000: l_sp = 4000
    elif l_sp < -4000: l_sp = -4000                     
    
    krs.set_position_ret(id_r, ct - r_sp)
    krs.set_position_ret(id_l, ct + l_sp)

def is_bootsel_pressed():
    """BOOTSELボタンの状態を取得"""
    return rp2.bootsel_button()

# ------------------------------------------------------------------------------
# PIDコントローラー Class
# ------------------------------------------------------------------------------
class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        if dt <= 0:
            return 0
        
        error = self.setpoint - current_value
        
        self.integral += error * dt
        if self.integral > 500: self.integral = 500
        elif self.integral < -500: self.integral = -500

        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

    def reset(self):
        self.integral = 0
        self.previous_error = 0

# ==============================================================================
# メイン初期化処理
# ==============================================================================

# ------------------------------------------------------------------------------
# 【追加機能】回転モードの右モータ（id_r）をゆっくり回し、7500で停止させる
# ------------------------------------------------------------------------------
print("Slowly rotating Right Motor (id_r) to 7500...")
led.on()

slow_speed = 20  # ゆっくり回転させるスピード値（調整可: 200〜500程度）
timeout_start = time.ticks_ms()

# 現在位置の取得
initial_pos = krs.read_position(id_r)

if initial_pos is not None and initial_pos > 0:
    # 現在位置と7500との位置関係に応じて回転方向（正転/逆転）を決定
    if initial_pos < ct:
        search_speed = slow_speed
    else:
        search_speed = -slow_speed

    # 右モーター低速回転開始
    drive(search_speed, 0)

    # 7500到達チェックループ
    while True:
        try:
            current_pos = krs.read_position(id_r)
            if current_pos is not None and current_pos > 0:
                # 7500に到達（または通り過ぎた）か判定
                if (search_speed > 0 and current_pos >= ct) or (search_speed < 0 and current_pos <= ct):
                    drive(0, 0)  # 即座に停止
                    print("Reached 7500! Stopped.")
                    break
        except Exception:
            pass

        # タイムアウト安全ガード (5秒経過したら停止)
        if time.ticks_diff(time.ticks_ms(), timeout_start) > 5000:
            drive(0, 0)
            print("Alignment timeout! Stopped.")
            break

        time.sleep(0.01)

# 位置決め完了後、ボタン待ちに入るためにサーボをフリーにする
drive(0, 0)
free()
led.off()
time.sleep(0.5)

print("Waiting for BOOTSEL button press...")

# ------------------------------------------------------------------------------
# 【状態1】待機中: ゆっくり点滅 (0.5s周期)
# ------------------------------------------------------------------------------
pitch = 0.0
last_blink = time.ticks_ms()

while is_bootsel_pressed() == 0:
    if time.ticks_diff(time.ticks_ms(), last_blink) > 500:
        led.toggle()
        last_blink = time.ticks_ms()

    time.sleep(0.02)
    try:
        eu = imu.euler()
        if eu[1] is not None:
            pitch = -eu[1]
    except Exception:
        pass

# ------------------------------------------------------------------------------
# 【状態2】キャリブレーション & 手放しブレ吸収中: 高速点滅 (0.1s周期)
# ------------------------------------------------------------------------------
print("Button Pressed! Hold stable...")

for _ in range(5):
    led.on()
    time.sleep(0.1)
    led.off()
    time.sleep(0.1)

# 安定した時点の姿勢角度を取得して目標値にセット
try:
    eu = imu.euler()
    if eu[1] is not None:
        pitch = -eu[1]
except Exception:
    pass

target_angle = pitch
print("Target Angle Set:", target_angle)

# PIDパラメータ設定
kp = 1.0
ki = 5.0
kd = 0.03

# 位置決め制御用のパラメータ設定
pos_kp = 0.0005   # 右サーボの位置偏差をピッチ角補正量へ変換するゲイン
r_pos = ct       # 右サーボ位置の初期値 (7500)

pid_controller = PIDController(kp, ki, kd, target_angle)

# ==============================================================================
# 制御メインループ
# 【状態3】制御実行中: LED常時点灯
# ==============================================================================
if __name__ == "__main__":
    pid_controller.reset()
    last_time = time.ticks_ms()
    last_valid_pitch = target_angle
    
    led.on()
    
    loop_count = 0
    
    while True:
        try:
            eu = imu.euler()
            if eu[1] is None:
                pitch = last_valid_pitch
            else:
                pitch = -eu[1]
                last_valid_pitch = pitch
        except Exception:
            pitch = last_valid_pitch

        # 転倒ガード（±15度を超えたらモーター停止して消灯）
        if not (-15 < pitch < 15):
            print("Fallen detected! Stopping motors.")
            free()
            led.off()
            break

        # ----------------------------------------------------------------------
        # 右モータ（id_r）の現在位置を取得して位置補正量を計算
        # ----------------------------------------------------------------------
        loop_count += 1
        if loop_count % 5 == 0:
            try:
                val = krs.read_position(id_r)
                if val is not None and val > 0:
                    r_pos = val
            except Exception:
                pass

        pos_error = r_pos - ct
        d_pos = pos_error * pos_kp

        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, last_time) / 1000.0
        
        if dt <= 0:
            time.sleep_ms(1)
            continue
            
        last_time = current_time

        control_output = pid_controller.update(pitch + d_pos, dt)
        
        factor = 10
        left_speed = int(control_output * factor)
        right_speed = int(control_output * factor * 0.8)
        
        drive(right_speed, left_speed)
        time.sleep(0.005)

    free()
    led.off()