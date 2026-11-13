import machine
import time
import ujson
from bno055 import BNO055, CONFIG_MODE

# I2C通信設定 (Pico: GP20=SDA, GP21=SCL)
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=100000)
OFFSET_FILE = "bno055_offsets.json"
led = machine.Pin("LED", machine.Pin.OUT)

# BNO055を初期化（crystal=True で外部クリスタル有効化）
imu = BNO055(i2c, address=0x28, crystal=True)

def load_offsets():
    """保存された22バイトのオフセットをBNO055に復元"""
    try:
        with open(OFFSET_FILE, "r") as f:
            offsets = ujson.load(f)
        if len(offsets) == 22:
            last_mode = imu.mode(CONFIG_MODE)
            time.sleep_ms(25)
            i2c.writeto_mem(imu.address, 0x55, bytes(offsets))
            time.sleep_ms(25)
            imu.mode(last_mode)
            print("[OK] 保存済みオフセットを復元しました！")
            return True
    except Exception:
        print("[INFO] 保存済みオフセットなし（新規キャリブレーション開始）")
    return False

def save_offsets():
    """現在の22バイトのオフセットをJSONファイルに保存"""
    last_mode = imu.mode(CONFIG_MODE)
    time.sleep_ms(25)
    offsets = list(i2c.readfrom_mem(imu.address, 0x55, 22))
    imu.mode(last_mode)

    with open(OFFSET_FILE, "w") as f:
        ujson.dump(offsets, f)
        f.flush()
    print(f"[保存完了] '{OFFSET_FILE}' に最新オフセット(22バイト)を保存しました！")

# --------------------------------------------------
# 1. キャリブレーションフェーズ（全項目 3/3 になるまで待機）
# --------------------------------------------------
load_offsets()
step = 0

while True:
    sys, gyro, accel, mag = imu.cal_status()

    if step % 5 == 0:
        print(f"キャリブレーション中 -> Sys:{sys}/3 | Gyro:{gyro}/3 | Accel:{accel}/3 | Mag:{mag}/3")

    # すべて 3 になったら保存して次のフェーズへ進む
    if imu.calibrated():
        led.value(1)
        print("\n🎉 キャリブレーション完了！オフセットを保存してオイラー角の出力を開始します。\n")
        save_offsets()
        break

    led.value(step % 2)
    step += 1
    time.sleep(0.1)

# --------------------------------------------------
# 2. オイラー角連続出力フェーズ
# --------------------------------------------------
while True:
    try:
        # imu.euler() は (Heading/Yaw, Roll, Pitch) の順で度数法(deg)を返す
        yaw, roll, pitch = imu.euler()
        print(f"{yaw:5.1f}{roll:5.1f}{pitch:5.1f}")
    except Exception as e:
        print("読み取りエラー:", e)
    
    time.sleep(0.02)  # 20Hz出力（必要に応じて調整）