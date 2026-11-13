import os
import time
import struct
import machine

# I2Cの初期化 (GP20: SDA, GP21: SCL)
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=100000)
BNO055_ADDR = 0x28
CALIB_FILE = "bno055_calib.bin"

# レジスタアドレス定義
REG_CHIP_ID = 0x00
REG_EULER_H_LSB = 0x1A
REG_CALIB_STAT = 0x35
REG_OPR_MODE = 0x3D
REG_SYS_TRIGGER = 0x3F
REG_ACC_OFFSET_X_LSB = 0x55  # オフセットデータ開始アドレス (0x55 ~ 0x6A の22バイト)

# 動作モード定義
MODE_CONFIG = 0x00
MODE_NDOF = 0x0C

def write_reg(reg, val):
    i2c.writeto_mem(BNO055_ADDR, reg, bytes([val]))

def read_reg(reg, length):
    return i2c.readfrom_mem(BNO055_ADDR, reg, length)

def set_mode(mode):
    write_reg(REG_OPR_MODE, mode)
    time.sleep_ms(25)  # モード切替待機（CONFIGへの移行は最低19ms必要）

def save_calibration():
    """CONFIGモードに切り替えて22バイトのオフセット値をファイルに保存"""
    set_mode(MODE_CONFIG)
    calib_data = read_reg(REG_ACC_OFFSET_X_LSB, 22)
    with open(CALIB_FILE, "wb") as f:
        f.write(calib_data)
    set_mode(MODE_NDOF)
    print(f"キャリブレーションデータを '{CALIB_FILE}' に保存しました。")

def load_calibration():
    """保存済みファイルがあればCONFIGモードでセンサーに書き戻す"""
    if CALIB_FILE in os.listdir():
        with open(CALIB_FILE, "rb") as f:
            calib_data = f.read()
        if len(calib_data) == 22:
            set_mode(MODE_CONFIG)
            i2c.writeto_mem(BNO055_ADDR, REG_ACC_OFFSET_X_LSB, calib_data)
            set_mode(MODE_NDOF)
            print(f"'{CALIB_FILE}' からキャリブレーションデータを復元しました。")
            return True
    return False

# 1. 接続確認とリセット初期化
if read_reg(REG_CHIP_ID, 1)[0] != 0xA0:
    raise RuntimeError("BNO055が見つかりません")

set_mode(MODE_CONFIG)
write_reg(REG_SYS_TRIGGER, 0x20)  # リセット
time.sleep_ms(650)
write_reg(REG_SYS_TRIGGER, 0x00)
time.sleep_ms(10)

# 2. 保存データの復元を試みる（なければ手動キャリブレーションを実施）
if not load_calibration():
    print("保存データが見つかりません。キャリブレーションを開始します...")
    set_mode(MODE_NDOF)
    
    while True:
        stat = read_reg(REG_CALIB_STAT, 1)[0]
        sys_cal = (stat >> 6) & 0x03
        gyro_cal = (stat >> 4) & 0x03
        accel_cal = (stat >> 2) & 0x03
        mag_cal = stat & 0x03
        
        print(f"Sys: {sys_cal} | Gyro: {gyro_cal} | Accel: {accel_cal} | Mag: {mag_cal}")
        
        if sys_cal == 3 and gyro_cal == 3 and accel_cal == 3 and mag_cal == 3:
            print("キャリブレーション完了！")
            save_calibration()
            break
        time.sleep(0.2)

# 3. 計測ループ（オイラー角の例）
print("計測開始...")
while True:
    data = read_reg(REG_EULER_H_LSB, 6)
    heading, roll, pitch = struct.unpack('<hhh', data)
    print(f"Heading: {heading/16.0:6.2f}°, Roll: {roll/16.0:6.2f}°, Pitch: {pitch/16.0:6.2f}°")
    time.sleep(0.2)