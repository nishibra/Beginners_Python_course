import machine
import time

i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=100000)
BNO055_ADDR = 0x28

def init_bno055():
    time.sleep(0.5)
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x00') # CONFIGモード
    time.sleep(0.05)
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x0C') # NDOFモード
    time.sleep(0.2)

def to_signed_16(val):
    """16ビット符号なし整数を符号付き整数（-32768 ～ 32767）に変換"""
    if val >= 32768:
        val -= 65536
    return val

def get_euler():
    """オイラー角 (Roll, Pitch, Heading) を取得 (単位: 度)"""
    try:
        # レジスタ 0x1A から 6バイト読み出し
        data = i2c.readfrom_mem(BNO055_ADDR, 0x1A, 6)
        
        # リトルエンディアンで読み出し後、符号付き変換
        heading_raw = to_signed_16(data[0] | (data[1] << 8))
        roll_raw    = to_signed_16(data[2] | (data[3] << 8))
        pitch_raw   = to_signed_16(data[4] | (data[5] << 8))

        # スケールファクタ: 1 LSB = 1/16 度
        heading = heading_raw / 16.0
        roll    = roll_raw / 16.0
        pitch   = pitch_raw / 16.0

        return roll, pitch, heading
    except Exception:
        return 0.0, 0.0, 0.0

init_bno055()

while True:
    roll, pitch, heading = get_euler()
    print(f"{roll:.2f},{pitch:.2f},{heading:.2f}")
    time.sleep(0.02)