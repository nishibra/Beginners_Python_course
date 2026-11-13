import machine
import time

# I2C通信設定 (Pico: GP20=SDA, GP21=SCL)
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=400000)
BNO055_ADDR = 0x28

def init_bno055():
    """BNO055の初期化"""
    time.sleep(0.5)
    
    # 1. CONFIGモード(0x00)に設定
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x00')
    time.sleep(0.05)

    # 2. NDOF（9軸フュージョンモード: 0x0C）に切替
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x0C')
    time.sleep(0.2)
    
    print("BNO055 NDOF Mode set.")

def to_signed_16(val):
    """16ビット符号なし整数を符号付き整数（-32768 ～ 32767）に変換"""
    if val >= 32768:
        val -= 65536
    return val

def get_quaternion():
    """クォータニオン(w, x, y, z)取得"""
    try:
        # 0x20 から 8バイト読み出し
        data = i2c.readfrom_mem(BNO055_ADDR, 0x20, 8)
        
        # リトルエンディアンで16ビット読み出し後、確実に符号付き変換
        w_raw = to_signed_16(data[0] | (data[1] << 8))
        x_raw = to_signed_16(data[2] | (data[3] << 8))
        y_raw = to_signed_16(data[4] | (data[5] << 8))
        z_raw = to_signed_16(data[6] | (data[7] << 8))

        # スケールファクタ: 1 LSB = 1 / 16384.0
        w = w_raw / 16384.0
        x = x_raw / 16384.0
        y = y_raw / 16384.0
        z = z_raw / 16384.0

        return w, x, y, z
    except Exception as e:
        return 1.0, 0.0, 0.0, 0.0

# 初期化実行
init_bno055()

while True:
    w, x, y, z = get_quaternion()
    print(f"{w:.4f},{x:.4f},{y:.4f},{z:.4f}")
    time.sleep(0.02)