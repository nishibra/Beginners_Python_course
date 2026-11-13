import machine
import time

# I2C通信設定 (Pico: GP20=SDA, GP21=SCL)
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=100000)
BNO055_ADDR = 0x28

def init_bno055():
    """BNO055の初期化（NDOFモード設定）"""
    time.sleep(0.5)
    # 1. CONFIGモード(0x00)に設定
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x00')
    time.sleep(0.05)
    # 2. NDOF（9軸センサーフュージョンモード: 0x0C）に切替
    # ※重力を分離するためにフュージョンモード（NDOF等）が必須です
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x0C')
    time.sleep(0.2)

def to_signed_16(val):
    """16ビット符号なし整数を符号付き整数（-32768 ～ 32767）に変換"""
    if val >= 32768:
        val -= 65536
    return val

def get_linear_acceleration():
    """線形加速度 (x, y, z) を取得 (単位: m/s^2)"""
    try:
        # レジスタ 0x28 から 6バイト（X, Y, Z 各2バイト）を一括読み出し
        data = i2c.readfrom_mem(BNO055_ADDR, 0x28, 6)
        
        # リトルエンディアンで読み出し後、符号付き変換
        x_raw = to_signed_16(data[0] | (data[1] << 8))
        y_raw = to_signed_16(data[2] | (data[3] << 8))
        z_raw = to_signed_16(data[4] | (data[5] << 8))

        # BNO055の線形加速度スケール: 1 m/s^2 = 100 LSB
        x = x_raw / 100.0
        y = y_raw / 100.0
        z = z_raw / 100.0

        return x, y, z
    except Exception:
        return 0.0, 0.0, 0.0

# 初期化実行
init_bno055()

# メインループ
while True:
    ax, ay, az = get_linear_acceleration()
    # カンマ区切りでシリアル出力 (単位: m/s^2)
    print(f"{ax:.2f},{ay:.2f},{az:.2f}")
    time.sleep(0.02)