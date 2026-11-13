import machine
import time

# I2C通信設定 (Pico: GP20=SDA, GP21=SCL)
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=400000)
BNO055_ADDR = 0x28

def init_bno055():
    """BNO055の初期化（NDOFモード設定）"""
    time.sleep(0.5)
    # 1. CONFIGモード(0x00)に設定
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x00')
    time.sleep(0.05)
    # 2. NDOF（9軸センサーフュージョンモード: 0x0C）に切替
    # ※重力ベクトルを分離するにはフュージョンモード（NDOF等）が必須です
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x0C')
    time.sleep(0.2)

def to_signed_16(val):
    """16ビット符号付き整数（-32768 ～ 32767）に変換"""
    if val >= 32768:
        val -= 65536
    return val

def get_gravity_vector():
    """重力ベクトル (gx, gy, gz) を取得 (単位: m/s^2)"""
    try:
        # レジスタ 0x2E (GRV_DATA_X_LSB) から 6バイト一括読み出し
        data = i2c.readfrom_mem(BNO055_ADDR, 0x2E, 6)
        
        # リトルエンディアン読み出し後、符号付き整数へ変換
        gx_raw = to_signed_16(data[0] | (data[1] << 8))
        gy_raw = to_signed_16(data[2] | (data[3] << 8))
        gz_raw = to_signed_16(data[4] | (data[5] << 8))

        # BNO055の重力ベクトルスケール: 1 m/s^2 = 100 LSB
        gx = gx_raw / 100.0
        gy = gy_raw / 100.0
        gz = gz_raw / 100.0

        return gx, gy, gz
    except Exception:
        return 0.0, 0.0, 0.0

# 初期化実行
init_bno055()

# メインループ
while True:
    gx, gy, gz = get_gravity_vector()
    print(f"Gravity (m/s2) -> X:{gx:6.2f}, Y:{gy:6.2f}, Z:{gz:6.2f}")
    time.sleep(0.02)