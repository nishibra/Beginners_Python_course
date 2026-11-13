import machine
import time
import math
import rp2

# I2C通信設定 (Pico: GP20=SDA, GP21=SCL)
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=100000)
BNO055_ADDR = 0x28
led = machine.Pin("LED", machine.Pin.OUT)

def init_bno055():
    """BNO055の初期化（NDOFモード設定）"""
    time.sleep(0.5)
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x00')
    time.sleep(0.05)
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x0C')
    time.sleep(0.2)

def to_signed_16(val):
    if val >= 32768:
        val -= 65536
    return val

def normalize_quat(q):
    w, x, y, z = q
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/norm, x/norm, y/norm, z/norm) if norm > 0 else (1.0, 0.0, 0.0, 0.0)

def quat_conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

def get_quaternion():
    """クォータニオン (w, x, y, z) を取得"""
    try:
        data = i2c.readfrom_mem(BNO055_ADDR, 0x20, 8)
        w = to_signed_16(data[0] | (data[1] << 8)) / 16384.0
        x = to_signed_16(data[2] | (data[3] << 8)) / 16384.0
        y = to_signed_16(data[4] | (data[5] << 8)) / 16384.0
        z = to_signed_16(data[6] | (data[7] << 8)) / 16384.0
        return normalize_quat((w, x, y, z))
    except Exception:
        return (1.0, 0.0, 0.0, 0.0)

def detect_fall_direction(q, threshold_deg=60.0):
    """
    直立基準からの相対クォータニオン q を受け取り、
    (傾き角度, 状態/転倒方向) を返す
    """
    w, x, y, z = q

    # 直立時の頭上ベクトル(0, 0, 1)が現在どちらを向いているか計算
    gx = 2.0 * (x * z + w * y)          # 前後方向の傾き成分
    gy = 2.0 * (y * z - w * x)          # 左右方向の傾き成分
    gz = 1.0 - 2.0 * (x * x + y * y)    # 上向きの成分

    # 直立からの総合傾き角 (0°～180°)
    gz_clipped = max(-1.0, min(1.0, gz))
    tilt_deg = math.degrees(math.acos(gz_clipped))

    # しきい値未満なら直立と判定
    if tilt_deg < threshold_deg:
        return tilt_deg, "UPRIGHT (直立)"

    # 前後(|gx|)と左右(|gy|)のどちらに強く倒れているか比較
    # ※センサーの取り付け向きに合わせて符号(+/-)を入れ替えて調整可能
    if abs(gx) >= abs(gy):
        if gx > 0:
            direction = "FRONT (前・うつ伏せ)"
        else:
            direction = "BACK (後・仰向け)"
    else:
        if gy > 0:
            direction = "LEFT (左倒れ)"
        else:
            direction = "RIGHT (右倒れ)"

    return tilt_deg, direction

# --------------------------------------------------
# 1. 初期化（起動時の姿勢を直立基準として記録）
# --------------------------------------------------
init_bno055()
time.sleep(0.5)

q_zero_inv = quat_conjugate(get_quaternion())
prev_button = 0

FALL_ANGLE_THRESHOLD = 30.0  # 60度以上傾いたら転倒判定
FALL_CONFIRM_COUNT = 15      # 0.3秒間(15ループ)継続で確定
fall_counter = 0
confirmed_status = "UPRIGHT (直立)"

# --------------------------------------------------
# 2. メインループ
# --------------------------------------------------
while True:
    q_current = get_quaternion()

    # BOOTSELボタンで直立基準をリセット
    btn = rp2.bootsel_button()
    if btn == 1 and prev_button == 0:
        q_zero_inv = quat_conjugate(q_current)
        fall_counter = 0
        confirmed_status = "UPRIGHT (直立)"
        print("--- UPRIGHT ZERO RESET ---")
    prev_button = btn

    # 直立基準からの相対クォータニオンを計算
    q_rel = normalize_quat(quat_multiply(q_zero_inv, q_current))

    # 傾き角と方向を判定
    tilt_deg, current_status = detect_fall_direction(q_rel, FALL_ANGLE_THRESHOLD)

    # 誤検知防止（一定時間継続したら確定）
    if current_status != "UPRIGHT (直立)":
        fall_counter += 1
        if fall_counter >= FALL_CONFIRM_COUNT:
            confirmed_status = current_status
            led.value(1)
    else:
        fall_counter = 0
        confirmed_status = "UPRIGHT (直立)"
        led.value(0)

    print(f"Tilt: {tilt_deg:5.1f} deg | Status: {confirmed_status}")
    time.sleep(0.02)