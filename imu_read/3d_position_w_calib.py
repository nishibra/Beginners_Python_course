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

def rotate_vector_by_quaternion(q, v):
    qw, qx, qy, qz = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx)
    )

def get_quat_and_linear_accel():
    """クォータニオン(0x20〜)と線形加速度(0x28〜)の計14バイトを一括読み出し"""
    try:
        data = i2c.readfrom_mem(BNO055_ADDR, 0x20, 14)
        qw = to_signed_16(data[0] | (data[1] << 8)) / 16384.0
        qx = to_signed_16(data[2] | (data[3] << 8)) / 16384.0
        qy = to_signed_16(data[4] | (data[5] << 8)) / 16384.0
        qz = to_signed_16(data[6] | (data[7] << 8)) / 16384.0

        lax = to_signed_16(data[8]  | (data[9]  << 8)) / 100.0
        lay = to_signed_16(data[10] | (data[11] << 8)) / 100.0
        laz = to_signed_16(data[12] | (data[13] << 8)) / 100.0

        return normalize_quat((qw, qx, qy, qz)), (lax, lay, laz)
    except Exception:
        return (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

def calibrate_stationary_bias(samples=100):
    """
    静止状態で指定サンプル数（約2秒間）の加速度と姿勢を計測し、
    基準クォータニオンとローカル加速度の平均誤差（バイアス）を算出する
    """
    print(f"[CALIB] Keep sensor completely still... ({samples} samples)")
    sum_acc = [0.0, 0.0, 0.0]
    q_last = (1.0, 0.0, 0.0, 0.0)

    for i in range(samples):
        # 計測中はLEDを高速点滅させる
        led.value(i % 2)
        q_last, acc = get_quat_and_linear_accel()
        sum_acc[0] += acc[0]
        sum_acc[1] += acc[1]
        sum_acc[2] += acc[2]
        time.sleep(0.02)

    bias = [sum_acc[0] / samples, sum_acc[1] / samples, sum_acc[2] / samples]
    q_zero_inv = quat_conjugate(q_last)
    
    led.value(1)  # 完了で点灯状態へ
    print(f"[CALIB] Done! Accel Bias: X={bias[0]:.4f}, Y={bias[1]:.4f}, Z={bias[2]:.4f}")
    return q_zero_inv, bias

# --------------------------------------------------
# 1. 初期化と起動時バイアス計測
# --------------------------------------------------
init_bno055()
time.sleep(0.5)

# 起動直後に100回サンプリング（約2秒）してバイアスと基準姿勢を取得
q_zero_inv, acc_bias = calibrate_stationary_bias(100)

vel = [0.0, 0.0, 0.0]
pos = [0.0, 0.0, 0.0]
prev_acc = [0.0, 0.0, 0.0]
stationary_count = 0

# バイアス除去済みのため、しきい値をより小さく設定可能
ACCEL_THRESHOLD = 0.05   # 0.05 m/s^2 未満はノイズとしてカット
STOP_COUNT_LIMIT = 10    # 10フレーム（約0.2秒）静止で即座に速度をゼロリセット

prev_button_state = 0
last_time = time.ticks_ms()

# --------------------------------------------------
# 2. メインループ
# --------------------------------------------------
while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000.0
    last_time = now

    if dt <= 0:
        continue

    # BOOTSELボタン押下で「再バイアス計測＆ゼロリセット」を実行
    button_state = rp2.bootsel_button()
    if button_state == 1 and prev_button_state == 0:
        q_zero_inv, acc_bias = calibrate_stationary_bias(60)  # 約1.2秒で再計測
        vel = [0.0, 0.0, 0.0]
        pos = [0.0, 0.0, 0.0]
        prev_acc = [0.0, 0.0, 0.0]
        stationary_count = 0
        last_time = time.ticks_ms()
        continue
    prev_button_state = button_state

    # データ取得
    q_current, raw_local_acc = get_quat_and_linear_accel()

    # 1. ローカル線形加速度から起動時のバイアス誤差を差し引く
    corrected_local_acc = (
        raw_local_acc[0] - acc_bias[0],
        raw_local_acc[1] - acc_bias[1],
        raw_local_acc[2] - acc_bias[2]
    )

    # 2. 相対クォータニオンの算出とワールド座標系への変換
    q_rel = normalize_quat(quat_multiply(q_zero_inv, q_current))
    world_acc = list(rotate_vector_by_quaternion(q_rel, corrected_local_acc))

    # 3. デッドバンド（不感帯）処理と静止判定
    is_stationary = True
    for i in range(3):
        if abs(world_acc[i]) < ACCEL_THRESHOLD:
            world_acc[i] = 0.0
        else:
            is_stationary = False

    if is_stationary:
        stationary_count += 1
    else:
        stationary_count = 0

    # 4. 積分計算と厳格ZUPT（静止時の速度完全リセット）
    if stationary_count >= STOP_COUNT_LIMIT:
        vel = [0.0, 0.0, 0.0]
        prev_acc = [0.0, 0.0, 0.0]
    else:
        for i in range(3):
            vel[i] += 0.5 * (prev_acc[i] + world_acc[i]) * dt
            pos[i] += vel[i] * dt
            prev_acc[i] = world_acc[i]

    # 7要素をカンマ区切りでシリアル出力 (qw, qx, qy, qz, px[m], py[m], pz[m])
    print(f"{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}")

    time.sleep(0.02)