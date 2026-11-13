import machine
import time
import math

# I2C通信設定 (Pico: GP20=SDA, GP21=SCL)
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21), freq=400000)
BNO055_ADDR = 0x28

def init_bno055():
    """BNO055の初期化（NDOFモード設定）"""
    time.sleep(0.5)
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x00')
    time.sleep(0.05)
    i2c.writeto_mem(BNO055_ADDR, 0x3D, b'\x0C')
    time.sleep(0.2)

def to_signed_16(val):
    """16ビット符号付き整数変換"""
    if val >= 32768:
        val -= 65536
    return val

def get_quat_and_linear_accel():
    """クォータニオン(0x20〜)と線形加速度(0x28〜)の計14バイトを一括読み出し"""
    try:
        # レジスタ 0x20 から 0x2D までの 14バイトを連続読み出し（通信のズレ防止）
        data = i2c.readfrom_mem(BNO055_ADDR, 0x20, 14)

        # 1. クォータニオン (0x20 - 0x27)
        qw = to_signed_16(data[0] | (data[1] << 8)) / 16384.0
        qx = to_signed_16(data[2] | (data[3] << 8)) / 16384.0
        qy = to_signed_16(data[4] | (data[5] << 8)) / 16384.0
        qz = to_signed_16(data[6] | (data[7] << 8)) / 16384.0

        # 2. 線形加速度・ローカル座標 (0x28 - 0x2D)
        lax = to_signed_16(data[8]  | (data[9]  << 8)) / 100.0
        lay = to_signed_16(data[10] | (data[11] << 8)) / 100.0
        laz = to_signed_16(data[12] | (data[13] << 8)) / 100.0

        return (qw, qx, qy, qz), (lax, lay, laz)
    except Exception:
        return (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

def rotate_vector_by_quaternion(q, v):
    """
    クォータニオン q = (w, x, y, z) を使って
    ローカルベクトル v = (vx, vy, vz) をワールド座標系へ回転変換する
    計算式: v' = q * v * q^(-1)
    """
    qw, qx, qy, qz = q
    vx, vy, vz = v

    # クォータニオン正規化（誤差蓄積防止）
    norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if norm > 0:
        qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm

    # 高速化されたクォータニオン回転公式: v' = v + 2 * r x (r x v + w * v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    wx = vx + qw * tx + (qy * tz - qz * ty)
    wy = vy + qw * ty + (qz * tx - qx * tz)
    wz = vz + qw * tz + (qx * ty - qy * tx)

    return wx, wy, wz

# --------------------------------------------------
# 初期化と積分用変数
# --------------------------------------------------
init_bno055()

vel = [0.0, 0.0, 0.0]       # ワールド速度 [vx, vy, vz] (m/s)
pos = [0.0, 0.0, 0.0]       # ワールド位置 [px, py, pz] (m)
prev_acc = [0.0, 0.0, 0.0]  # 1ステップ前の加速度
stationary_count = 0

ACCEL_THRESHOLD = 0.09      # ノイズ除去のしきい値 (m/s^2)
STOP_COUNT_LIMIT = 15       # 静止判定フレーム数

last_time = time.ticks_ms()

# --------------------------------------------------
# メインループ
# --------------------------------------------------
while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000.0
    last_time = now

    if dt <= 0:
        continue

    # 1. クォータニオンとローカル線形加速度を同時取得
    quat, local_acc = get_quat_and_linear_accel()

    # 2. ワールド座標系（地面基準の X, Y, Z）へ回転変換
    world_acc = list(rotate_vector_by_quaternion(quat, local_acc))

    # 3. 各軸のノイズ除去と静止判定
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

    # 4. 積分計算（速度・距離）と ZUPT（静止時速度リセット）
    if stationary_count >= STOP_COUNT_LIMIT:
        vel = [0.0, 0.0, 0.0]
        prev_acc = [0.0, 0.0, 0.0]
    else:
        for i in range(3):
            vel[i] += 0.5 * (prev_acc[i] + world_acc[i]) * dt
            pos[i] += vel[i] * dt
            prev_acc[i] = world_acc[i]

    # 5. ワールド座標での移動距離 (cm) を出力
    print(f"X:{pos[0]*100:6.1f} cm | Y:{pos[1]*100:6.1f} cm | Z:{pos[2]*100:6.1f} cm")

    time.sleep(0.02)