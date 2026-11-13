import math
import time
import serial
import mujoco
import mujoco.viewer

# --------------------------------------------------
# 1. シリアル通信設定
# --------------------------------------------------
PORT = 'COM14'  # 環境に合わせて変更してください ('/dev/ttyACM0' など)
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
    print(f"Connected to {PORT}")
except Exception as e:
    print(f"Failed to connect to serial port: {e}")
    exit()

# --------------------------------------------------
# 2. MuJoCoモデル読み込み
# --------------------------------------------------
model = mujoco.MjModel.from_xml_path('board.xml')
data = mujoco.MjData(model)

# --------------------------------------------------
# 3. メインループ（ビューアーの実行）
# --------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    # カメラの初期視点調整
    viewer.cam.distance = 2.0
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 45

    while viewer.is_running():
        step_start = time.time()

        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                parts = line.split(',')

                if len(parts) == 3:
                    roll = float(parts[0])
                    pitch = float(parts[1])
                    yaw = float(parts[2])

                    # 度 (degree) を ラジアン (radian) に変換してジョイントへ適用
                    data.qpos[0] = math.radians(yaw)
                    data.qpos[1] = math.radians(pitch)
                    data.qpos[2] = math.radians(roll)

                    mujoco.mj_forward(model, data)
            except (ValueError, UnicodeDecodeError):
                pass

        viewer.sync()

        # ループ周期調整 (約60FPS)
        time_until_next_frame = (1.0 / 60.0) - (time.time() - step_start)
        if time_until_next_frame > 0:
            time.sleep(time_until_next_frame)
