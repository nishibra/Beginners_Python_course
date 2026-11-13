import time
import serial
import mujoco
import mujoco.viewer

# --------------------------------------------------
# 1. シリアル通信設定
# --------------------------------------------------
PORT = 'COM3'
BAUD = 115200

try:
    # タイムアウトなし(0)でバッファ詰まりを防止
    ser = serial.Serial(PORT, BAUD, timeout=0)
    time.sleep(1.0)
    ser.reset_input_buffer()
    print(f"Successfully connected to {PORT}!")
except Exception as e:
    print(f"Serial Error: {e}")
    exit()

# --------------------------------------------------
# 2. MuJoCoモデル読み込み
# --------------------------------------------------
try:
    model = mujoco.MjModel.from_xml_path('boardBNO055.xml')
    data = mujoco.MjData(model)
except Exception as e:
    print(f"XML Load Error: {e}")
    print("board.xml が同じフォルダに存在し、<freejoint> が記述されているか確認してください。")
    exit()

# --------------------------------------------------
# 3. メインループ（リアルタイム描画）
# --------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 2.0
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 45

    line_buffer = ""

    while viewer.is_running():
        step_start = time.time()

        # シリアルバッファにデータがある場合
        if ser.in_waiting > 0:
            raw_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            line_buffer += raw_data

            # 改行コード区切りで最新データを抽出
            if '\n' in line_buffer:
                lines = line_buffer.split('\n')
                line_buffer = lines[-1]  # 未完成の末尾だけ残す

                # 到着した最新の完全な1行を取得
                last_line = lines[-2].strip()
                parts = last_line.split(',')

                if len(parts) == 4:
                    try:
                        w = float(parts[0])
                        x = -float(parts[1])
                        y = -float(parts[2])
                        z = float(parts[3])

                        # freejointの姿勢(qpos[3:7])へクォータニオンを適用
                        data.qpos[0:3] = [0, 0, 0.5]       # 位置を空中(Z=1)に固定
                        data.qpos[3:7] = [w, x, y, z]    # クォータニオン (w, x, y, z)

                        mujoco.mj_forward(model, data)
                    except ValueError:
                        pass

        viewer.sync()

        # 60FPS描画同期
        time_until_next_frame = (1.0 / 60.0) - (time.time() - step_start)
        if time_until_next_frame > 0:
            time.sleep(time_until_next_frame)

ser.close()
