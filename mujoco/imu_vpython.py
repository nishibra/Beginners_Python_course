import serial
import math
from vpython import box, vector, color, rate, scene

# --------------------------------------------------
# 1. シリアル通信の設定
# --------------------------------------------------
# お使いのPicoのCOMポート番号（例: 'COM3' や '/dev/ttyACM0'）に変更してください
PORT = 'COM3'
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
except Exception as e:
    print(f"Failed to connect: {e}")
    exit()

# --------------------------------------------------
# 2. 3D描画画面の設定 (VPython)
# --------------------------------------------------
scene.title = "BNO055 3D Orientation Visualizer"
scene.width = 800
scene.height = 600
scene.forward = vector(-1, -1, -1)  # カメラの視点調整

# 基準となる3Dブロック（ボードモデル）を作成
board = box(length=6, height=0.5, width=3, color=color.cyan)

# --------------------------------------------------
# 3. メインループ（データの受信と3D描画更新）
# --------------------------------------------------
while True:
    rate(60)  # ループ速度を最大60FPSに制限
    
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            data = line.split(',')
            
            if len(data) == 3:
                roll = float(data[0])
                pitch = float(data[1])
                yaw = float(data[2])
                
                # 度（Degree）からラジアン（Radian）へ変換
                r_rad = math.radians(roll)
                p_rad = math.radians(pitch)
                y_rad = math.radians(yaw)
                
                # オブジェクトの回転姿勢をリセット
                board.up = vector(0, 1, 0)
                board.axis = vector(1, 0, 0)
                
                # Roll, Pitch, Yaw の回転を順に適用
                board.rotate(angle=r_rad, axis=vector(1, 0, 0))
                board.rotate(angle=p_rad, axis=vector(0, 0, 1))
                board.rotate(angle=y_rad, axis=vector(0, 1, 0))
                
        except (ValueError, UnicodeDecodeError):
            # 受信データの欠損や変換エラーをスキップ
            pass
