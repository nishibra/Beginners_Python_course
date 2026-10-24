MuJoCo（Multi-Joint dynamics with Contact）を使用すると、非常に綺麗で洗練された3D物理レンダリング画面でBNO055の姿勢を可視化できます。

Python用の **`mujoco`** ライブラリを使い、直感的に扱えるMJCF（XMLフォーマットの3Dモデル定義）を作成して動かす構成が最もスムーズです。

---

## 1. 事前準備（ライブラリのインストール）

PCのターミナルまたはThonnyの「システムシェルを開く」で以下を実行します。

```bash
pip install mujoco pyserial

```

---

## 2. 3Dモデルファイルの作成 (`board.xml`)

Pythonスクリプトと同じフォルダに、以下の内容で **`board.xml`** という名前のファイルを作成・保存します。

```xml
<mujoco model="bno055_board">
  <visual>
    <global offwidth="1280" offheight="720"/>
  </visual>

  <asset>
    <material name="board_mat" rgba="0.1 0.6 0.8 1" shininess="0.8" specular="0.5"/>
    <material name="sensor_mat" rgba="0.1 0.1 0.1 1" shininess="0.9"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/>
    
    <!-- BNO055ボード（Roll, Pitch, Yawの3軸ジョイントを持つボディ） -->
    <body name="board_body" pos="0 0 1">
      <joint name="yaw" type="hinge" axis="0 0 1" limited="false"/>
      <joint name="pitch" type="hinge" axis="0 1 0" limited="false"/>
      <joint name="roll" type="hinge" axis="1 0 0" limited="false"/>
      
      <!-- メイン基板 -->
      <geom type="box" size="0.3 0.2 0.02" material="board_mat"/>
      <!-- BNO055チップ表現（黒いチップ） -->
      <geom type="box" size="0.05 0.05 0.01" pos="0 0 0.025" material="sensor_mat"/>
    </body>
  </worldbody>
</mujoco>

```

---

## 3. PC側のPythonプログラム (`main.py`)

MuJoCoのビューアーを開き、Picoから受領した姿勢角（Roll, Pitch, Yaw）をジョイント角度（`qpos`）に直接代入してリアルタイムに可視化します。

```python
import math
import time
import serial
import mujoco
import mujoco.viewer

# --------------------------------------------------
# 1. シリアル通信設定
# --------------------------------------------------
PORT = 'COM3'  # 環境に合わせて変更してください ('/dev/ttyACM0' など)
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

```

---

## 主なポイント

* **グラフィックスのクオリティ**: ライティングやシャドウ、質感（マテリアル）の設定をXML側で行うため、非常に美しく滑らかなレンダリングが行われます。
* **カメラ操作**: マウスのドラッグ（回転）やホイール（ズーム）で、表示中に自由な角度から姿勢を確認できます。
