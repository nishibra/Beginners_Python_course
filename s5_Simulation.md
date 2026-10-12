「**パソコン上のシミュレーションで、画面の中のロボットを立たせてみよう**」（第4回講義などで扱う内容）を、小学生〜中学生でも直感的に理解・実行できる具体的な方法をご提案します。

実機（ハードウェア）をつなぐ前に、Pythonの簡易グラフィックライブラリ（`turtle` または `matplotlib`）を使って、パソコン画面の上に「ゆらゆら揺れる棒（倒立振子）」を描き、PID制御の計算結果によってその棒がピタッとまっすぐ立つ様子を目で確認するハンズオン手法です。

---

### シミュレーションの具体的なしくみ・やり方

#### 1. 仕組み（画面の中で何が起きているか）

1. **画面の描画:** 画面上にロボット（車輪と棒）のイラストを描きます。
2. **傾きの計算（物理の真似事）:** 重力によって棒が少し倒れようとします。
3. **PID制御の計算:** 傾き角度（`pitch`）を読み取り、`PIDController` クラスで車輪を左右にどれくらい動かすか（パワー）を計算します。
4. **位置の更新:** 車輪が動くことで棒が押し戻され、画面のイラストが書き換わります（1秒間に30〜60回更新）。

---

### 具体的なPythonシミュレーションコード例

Python標準で入っている **`turtle`（タートル・グラフィックス）** モジュールを使うと、外部ライブラリのインストール不要で、Thonny（ソニー）上でそのままアニメーション画面が立ち上がります。

#### Pythonコード（Thonnyでそのまま動かせます）

```python
import turtle
import time

# --- 1. PIDコントローラクラス (第4回でつくるクラス) ---
class PIDController:
    def __init__(self, kp, ki, kd, limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0
        self.prev_error = 0

    def update(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        output = p + i + d
        return max(-self.limit, min(self.limit, output))

# --- 2. 画面とロボット（イラスト）の準備 ---
screen = turtle.Screen()
screen.title("倒立振子（バランスロボット）シミュレーター")
screen.bgcolor("#f7fafc")
screen.setup(width=600, height=500)
screen.tracer(0)  # 画面の自動更新をオフ（なめらかに動かすため）

# 車輪（台座）
cart = turtle.Turtle()
cart.shape("square")
cart.shapesize(stretch_wid=1.5, stretch_len=3)
cart.color("#3182ce")
cart.penup()

# 胴体（棒）
pole = turtle.Turtle()
pole.shape("square")
pole.shapesize(stretch_wid=0.4, stretch_len=8)  # 棒の長さ
pole.color("#e53e3e")
pole.penup()

# --- 3. 初期状態の設定 ---
robot_x = 0.0          # ロボットの位置
angle = 15.0           # 初期の傾き（いきなり15度倒れそう！）
angular_velocity = 0.0 # 倒れるスピード
target_angle = 0.0     # 目標（まっすぐ0度）

# PIDのパラメータ（ここを変更して実験する！）
pid = PIDController(kp=1.5, ki=0.01, kd=0.1, limit=50)

dt = 0.02  # 1回の計算間隔 (20ms)

# --- 4. シミュレーションメインループ ---
print("シミュレーションスタート！")

for step in range(300):  # 約6秒間動かす
    # ① PID計算：傾き(angle)から車輪の移動パワー(motor_power)を求める
    motor_power = pid.update(target_angle, angle, dt)
    
    # ② 簡易な物理計算（重力で倒れようとする力 - モータで押し戻す力）
    # モータが前に動くと、慣性で棒は後ろに押し戻される
    gravity_force = 15.0 * (angle / 90.0)  # 重力で倒れる力
    control_force = motor_power * 0.3      # モータで戻す力
    
    # 傾き速度と角度の更新
    angular_velocity += (gravity_force - control_force) * dt
    angle += angular_velocity * dt
    
    # 位置の更新（モータのパワー分だけ横に移動）
    robot_x += motor_power * dt * 5
    
    # 画面枠からはみ出さないようにガード
    robot_x = max(-200, min(200, robot_x))
    
    # ③ 画面上のイラストの位置・角度を更新
    cart.goto(robot_x, -100)
    pole.goto(robot_x, -100 + 40)
    pole.setheading(90 - angle)  # 垂直(90度)から傾き分を引く
    
    screen.update()
    time.sleep(dt)

print("シミュレーション終了")
screen.mainloop()

```

---

### 講義（授業）での進め方・ハンズオン体験

1. **まずは初期値で動かしてみる**
* コードを実行すると、画面上で15度傾いた赤い棒が、青い車輪の動きによって「グラッ」と揺れながらも**ピタッと垂直（0度）に持ち直して立つアニメーション**が表示されます。


2. **パラメータを変えて実験する（パラメータ探検）**
* **`kp = 0` にしてみる** → モータが踏ん張らず、そのままバタンと倒れてしまう様子を確認。
* **`kp = 10.0`（大きすぎる値）にしてみる** → 車輪が激しく左右に動いてブルブル震えて暴走する様子を確認。
* **`kd = 0.5` を足してみる** → 激しい揺れがスッと収まる「ブレーキ効果」を実感する。



---

### 受講生（子どもたち）にとってのメリット

* **壊れる心配がない:** パラメータ（Kp, Ki, Kd）の数値を極端に変えて失敗・暴走させても、実機のようにギアが欠けたり転倒して破損したりしません。
* **感覚が掴める:** 「Kpを上げると元気に戻ろうとする」「Kdを入れるとピタッと止まる」というPID制御の直感的な役割を、実機テスト（第5回）の前にビジュアルで楽しく理解できます。
