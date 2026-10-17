import time
from machine import Pin
import rp2

# 基板上のLED設定 ("LED" 指定でPico / Pico W 両対応)
led = Pin("LED", Pin.OUT)

# BOOTSELボタンの状態を取得する関数
def is_bootsel_pressed():
    return rp2.bootsel_button()

# 状態管理変数
state = "STOPPED"  # "STOPPED", "RUNNING"
start_time = 0
elapsed_time = 0

btn_press_start = 0
last_btn_state = False

print("=== BOOTSEL ストップウォッチ ===")
print("・短押し : スタート / ストップ")
print("・長押し(1秒) : リセット")
print("--------------------------------")

while True:
    current_msec = time.ticks_ms()
    btn_pressed = is_bootsel_pressed()

    # --- ボタンが押された瞬間 ---
    if btn_pressed and not last_btn_state:
        btn_press_start = current_msec

    # --- ボタンが離された瞬間（短押し判断） ---
    elif not btn_pressed and last_btn_state:
        press_duration = time.ticks_diff(current_msec, btn_press_start)

        # 50ms〜1000msの押下を「短押し」と判断（チャタリング回避含む）
        if 50 <= press_duration < 1000:
            if state == "STOPPED":
                # スタート
                start_time = current_msec - elapsed_time
                state = "RUNNING"
                led.value(1)  # 計測中はLED点灯
                print("\n[計測スタート]")
            elif state == "RUNNING":
                # ストップ
                elapsed_time = time.ticks_diff(current_msec, start_time)
                state = "STOPPED"
                led.value(0)  # 停止中はLED消灯
                secs = elapsed_time / 1000.0
                print(f"\n[ストップ] 記録: {secs:.2f} 秒")

    # --- 長押し検知（リセット処理） ---
    if btn_pressed and state == "STOPPED":
        if time.ticks_diff(current_msec, btn_press_start) >= 1000:
            elapsed_time = 0
            print("\n[リセット] 0.00 秒")
            # リセット完了通知でLEDを瞬き
            led.value(1)
            time.sleep(0.1)
            led.value(0)
            # ボタンが離されるまで待機
            while is_bootsel_pressed():
                time.sleep(0.01)
            btn_pressed = False

    last_btn_state = btn_pressed

    # --- 計測中のリアルタイム時間表示 ---
    if state == "RUNNING":
        current_elapsed = time.ticks_diff(current_msec, start_time) / 1000.0
        # \r で同じ行を上書き表示
        print(f"\r計測中: {current_elapsed:.2f} 秒", end="")
        time.sleep(0.05)
    else:
        time.sleep(0.01)
