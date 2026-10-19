from machine import Pin
import time

# Picoの基板上のLED（GP25）を準備
led = Pin(25, Pin.OUT)

print("起動プログラムスタート！")

# 永久ループでLEDを点滅
while True:
    led.value(1)  # 点灯
    time.sleep(0.5)
    led.value(0)  # 消灯
    time.sleep(0.5)
