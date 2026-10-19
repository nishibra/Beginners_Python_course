from machine import Pin
import rp2
led = Pin(25, Pin.OUT)
led.value(0)  
button = rp2.bootsel_button()
if button == 1:
    print("ボタンが押されました！")
    led.value(1) 

