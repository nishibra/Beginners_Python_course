from machine import I2C, Pin

# I2C0の初期化 (例: SCL=Pin(21), SDA=Pin(20))
i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)

# 接続されているI2Cデバイスのアドレスをスキャン
devices = i2c.scan()

if devices:
    for dev in devices:
        print(f"I2Cデバイスを発見しました: 10進数={dev}, 16進数={hex(dev)}")
else:
    print("I2Cデバイスが見つかりませんでした。配線を確認してください。")
