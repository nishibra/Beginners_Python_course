import time
import serial

# 環境に合わせてCOMポートを変更してください
PORT = 'COM3'
BAUD = 115200

print(f"Connecting to {PORT}...")

try:
    # タイムアウトを設定してシリアル接続を開く
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    time.sleep(1.0)              # 接続の安定待ち
    ser.reset_input_buffer()     # 接続時に溜まっている古いデータをクリア
    print(f"Successfully connected to {PORT}!")
    print("Reading data... Press Ctrl+C to stop.\n" + "-" * 40)
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# 受信ループ
try:
    while True:
        if ser.in_waiting > 0:
            # 1行読み込んでデコード（改行コードを除去）
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"Received: {line}")
        time.sleep(0.01) # CPU負荷軽減のための短いスリープ

except KeyboardInterrupt:
    print("\nStopping serial test.")
finally:
    ser.close()
    print("Serial port closed.")
