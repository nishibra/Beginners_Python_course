**Raspberry Pi PicoのUART（ユーアート）**とは、マイコン（RP2040 / RP2350）が外部のデバイス（PC、GPSモジュール、Bluetoothモジュール、別のマイコンなど）と**1対1でシリアル通信を行うための機能**です。

正式名称は *Universal Asynchronous Receiver-Transmitter*（汎用非同期送受信機）で、クロック信号線を使わず、あらかじめ決めた通信速度（ボーレート）を頼りにデータを1ビットずつ順番に送受信します。

---

### PicoにおけるUARTの主な特徴

* **2つの独立したUART回路を搭載**
Picoのチップ内部には、**UART0** と **UART1** の2系統のハードウェアUARTが内蔵されています。それぞれ別の機器と同時に通信が可能です。
* **割り当て可能なピンが複数ある**
図の紫色のラベル（`UART0 TX` / `UART0 RX` など）が割り当て候補です。1つのUARTにつき複数のピン候補がありますが、実際に同時に使えるのはそれぞれ1ペア（TXとRXを1本ずつ）です。
* **UART0 の代表ピン:** GP0 (TX), GP1 (RX)
* **UART1 の代表ピン:** GP4 (TX), GP5 (RX)


* **電圧レベルは 3.3V**
PicoのGPIOは **3.3V専用** です。5V動作のArduinoや古いUSB-シリアル変換器と直結すると故障の原因になるため、レベル変換器（レベルシフタ）を挟む必要があります。

### 配線と通信の基本ルール

UARTで機器同士をつなぐ際は、以下の3つを合わせる必要があります。

| 項目 | 設定・つなぎ方のルール |
| --- | --- |
| **配線（クロス接続）** | Picoの **TX（送信）** ⇔ 相手の **RX（受信）**<br>

<br>Picoの **RX（受信）** ⇔ 相手の **TX（送信）**<br>

<br>Picoの **GND** ⇔ 相手の **GND**（基準電位を揃えるため必須） |
| **ボーレート（通信速度）** | `9600` や `115200` bps など、送受信側で完全に同じ数値に設定する |
| **データ形式** | 通常は「データ8ビット / パリティなし / ストップビット1（8N1）」を使用 |

https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTqFup7czFKehhmW7pfX9CJnQz1Xkgfn-N3Xeq6JRbV-j27kufKXn0BHWc&s=10

### MicroPythonでの基本的なコード例

`GP0` と `GP1`（UART0）を使って、115200bpsで文字列を送信し、受信データがあれば読み取る最小構成です。

```python
from machine import UART, Pin
import time

# UART0を初期化（TX=GP0, RX=GP1, ボーレート=115200）
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# データの送信
uart.write("Hello Pico UART!\n")

# データの受信確認と読み取り
while True:
    if uart.any():  # 受信バッファにデータがあるか確認
        data = uart.read()
        print("受信:", data)
    time.sleep(0.1)

```

※ USBケーブルでPCとつないでThonnyなどで `print()` を表示する通信は「USB CDC（仮想シリアルポート）」という別の仕組みで動いているため、UART0・UART1のピンとは干渉せずに併用できます。
