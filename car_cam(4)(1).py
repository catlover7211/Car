import os
import cv2
import serial
import time

# 配置串口參數
SERIAL_PORT = '/dev/serial/by-path/platform-3f980000.usb-usb-0:1.1.2:1.0'  # 替換為您的 Arduino 串口號
BAUD_RATE = 9600      # 必須與 Arduino 程式中的 Serial.begin 設定一致

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # 等待串口連接穩定
    print(f"已連接到 {SERIAL_PORT}，波特率 {BAUD_RATE}")
except serial.SerialException:
    print(f"無法連接到串口 {SERIAL_PORT}")
    exit()

# 初始化攝像頭（嘗試不同的設備編號）
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("無法開啟攝像頭")
    ser.close()
    exit()
else:
    print("攝像頭已成功打開")

# 設定攝像頭的解析度為較低的值
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# 說明文字
instructions = """
按鍵控制說明：
W - 向前行進
A - 向左轉
S - 倒退
D - 向右轉
X - 停止
Q - 退出
"""

print(instructions)

# 主迴圈
while True:
    ret, frame = cap.read()
    if not ret:
        print("無法讀取攝像頭影像")
        break

    # 顯示說明文字
    cv2.putText(frame, "Press W/A/S/D/X to Control Car", (50, 450),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    # 等待鍵盤輸入
    key = cv2.waitKey(1) & 0xFF

    if key != 255:  # 檢查是否有按鍵被按下
        try:
            key_char = chr(key).upper()
            print(f"按下的鍵: {key_char}")  # 調試用
        except ValueError:
            print(f"無法轉換按鍵代碼: {key}")
            continue

        if key_char == 'W':
            ser.write(b'W')
            print("向前行進")
        elif key_char == 'A':
            ser.write(b'A')
            print("向左轉")
        elif key_char == 'S':
            ser.write(b'S')
            print("倒退")
        elif key_char == 'D':
            ser.write(b'D')
            print("向右轉")
        elif key_char == 'X':
            ser.write(b'X')
            print("停止")
        elif key_char == 'Q':
            print("退出控制程式")
            break
        else:
            print("無效的按鍵，請按 W/A/S/D/X 或 Q 退出")

    # 顯示影像
    cv2.imshow("Arduino1", frame)

# 清理
cap.release()
ser.close()
cv2.destroyAllWindows()
