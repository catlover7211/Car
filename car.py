import cv2
import numpy as np
import serial
import time

# 配置串口參數
SERIAL_PORT = '/dev/cu.usbmodem1201'  # 替換為您的 Arduino 串口號，例如 'COM3'（Windows）或 '/dev/ttyACM0'（Linux）
BAUD_RATE = 9600      # 必須與 Arduino 程式中的 Serial.begin 設定一致

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # 等待串口連接穩定
    print(f"已連接到 {SERIAL_PORT}，波特率 {BAUD_RATE}")
except serial.SerialException:
    print(f"無法連接到串口 {SERIAL_PORT}")
    exit()

# 建立一個黑色的視窗
window_name = "Arduino 控制"
cv2.namedWindow(window_name)

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
    # 顯示黑色畫面
    frame = 255 * np.ones(shape=[500, 500, 3], dtype=np.uint8)
    cv2.putText(frame, "Press W/A/S/D/X Control Car", (50, 250),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.imshow(window_name, frame)

    # 等待鍵盤輸入
    key = cv2.waitKey(1) & 0xFF

    if key != 255:  # 檢查是否有按鍵被按下
        key_char = chr(key).upper()
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

# 清理
ser.close()
cv2.destroyAllWindows()
