import os
import cv2
import numpy as np
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

def detect_obstacle(frame, debug=False):
    """
    簡單的障礙物偵測函數。
    這裡使用灰階轉換、模糊處理和閾值分割來偵測障礙物。
    """
    try:
        # 將影像轉換為灰階
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 模糊處理以減少噪點
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        # 設定閾值
        _, thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)  # 減少閾值值

        # 添加形態學操作以減少噪聲
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # 定義前方區域（畫面下半部的1.5%）
        height, width = thresh.shape
        roi_height = height // 90  # 只取畫面下半部的約1.5%
        roi = thresh[height - roi_height:height, :]

        # 計算前方區域白色像素的比例
        white_pixels = cv2.countNonZero(roi)
        total_pixels = roi.shape[0] * roi.shape[1]
        ratio = white_pixels / total_pixels

        if debug:
            print(f"白色像素比例: {ratio*100:.2f}%")
            # 保存中間結果進行檢查
            cv2.imwrite("debug_frame.jpg", frame)
            cv2.imwrite("debug_thresh.jpg", thresh)
            cv2.imwrite("debug_roi.jpg", roi)

        # 設定比例閾值，根據實際情況調整
        return ratio > 0.85  # 如果前方有超過 85% 的白色像素，視為有障礙物
    except Exception as e:
        print(f"障礙物偵測出錯: {e}")
        return False

# 冷卻時間設定（秒）
COOLDOWN_TIME = 7
last_stop_time = 0

# 主迴圈
while True:
    ret, frame = cap.read()
    if not ret:
        print("無法讀取攝像頭影像")
        break

    obstacle = detect_obstacle(frame, debug=False)  # 禁用調試模式

    current_time = time.time()

    if obstacle:
        if current_time - last_stop_time > COOLDOWN_TIME:
            try:
                ser.write(b'X')  # 發送停止指令
                print("障礙物偵測，已發送停止指令")
                last_stop_time = current_time
            except Exception as e:
                print(f"串口寫入出錯: {e}")

        # 在畫面上顯示警告文字
        cv2.putText(frame, "Stop", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    else:
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
