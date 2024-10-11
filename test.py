import cv2
import numpy as np
import serial
import time
import threading
import queue

# 引入機器學習相關庫
import tensorflow as tf

# 配置串口參數
SERIAL_PORT = '/dev/cu.usbmodem1201'  # 替換為您的 Arduino 串口號
BAUD_RATE = 9600      # 必須與 Arduino 程式中的 Serial.begin 設定一致

# 嘗試連接串口，增加錯誤處理
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # 等待串口連接穩定
    print(f"已連接到 {SERIAL_PORT}，波特率 {BAUD_RATE}")
except serial.SerialException as e:
    print(f"無法連接到串口 {SERIAL_PORT}: {e}")
    exit()

# 初始化攝像頭，增加錯誤處理
cap = cv2.VideoCapture(0)  # 0 表示第一個攝像頭
if not cap.isOpened():
    print("無法打開攝像頭")
    ser.close()
    exit()

# 載入預訓練的機器學習模型
# 假設我們有一個名為 'path_detection_model.h5' 的模型
try:
    model = tf.keras.models.load_model('path_detection_model.h5')
    print("機器學習模型載入成功")
except Exception as e:
    print(f"無法載入機器學習模型: {e}")
    cap.release()
    ser.close()
    exit()

# 創建一個隊列，用於在線程之間傳遞影像
frame_queue = queue.Queue()
command_queue = queue.Queue()

def capture_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("無法獲取影像")
            break
        # 將影像放入隊列
        frame_queue.put(frame)
        # 如果收到退出信號，跳出循環
        if threading.current_thread().stopped():
            break

def process_frames():
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            # 影像預處理，適配模型輸入
            resized_frame = cv2.resize(frame, (224, 224))  # 根據模型輸入大小調整
            input_image = resized_frame / 255.0  # 正規化
            input_image = np.expand_dims(input_image, axis=0)

            # 使用模型進行預測
            prediction = model.predict(input_image)
            # 將預測結果轉換為指令
            command = interpret_prediction(prediction)
            # 將指令放入隊列
            command_queue.put((command, frame))

            # 如果收到退出信號，跳出循環
            if threading.current_thread().stopped():
                break
        else:
            time.sleep(0.01)  # 避免佔用過多 CPU 資源

def send_commands():
    while True:
        if not command_queue.empty():
            command, frame = command_queue.get()
            try:
                ser.write(command.encode())
            except serial.SerialException as e:
                print(f"串口通信錯誤: {e}")
                break
            # 顯示影像和指令
            cv2.putText(frame, f"Command: {command}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('AI 自動導航', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # 如果收到退出信號，跳出循環
            if threading.current_thread().stopped():
                break
        else:
            time.sleep(0.01)

def interpret_prediction(prediction):
    # 根據模型的輸出，解釋為具體的指令
    # 這裡假設模型輸出為三個類別的概率：[左轉, 前進, 右轉]
    command_index = np.argmax(prediction)
    if command_index == 0:
        print("向左轉")
        return 'A'
    elif command_index == 1:
        print("向前行進")
        return 'W'
    elif command_index == 2:
        print("向右轉")
        return 'D'
    else:
        print("停止")
        return 'X'

# 創建並啟動線程
capture_thread = threading.Thread(target=capture_frames)
process_thread = threading.Thread(target=process_frames)
send_thread = threading.Thread(target=send_commands)

# 增加線程停止的屬性
def set_thread_stopped(thread):
    def stopper():
        thread._stop = True
    return stopper

capture_thread.stopped = lambda: hasattr(capture_thread, "_stop")
process_thread.stopped = lambda: hasattr(process_thread, "_stop")
send_thread.stopped = lambda: hasattr(send_thread, "_stop")

capture_thread.start()
process_thread.start()
send_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("退出程式")
finally:
    # 設置線程停止
    set_thread_stopped(capture_thread)()
    set_thread_stopped(process_thread)()
    set_thread_stopped(send_thread)()
    # 等待線程結束
    capture_thread.join()
    process_thread.join()
    send_thread.join()
    # 清理資源
    ser.close()
    cap.release()
    cv2.destroyAllWindows()
