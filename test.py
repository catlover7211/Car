import cv2

# 初始化攝像頭（嘗試不同的設備編號）
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("無法開啟攝像頭")
    exit()
else:
    print("攝像頭已成功打開")

# 設定攝像頭的解析度
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        print("無法讀取攝像頭影像")
        break

    # 打印幀的形狀以進行調試
    print(f"幀形狀：{frame.shape}")

    cv2.imshow('測試視窗', frame)

    # 按 'q' 鍵退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
