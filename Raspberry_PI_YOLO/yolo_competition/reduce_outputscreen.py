import time
import serial
import argparse
from ultralytics import YOLO
import cv2

# 参数解析
parser = argparse.ArgumentParser(description='YOLO物体检测串口输出程序')
parser.add_argument('--model', required=True, help='YOLO模型路径')
parser.add_argument('--source', required=True, help='输入源(摄像头索引如0，或usb0等)')
parser.add_argument('--thresh', type=float, default=0.5, help='置信度阈值(0-1)')
parser.add_argument('--detect-size', type=int, default=320,
                    help='检测分辨率(建议160-640)')
args = parser.parse_args()

# 串口初始化
try:
    ser = serial.Serial("/dev/ttyAMA0", 57600, timeout=1)
    print(f"[初始化] 串口已打开 {ser.name}")
except Exception as e:
    print(f"[错误] 串口初始化失败: {e}")
    ser = None

# 加载YOLO模型
model = YOLO(args.model, task='detect')


# 视频源初始化（支持usb0格式）
def open_video_source(source):
    if source.startswith('usb'):
        return cv2.VideoCapture(int(source[3:]))  # 提取数字部分
    elif source.isdigit():
        return cv2.VideoCapture(int(source))
    else:
        return cv2.VideoCapture(source)


cap = open_video_source(args.source)
if not cap.isOpened():
    print(f"[错误] 无法打开视频源 {args.source}")
    print("请尝试:")
    print("1. 使用数字索引 (如 0 对应 /dev/video0)")
    print("2. 确认摄像头已连接 (ls /dev/video*)")
    print("3. 检查用户权限 (video用户组)")
    exit()

print(f"[初始化] 已连接视频源 {args.source}")

# 主循环
last_sent = time.time()
SEND_INTERVAL = 0.1  # 最小发送间隔(秒)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[警告] 无法读取帧，重试中...")
            time.sleep(0.1)
            continue

        # 降分辨率检测
        detect_img = cv2.resize(frame, (args.detect_size, args.detect_size))

        # YOLO检测
        results = model(detect_img, verbose=False, imgsz=args.detect_size)
        detections = results[0].boxes

        if len(detections) > 0:
            # 寻找最大检测目标
            best_box = max(
                [box for box in detections if box.conf.item() > args.thresh],
                key=lambda box: (box.xyxy[0, 2] - box.xyxy[0, 0]) * (box.xyxy[0, 3] - box.xyxy[0, 1]),
                default=None
            )

            if best_box and time.time() - last_sent > SEND_INTERVAL:
                # 坐标转换
                x1, y1, x2, y2 = map(int, best_box.xyxy.cpu().numpy().squeeze())
                h, w = frame.shape[:2]
                scale_x, scale_y = w / args.detect_size, h / args.detect_size

                # 计算相对中心坐标
                center_x = ((x1 + x2) / 2 * scale_x - w / 2)
                center_y = ((y1 + y2) / 2 * scale_y - h / 2)

                # 串口发送
                if ser and ser.is_open:
                    command = f"&{center_x:.1f},{center_y:.1f}#\r\n"
                    try:
                        ser.write(command.encode('ascii'))
                        print(
                            f"[发送] {command.strip()} | 类别: {model.names[int(best_box.cls)]} | 置信度: {best_box.conf.item():.2f}")
                        last_sent = time.time()
                    except Exception as e:
                        print(f"[错误] 串口发送失败: {e}")

except KeyboardInterrupt:
    print("\n[退出] 用户中断")
except Exception as e:
    print(f"[异常] 发生错误: {e}")
finally:
    # 资源释放
    cap.release()
    if ser and ser.is_open:
        ser.close()
    print("程序已退出")