import os
import sys
import argparse
import time
import cv2
import numpy as np
from ultralytics import YOLO
import serial
import RPi.GPIO as GPIO

# 初始化GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT)
GPIO.output(26, GPIO.HIGH)

# 参数解析
parser = argparse.ArgumentParser()
parser.add_argument('--model', required=True, help='YOLO模型路径')
parser.add_argument('--source', required=True, help='输入源(文件/摄像头)')
parser.add_argument('--thresh', type=float, default=0.5, help='置信度阈值')
parser.add_argument('--detect-size', type=int, default=640, help='检测分辨率(默认640)')
args = parser.parse_args()

# 初始化模型
model = YOLO(args.model, task='detect')

# 初始化视频源
if args.source.isdigit():
    cap = cv2.VideoCapture(int(args.source))
elif 'usb' in args.source:
    cap = cv2.VideoCapture(int(args.source[3:]))
else:
    cap = cv2.VideoCapture(args.source)

# 串口初始化
ser = serial.Serial("/dev/ttyAMA0", 57600)

# 追踪器优化设置
tracker = None
tracking = False
last_detection_time = 0
DETECT_INTERVAL = 0.5  # 检测间隔(秒)
NO_OBJECT_THRESHOLD = 3  # 丢失目标阈值(秒)

# 主循环
while True:
    start_time = time.perf_counter()

    # 1. 读取帧
    ret, frame = cap.read()
    if not ret: break

    # 2. 降低检测分辨率
    detect_img = cv2.resize(frame, (args.detect_size, args.detect_size))
    h, w = frame.shape[:2]

    # 3. 间隔检测或初始化追踪器
    current_time = time.time()
    if not tracking or current_time - last_detection_time > DETECT_INTERVAL:
        # YOLO检测
        results = model(detect_img, verbose=False, imgsz=args.detect_size)
        detections = results[0].boxes

        if len(detections) > 0:
            # 找最大面积的检测框
            max_area = 0
            best_box = None
            for box in detections:
                if box.conf.item() > args.thresh:  # 注意这里使用了.item()
                    x1, y1, x2, y2 = map(int, box.xyxy.cpu().numpy().squeeze())
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        best_box = box

            if best_box is not None:
                # 转换坐标到原始分辨率
                x1, y1, x2, y2 = map(int, best_box.xyxy.cpu().numpy().squeeze())
                scale_x, scale_y = w / args.detect_size, h / args.detect_size
                x1, x2 = int(x1 * scale_x), int(x2 * scale_x)
                y1, y2 = int(y1 * scale_y), int(y2 * scale_y)

                # 更新追踪器
                if tracker is None:
                    tracker = cv2.TrackerCSRT_create()
                tracker.init(frame, (x1, y1, x2 - x1, y2 - y1))

                tracking = True
                last_detection_time = current_time

                # 计算中心点
                center_x = (x1 + x2) / 2 - w / 2
                center_y = (y1 + y2) / 2 - h / 2

                # 发送串口数据
                ser.write(f"&{center_x:.1f},{center_y:.1f}#\r\n".encode())

                # 绘制检测框（修正了置信度显示问题）
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{model.names[int(best_box.cls)]} {best_box.conf.item():.2f}",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 4. 追踪模式
    elif tracking:
        success, bbox = tracker.update(frame)
        if success:
            x, y, w, h = map(int, bbox)
            center_x = x + w / 2 - frame.shape[1] / 2
            center_y = y + h / 2 - frame.shape[0] / 2

            # 发送串口数据
            ser.write(f"&{center_x:.1f},{center_y:.1f}#\r\n".encode())

            # 绘制追踪框
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, "Tracking", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        elif current_time - last_detection_time > NO_OBJECT_THRESHOLD:
            tracking = False

    # 5. 显示结果
    fps = 1 / (time.perf_counter() - start_time)
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("Optimized Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
ser.close()
GPIO.output(26, GPIO.LOW)
GPIO.cleanup()