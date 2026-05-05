import os
import sys
import argparse
import glob
import time
import serial
import cv2
import numpy as np
from ultralytics import YOLO
import RPi.GPIO as GPIO

# GPIO初始化
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT)
GPIO.output(26, GPIO.HIGH)

# 参数解析
parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file', required=True)
parser.add_argument('--source', help='Image source (file/folder/video/camera)', required=True)
parser.add_argument('--thresh', help='Confidence threshold', default=0.5)
parser.add_argument('--resolution', help='Display resolution WxH', default=None)
parser.add_argument('--record', help='Record output video', action='store_true')
args = parser.parse_args()

# 加载模型
model = YOLO(args.model, task='detect')
labels = model.names

# 输入源处理
img_ext_list = ['.jpg', '.jpeg', '.png', '.bmp']
vid_ext_list = ['.avi', '.mov', '.mp4', '.mkv']

if os.path.isdir(args.source):
    source_type = 'folder'
    imgs_list = [f for f in glob.glob(args.source + '/*') if os.path.splitext(f)[1] in img_ext_list]
elif os.path.isfile(args.source):
    ext = os.path.splitext(args.source)[1]
    if ext in img_ext_list:
        source_type = 'image'
        imgs_list = [args.source]
    elif ext in vid_ext_list:
        source_type = 'video'
    else:
        print(f'Unsupported file format: {ext}')
        sys.exit(0)
elif 'usb' in args.source:
    source_type = 'usb'
    usb_idx = int(args.source[3:])
elif 'picamera' in args.source:
    source_type = 'picamera'
    from picamera2 import Picamera2
else:
    print(f'Invalid source: {args.source}')
    sys.exit(0)

# 分辨率设置
resize = False
if args.resolution:
    resize = True
    resW, resH = map(int, args.resolution.split('x'))

# 视频录制设置
if args.record:
    if source_type not in ['video', 'usb'] or not args.resolution:
        print('Recording requires video source and resolution')
        sys.exit(0)
    recorder = cv2.VideoWriter('demo1.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (resW, resH))

# 初始化摄像头/视频
if source_type == 'video':
    cap = cv2.VideoCapture(args.source)
elif source_type == 'usb':
    cap = cv2.VideoCapture(usb_idx)
elif source_type == 'picamera':
    cap = Picamera2()
    cap.configure(cap.create_video_configuration(main={"format": 'XRGB8888', "size": (resW, resH)}))
    cap.start()

# 颜色和追踪设置
bbox_colors = [(164, 120, 87), (68, 148, 228), (93, 97, 209), (178, 182, 133)]
tracker = None
tracking = False
tracking_bbox = None
last_detection_time = time.time()
no_object_threshold = 20  # 20秒无检测视为丢失目标

# 串口初始化
ser = serial.Serial(port="/dev/ttyAMA0", baudrate=57600)

# 主循环
frame_rate_buffer = []
while True:
    t_start = time.perf_counter()

    # 获取帧
    if source_type in ['image', 'folder']:
        if not imgs_list: break
        frame = cv2.imread(imgs_list.pop(0))
    elif source_type == 'video':
        ret, frame = cap.read()
        if not ret: break
    elif source_type == 'usb':
        ret, frame = cap.read()
        if not ret: break
    elif source_type == 'picamera':
        frame = cv2.cvtColor(np.copy(cap.capture_array()), cv2.COLOR_BGRA2BGR)

    if resize:
        frame = cv2.resize(frame, (resW, resH))

    # YOLO检测
    results = model(frame, verbose=False)
    detections = results[0].boxes

    # 初始化变量
    x_label = y_label = 1500  # 默认值
    max_area = 0
    best_detection = None

    if len(detections) > 0:
        # 查找面积最大的有效检测框
        for detection in detections:
            xyxy = detection.xyxy.cpu().numpy().squeeze().astype(int)
            xmin, ymin, xmax, ymax = xyxy
            area = (xmax - xmin) * (ymax - ymin)

            if area > max_area and detection.conf.item() > args.thresh:
                max_area = area
                best_detection = detection

        # 处理最佳检测框
        if best_detection is not None:
            xyxy = best_detection.xyxy.cpu().numpy().squeeze().astype(int)
            xmin, ymin, xmax, ymax = xyxy

            # 更新追踪器
            last_detection_time = time.time()
            tracking_bbox = (xmin, ymin, xmax - xmin, ymax - ymin)
            if tracker is None:
                tracker = cv2.TrackerCSRT_create()
            tracker.init(frame, tracking_bbox)
            tracking = True

            # 计算中心点坐标（相对画面中心）
            middle_x = (xmin + xmax) / 2
            x_label = middle_x - (frame.shape[1] / 2)
            middle_y = (ymin + ymax) / 2
            y_label = middle_y - (frame.shape[0] / 2)

            # 绘制检测框
            classidx = int(best_detection.cls.item())
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), bbox_colors[classidx % 4], 2)
            label = f'{labels[classidx]}: {int(best_detection.conf.item() * 100)}% (Area: {max_area})'
            cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    # 追踪逻辑（当检测失效时）
    elif tracking and tracker is not None:
        success, bbox = tracker.update(frame)
        if success:
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, "Tracking", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            x_label = (x + w / 2) - (frame.shape[1] / 2)
            y_label = (y + h / 2) - (frame.shape[0] / 2)
        elif time.time() - last_detection_time > no_object_threshold:
            tracking = False

    # 串口通信
    if x_label != 1500 and y_label != 1500:
        command = f"&{x_label:.1f},{y_label:.1f}#\r\n"
        ser.write(command.encode('ascii'))
        print(f"Sent: {command.strip()} (Area: {max_area})")

    # 显示信息
    fps = 1 / (time.perf_counter() - t_start)
    frame_rate_buffer.append(fps)
    if len(frame_rate_buffer) > 200:
        frame_rate_buffer.pop(0)
    avg_fps = np.mean(frame_rate_buffer)

    cv2.putText(frame, f'FPS: {avg_fps:.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow('YOLO Tracking', frame)
    if args.record:
        recorder.write(frame)

    # 按键控制
    key = cv2.waitKey(5) if source_type != 'image' else cv2.waitKey(0)
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.waitKey(0)
    elif key == ord('p'):
        cv2.imwrite('capture.png', frame)

# 资源释放
ser.close()
GPIO.output(26, GPIO.LOW)
GPIO.cleanup()
if source_type in ['video', 'usb']:
    cap.release()
elif source_type == 'picamera':
    cap.stop()
if args.record:
    recorder.release()
cv2.destroyAllWindows()
print(f"Average FPS: {np.mean(frame_rate_buffer):.1f}")