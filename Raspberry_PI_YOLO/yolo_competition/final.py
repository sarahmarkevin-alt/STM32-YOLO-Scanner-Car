import time
import serial
import argparse
from ultralytics import YOLO
import cv2
import numpy as np
import RPi.GPIO as GPIO

# 配置参数
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 57600
SEND_INTERVAL = 0.1  # 秒
DEFAULT_MODEL = "best_ncnn_model"
DEFAULT_SOURCE = "0"
DEFAULT_SIZE = 320
TRIGGER_PIN_1 = 26  # BCM26引脚
TRIGGER_PIN_2 = 16  # BCM16引脚（新增）
TRIGGER_DURATION = 2.0  # 触发持续时间(秒)
OUTPUT_DURATION = 1.5  # 输出持续时间(秒)


class ObjectDetectionSystem:
    def __init__(self, model_path, source, detect_size):
        # 初始化GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIGGER_PIN_1, GPIO.OUT)
        GPIO.setup(TRIGGER_PIN_2, GPIO.OUT)
        GPIO.output(TRIGGER_PIN_1, GPIO.LOW)
        GPIO.output(TRIGGER_PIN_2, GPIO.LOW)

        # 初始化串口
        self.ser = self._init_serial()

        # 初始化模型
        self.model = YOLO(model_path, task='detect')

        # 初始化摄像头
        self.cap = self._init_camera(source)
        self.detect_size = (detect_size, detect_size)
        self.scale = np.zeros(2, dtype=np.float32)

        # 状态变量
        self.center_in_box_start_time = None
        self.output_start_time = None
        self.frame_count = 0
        self.fps = 0
        self.last_fps_time = time.time()

        # 创建显示窗口
        cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)

    def _init_serial(self):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            print(f"[SER] Port {SERIAL_PORT} opened")
            return ser
        except Exception as e:
            print(f"[SER] Error: {e}")
            return None

    def _init_camera(self, source):
        idx = int(source[3:]) if source.startswith('usb') else int(source) if source.isdigit() else source
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open video source {source}")
        print(f"[CAM] Source {source} opened")
        return cap

    def _is_center_in_box(self, box):
        """检查中心点是否在检测框内"""
        center_x = self.detect_size[0] / 2
        center_y = self.detect_size[1] / 2
        return (box[0] <= center_x <= box[2]) and (box[1] <= center_y <= box[3])

    def _update_gpio_state(self):
        """更新GPIO状态"""
        current_time = time.time()

        # 处理输出关闭
        if self.output_start_time is not None:
            if current_time - self.output_start_time >= OUTPUT_DURATION:
                GPIO.output(TRIGGER_PIN_1, GPIO.LOW)
                GPIO.output(TRIGGER_PIN_2, GPIO.LOW)
                self.output_start_time = None
                print("[GPIO] Outputs turned OFF")
                return

        # 处理触发条件
        if self.center_in_box_start_time is not None:
            if current_time - self.center_in_box_start_time >= TRIGGER_DURATION:
                if self.output_start_time is None:
                    GPIO.output(TRIGGER_PIN_1, GPIO.HIGH)
                    GPIO.output(TRIGGER_PIN_2, GPIO.HIGH)
                    self.output_start_time = current_time
                    print("[GPIO] Outputs turned ON")

    def _draw_detection(self, frame, box, class_name, confidence):
        """绘制检测框和信息"""
        # 计算实际坐标
        x1 = int(box[0] * self.scale[0])
        y1 = int(box[1] * self.scale[1])
        x2 = int(box[2] * self.scale[0])
        y2 = int(box[3] * self.scale[1])

        # 绘制检测框
        color = (0, 255, 0)  # 绿色
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # 绘制标签
        label = f"{class_name}: {confidence:.2f}"
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 绘制中心点
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        # 绘制FPS
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 绘制GPIO状态
        gpio_state = "ON" if self.output_start_time is not None else "OFF"
        cv2.putText(frame, f"GPIO26: {gpio_state}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"GPIO16: {gpio_state}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return frame

    def run(self):
        last_sent = 0
        while True:
            try:
                # 1. 读取帧
                ret, frame = self.cap.read()
                if not ret:
                    continue

                # 计算FPS
                self.frame_count += 1
                if time.time() - self.last_fps_time >= 1.0:
                    self.fps = self.frame_count / (time.time() - self.last_fps_time)
                    self.frame_count = 0
                    self.last_fps_time = time.time()

                # 2. 缩放
                detect_img = cv2.resize(frame, self.detect_size)
                h, w = frame.shape[:2]
                self.scale[0], self.scale[1] = w / self.detect_size[0], h / self.detect_size[1]

                # 3. 推理
                results = self.model(detect_img, verbose=False, imgsz=self.detect_size[0])
                boxes = results[0].boxes

                # 4. 处理检测结果
                if len(boxes) > 0:
                    confs = boxes.conf.cpu().numpy()
                    classes = boxes.cls.cpu().numpy()
                    mask = confs > 0.5

                    if np.any(mask):
                        valid_boxes = boxes.xyxy.cpu().numpy()[mask]
                        valid_confs = confs[mask]
                        valid_classes = classes[mask]

                        # 选择面积最大的框
                        areas = (valid_boxes[:, 2] - valid_boxes[:, 0]) * (valid_boxes[:, 3] - valid_boxes[:, 1])
                        best_idx = np.argmax(areas)
                        best_box = valid_boxes[best_idx]
                        best_conf = valid_confs[best_idx]
                        best_class = valid_classes[best_idx]

                        # 更新中心点状态
                        if self._is_center_in_box(best_box):
                            if self.center_in_box_start_time is None:
                                self.center_in_box_start_time = time.time()
                        else:
                            self.center_in_box_start_time = None

                        # 发送数据
                        if time.time() - last_sent > SEND_INTERVAL:
                            self._send_data(best_box)
                            last_sent = time.time()

                        # 绘制检测结果
                        frame = self._draw_detection(frame, best_box,
                                                     self.model.names[int(best_class)],
                                                     best_conf)
                else:
                    self.center_in_box_start_time = None
                    # 显示无检测结果
                    cv2.putText(frame, "No detection", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # 更新GPIO状态
                self._update_gpio_state()

                # 显示画面
                cv2.imshow("Object Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[ERR] {e}")
                time.sleep(0.1)

    def _send_data(self, box):
        if not self.ser: return

        center = np.array([
            (box[0] + box[2]) * 0.5 * self.scale[0] - self.detect_size[0] * 0.5,
            (box[1] + box[3]) * 0.5 * self.scale[1] - self.detect_size[1] * 0.5
        ])

        data = bytearray(9)
        data[0] = 0x24
        data[1:5] = np.float32(center[0]).tobytes()
        data[5:9] = np.float32(center[1]).tobytes()

        try:
            self.ser.write(data)
        except Exception as e:
            print(f"[SER] Send failed: {e}")

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("[SYS] Resources released")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default=DEFAULT_MODEL, help='model path')
    parser.add_argument('--source', type=str, default=DEFAULT_SOURCE, help='video source')
    parser.add_argument('--detect-size', type=int, default=DEFAULT_SIZE, help='inference size (pixels)')
    opt = parser.parse_args()

    app = ObjectDetectionSystem(opt.model, opt.source, opt.detect_size)
    app.run()