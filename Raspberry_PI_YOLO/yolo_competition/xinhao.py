import time
import serial
import argparse
from ultralytics import YOLO
import cv2
import numpy as np
import RPi.GPIO as GPIO

# 硬编码参数（避免运行时解析）
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 57600
SEND_INTERVAL = 0.1  # 秒
DEFAULT_MODEL = "best_ncnn_model"
DEFAULT_SOURCE = "0"
DEFAULT_SIZE = 320
TRIGGER_PIN = 26  # BCM26引脚
TRIGGER_DURATION = 2.0  # 触发持续时间(秒)
OUTPUT_DURATION = 1.0  # 输出持续时间(秒)


class OptimizedYOLOSerial:
    def __init__(self, model_path, source, detect_size):
        # 初始化GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIGGER_PIN, GPIO.OUT)
        GPIO.output(TRIGGER_PIN, GPIO.LOW)

        # 初始化串口
        self.ser = self._init_serial()
        # 初始化模型和摄像头
        self.model = YOLO(model_path, task='detect')
        self.cap = self._init_camera(source)
        # 预分配内存
        self.detect_size = (detect_size, detect_size)
        self.scale = np.zeros(2, dtype=np.float32)

        # 新增状态变量
        self.center_in_box_start_time = None
        self.output_start_time = None

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

        # 如果正在输出高电平且时间已到，关闭输出
        if self.output_start_time is not None:
            if current_time - self.output_start_time >= OUTPUT_DURATION:
                GPIO.output(TRIGGER_PIN, GPIO.LOW)
                self.output_start_time = None
                print("[GPIO] Output turned OFF")
                return

        # 如果中心点在框内且持续时间足够，触发输出
        if self.center_in_box_start_time is not None:
            if current_time - self.center_in_box_start_time >= TRIGGER_DURATION:
                if self.output_start_time is None:
                    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
                    self.output_start_time = current_time
                    print("[GPIO] Output turned ON")

    def run(self):
        last_sent = 0
        while True:
            try:
                # 1. 读取帧
                ret, frame = self.cap.read()
                if not ret: continue

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
                    mask = confs > 0.5
                    if np.any(mask):
                        valid_boxes = boxes.xyxy.cpu().numpy()[mask]
                        areas = (valid_boxes[:, 2] - valid_boxes[:, 0]) * (valid_boxes[:, 3] - valid_boxes[:, 1])
                        best_idx = np.argmax(areas)
                        best_box = valid_boxes[best_idx]

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
                else:
                    self.center_in_box_start_time = None

                # 更新GPIO状态
                self._update_gpio_state()

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
        print("[SYS] Resources released")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default=DEFAULT_MODEL, help='model path')
    parser.add_argument('--source', type=str, default=DEFAULT_SOURCE, help='video source')
    parser.add_argument('--detect-size', type=int, default=DEFAULT_SIZE, help='inference size (pixels)')
    opt = parser.parse_args()

    app = OptimizedYOLOSerial(opt.model, opt.source, opt.detect_size)
    app.run()
