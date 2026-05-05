import time
import serial
import argparse
from ultralytics import YOLO
import numpy as np
import RPi.GPIO as GPIO
import cv2

# Configuration parameters
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 57600
SEND_INTERVAL = 0.1  # seconds
DEFAULT_MODEL = "best_ncnn_model"
DEFAULT_SOURCE = "0"
DEFAULT_SIZE = 320
TRIGGER_PIN_1 = 26  # BCM26 pin
TRIGGER_PIN_2 = 16  # BCM16 pin (additional)
TRIGGER_DURATION = 2.0  # trigger duration (seconds)
OUTPUT_DURATION = 1.5  # output duration (seconds)


class ObjectDetectionSystem:
    def __init__(self, model_path, source, detect_size):
        # Initialize size parameters first
        self.detect_size = (detect_size, detect_size)
        self.scale = np.zeros(2, dtype=np.float32)

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIGGER_PIN_1, GPIO.OUT)
        GPIO.setup(TRIGGER_PIN_2, GPIO.OUT)
        GPIO.output(TRIGGER_PIN_1, GPIO.LOW)
        GPIO.output(TRIGGER_PIN_2, GPIO.LOW)

        # Initialize serial port and camera
        self.ser = self._init_serial()
        self.cap = self._init_camera(source)  # self.detect_size is now available

        # Initialize model
        self.model = YOLO(model_path, task='detect')

        # State variables
        self.center_in_box_start_time = None
        self.output_start_time = None
        self.frame_count = 0
        self.fps = 0
        self.last_fps_time = time.time()

    def _init_serial(self):
        """Initialize serial port"""
        try:
            ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            print(f"[SER] Port {SERIAL_PORT} opened")
            return ser
        except Exception as e:
            print(f"[SER] Error: {e}")
            return None

    def _init_camera(self, source):
        """Initialize camera"""
        idx = int(source[3:]) if source.startswith('usb') else int(source) if source.isdigit() else source
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open video source {source}")

        # Get original image dimensions and calculate scale factor
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.scale = np.array([w / self.detect_size[0], h / self.detect_size[1]])
        print(f"[CAM] Source {source} opened (Original: {w}x{h}, Detect: {self.detect_size[0]}x{self.detect_size[1]})")
        return cap

    def _is_center_in_box(self, box):
        """Check if center point is within detection box"""
        center_x = self.detect_size[0] / 2
        center_y = self.detect_size[1] / 2
        return (box[0] <= center_x <= box[2]) and (box[1] <= center_y <= box[3])

    def _update_gpio_state(self):
        """Update GPIO output states"""
        current_time = time.time()

        # Handle output turn-off
        if self.output_start_time is not None:
            if current_time - self.output_start_time >= OUTPUT_DURATION:
                GPIO.output(TRIGGER_PIN_1, GPIO.LOW)
                GPIO.output(TRIGGER_PIN_2, GPIO.LOW)
                self.output_start_time = None
                print("[GPIO] Outputs turned OFF")

        # Handle trigger condition
        if self.center_in_box_start_time is not None:
            if current_time - self.center_in_box_start_time >= TRIGGER_DURATION:
                if self.output_start_time is None:
                    GPIO.output(TRIGGER_PIN_1, GPIO.HIGH)
                    GPIO.output(TRIGGER_PIN_2, GPIO.HIGH)
                    self.output_start_time = current_time
                    print("[GPIO] Outputs turned ON")

    def _send_data(self, box):
        """Send relative coordinates in detection size (centered)"""
        if not self.ser: return

        # Center offset in detection size (relative to image center)
        center_x = (box[0] + box[2]) * 0.5 - self.detect_size[0] * 0.5
        center_y = (box[1] + box[3]) * 0.5 - self.detect_size[1] * 0.5

        # Send data (format: &X.XX,Y.YY#)
        data_str = f"&{center_x:.2f},{center_y:.2f}#\n"
        self.ser.write(data_str.encode('utf-8'))
        print(f"[SER] Sent: {data_str.strip()}")

        # Debug output (optional)
        raw_x = (box[0] + box[2]) * 0.5 * self.scale[0]
        raw_y = (box[1] + box[3]) * 0.5 * self.scale[1]
        print(f"[DEBUG] Box: [{box[0]:.1f},{box[1]:.1f},{box[2]:.1f},{box[3]:.1f}]")
        print(f"[DEBUG] Center (Detect): {center_x:.1f}, {center_y:.1f}")
        print(f"[DEBUG] Center (Raw): {raw_x:.1f}, {raw_y:.1f}")

    def run(self):
        """Main detection loop"""
        last_sent = 0
        while True:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                # Calculate FPS
                self.frame_count += 1
                if time.time() - self.last_fps_time >= 1.0:
                    self.fps = self.frame_count / (time.time() - self.last_fps_time)
                    self.frame_count = 0
                    self.last_fps_time = time.time()
                    print(f"[FPS] {self.fps:.1f}")

                # Resize image to detection size
                detect_img = cv2.resize(frame, self.detect_size)

                # Run detection
                results = self.model(detect_img, verbose=False, imgsz=self.detect_size[0])
                boxes = results[0].boxes

                if len(boxes) > 0:
                    confs = boxes.conf.cpu().numpy()
                    classes = boxes.cls.cpu().numpy()
                    mask = confs > 0.5  # Confidence threshold

                    if np.any(mask):
                        valid_boxes = boxes.xyxy.cpu().numpy()[mask]
                        valid_confs = confs[mask]
                        valid_classes = classes[mask]

                        # Select box with largest area
                        areas = (valid_boxes[:, 2] - valid_boxes[:, 0]) * (valid_boxes[:, 3] - valid_boxes[:, 1])
                        best_idx = np.argmax(areas)
                        best_box = valid_boxes[best_idx]

                        # Update center point state
                        if self._is_center_in_box(best_box):
                            if self.center_in_box_start_time is None:
                                self.center_in_box_start_time = time.time()
                        else:
                            self.center_in_box_start_time = None

                        # Send data
                        if time.time() - last_sent > SEND_INTERVAL:
                            self._send_data(best_box)
                            last_sent = time.time()
                else:
                    self.center_in_box_start_time = None

                # Update GPIO
                self._update_gpio_state()

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[ERR] {e}")
                time.sleep(0.1)

    def __del__(self):
        """Cleanup resources"""
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

    app = ObjectDetectionSystem(opt.model, opt.source, opt.detect_size)
    app.run()