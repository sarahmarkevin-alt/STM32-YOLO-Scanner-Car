import cv2
import os
import argparse  # For parsing command line arguments

# Fix Qt display issues
os.environ["QT_QPA_PLATFORM"] = "xcb"

# Default parameters (can be overridden by command line)
DEFAULT_SOURCE = "/dev/video0"  # Default camera device
DEFAULT_WIDTH = 1280           # Default width
DEFAULT_HEIGHT = 720           # Default height
SHUIPINGOFFSET = 35                    # Horizontal center point offset
SHUZHIOFFSET = -2

def main():
    # 1. Parse command line arguments
    parser = argparse.ArgumentParser(description="Camera Center Point Positioning Program")
    parser.add_argument("--source", type=str, default=DEFAULT_SOURCE, help="Camera device path (e.g., /dev/video0)")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Camera horizontal resolution")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Camera vertical resolution")
    args = parser.parse_args()

    # 2. Open camera and set resolution
    cap = cv2.VideoCapture(args.source)
    if not cap.isOpened():
        print(f" Unable to open camera device: {args.source}")
        return

    # Set camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f" Camera activated | Resolution: {actual_width}x{actual_height}")

    # 3. Create resizable window
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Feed", 1200, 900)  # Display window size (independent of camera resolution)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print(" Video stream read failed")
                break

            # Calculate and mark center point (with horizontal offset)
            h, w = frame.shape[:2]
            center_x = w // 2 - SHUIPINGOFFSET
            center_y = h // 2 + SHUZHIOFFSET
            cv2.circle(frame, (center_x, center_y), 1, (0, 0, 255), -1)  # Red solid circle

            # Display frame
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Ensure resource cleanup
        cap.release()
        cv2.destroyAllWindows()
        print(" Program exited")

if __name__ == "__main__":
    main()
