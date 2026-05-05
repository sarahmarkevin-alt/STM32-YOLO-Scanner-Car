import cv2
import argparse
import os
import time
from datetime import datetime
import sys


def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='树莓派视频存储程序')
    parser.add_argument('--source', type=str, default='/dev/video0', help='视频源设备路径')
    parser.add_argument('--width', type=int, default=1280, help='视频宽度')
    parser.add_argument('--height', type=int, default=720, help='视频高度')
    parser.add_argument('--fps', type=int, default=30, help='帧率')
    parser.add_argument('--duration', type=int, default=0, help='录制时长(秒)，0表示无限录制')
    parser.add_argument('--output', type=str, default='', help='输出文件路径，默认为桌面')

    args = parser.parse_args()

    # 设置输出路径
    if not args.output:
        # 获取桌面路径
        desktop_path = os.path.join(os.path.expanduser('~'), 'Desktop')
        # 创建带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(desktop_path, f"video_{timestamp}.avi")
    else:
        output_file = args.output

    print(f"视频源: {args.source}")
    print(f"分辨率: {args.width}x{args.height}")
    print(f"帧率: {args.fps} FPS")
    print(f"输出文件: {output_file}")

    # 创建视频捕获对象
    cap = cv2.VideoCapture(args.source)

    if not cap.isOpened():
        print(f"错误: 无法打开视频源 {args.source}")
        sys.exit(1)

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # 设置帧率
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    # 获取实际设置值
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"实际分辨率: {actual_width}x{actual_height}")
    print(f"实际帧率: {actual_fps:.2f} FPS")

    # 创建视频编码器
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_file, fourcc, actual_fps, (actual_width, actual_height))

    if not out.isOpened():
        print(f"错误: 无法创建输出文件 {output_file}")
        cap.release()
        sys.exit(1)

    print("开始录制... (按 'q' 键停止)")

    start_time = time.time()
    frame_count = 0

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("错误: 无法读取帧")
                break

            # 写入帧
            out.write(frame)
            frame_count += 1

            # 显示预览
            cv2.imshow('录制中...', frame)

            # 检查按键或录制时间
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("用户手动停止录制")
                break

            # 检查录制时长
            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                print(f"达到指定录制时长: {args.duration}秒")
                break
    except KeyboardInterrupt:
        print("\n用户中断录制")
    finally:
        # 释放资源
        cap.release()
        out.release()
        cv2.destroyAllWindows()

        # 计算统计数据
        elapsed_time = time.time() - start_time
        actual_fps = frame_count / elapsed_time if elapsed_time > 0 else 0

        print("\n录制完成!")
        print(f"录制时长: {elapsed_time:.2f}秒")
        print(f"总帧数: {frame_count}")
        print(f"实际帧率: {actual_fps:.2f} FPS")
        print(f"文件大小: {os.path.getsize(output_file) / (1024 * 1024):.2f} MB")
        print(f"视频已保存至: {output_file}")


if __name__ == "__main__":
    main()