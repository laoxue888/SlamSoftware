import cv2
import paho.mqtt.client as mqtt
import numpy as np
import time
import argparse
import json
from datetime import datetime


def on_connect(client, userdata, flags, rc):
    """连接到MQTT服务器后的回调函数"""
    if rc == 0:
        print("已成功连接到MQTT服务器")
    else:
        print(f"连接失败，错误代码: {rc}")


def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='带时间戳的摄像头图像MQTT发布程序')
    parser.add_argument('--broker', type=str, default='localhost', help='MQTT服务器地址')
    parser.add_argument('--port', type=int, default=1883, help='MQTT服务器端口')
    parser.add_argument('--topic', type=str, default='camera/image', help='发布图像的MQTT话题')
    parser.add_argument('--camera', type=int, default=0, help='摄像头索引，默认0表示默认摄像头')
    parser.add_argument('--quality', type=int, default=90, help='JPEG图像质量，0-100')
    parser.add_argument('--interval', type=float, default=0.04, help='发布间隔时间(秒)')
    args = parser.parse_args()

    # 初始化MQTT客户端
    client = mqtt.Client()
    client.on_connect = on_connect

    # 连接到MQTT服务器
    print(f"连接到MQTT服务器: {args.broker}:{args.port}")
    client.connect(args.broker, args.port, 60)

    # 启动客户端网络循环
    client.loop_start()

    # 打开摄像头
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    try:
        print(f"开始发布图像到话题: {args.topic}，按Ctrl+C停止")
        while True:
            # 获取当前时间戳
            timestamp = datetime.now().isoformat()  # ISO 8601格式时间戳
            timestamp_ms = int(time.time() * 1000)  # 毫秒级时间戳

            # 读取一帧图像
            ret, frame = cap.read()
            if not ret:
                print("无法获取图像帧")
                break

            # 将图像转换为JPEG格式
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
            result, img_encoded = cv2.imencode('.jpg', frame, encode_param)

            if not result:
                print("图像编码失败")
                continue

            # 将编码的图像转换为字节流
            img_bytes = np.array(img_encoded).tobytes()

            # 准备要发送的数据 - 包含时间戳和图像数据
            # 注意：MQTT消息可以是字节，这里我们使用JSON格式存储元数据
            # 然后将元数据和图像数据一起发送
            metadata = {
                "timestamp": timestamp,
                "timestamp_ms": timestamp_ms,
                "image_length": len(img_bytes)
            }

            # 将元数据转换为JSON字节
            metadata_bytes = json.dumps(metadata).encode('utf-8')

            # 使用分隔符分隔元数据和图像数据
            separator = b'||SEPARATOR||'
            message = metadata_bytes + separator + img_bytes

            # 发布包含时间戳的图像数据
            client.publish(args.topic, message, qos=1)
            # print(f"已发布: {timestamp} (图像大小: {len(img_bytes)} bytes)")

            # 等待指定的时间间隔
            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        # 释放资源
        cap.release()
        client.loop_stop()
        client.disconnect()
        print("程序已退出")


if __name__ == "__main__":
    main()
