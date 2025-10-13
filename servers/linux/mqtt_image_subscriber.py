import cv2
import paho.mqtt.client as mqtt
import numpy as np
import argparse
import threading
from queue import Queue
import time
import json
from datetime import datetime

# 创建一个队列用于在MQTT回调和显示线程之间传递图像
image_queue = Queue(maxsize=10)

def on_connect(client, userdata, flags, rc):
    """连接到MQTT服务器后的回调函数"""
    if rc == 0:
        print("已成功连接到MQTT服务器")
        # 连接成功后订阅话题
        client.subscribe(args.topic)
        print(f"已订阅话题: {args.topic}")
    else:
        print(f"连接失败，错误代码: {rc}")

def on_message(client, userdata, msg):
    """收到MQTT消息时的回调函数"""
    try:
        # 分割元数据和图像数据（使用约定的分隔符）
        separator = b'||SEPARATOR||'
        parts = msg.payload.split(separator, 1)
        
        if len(parts) != 2:
            print("❌ 消息格式错误，未找到正确的分隔符")
            return
            
        metadata_bytes, img_bytes = parts
        
        # 解析元数据（包含时间戳）
        metadata = json.loads(metadata_bytes.decode('utf-8'))
        
        # 验证图像数据完整性
        if len(img_bytes) != metadata.get("image_length", 0):
            print(f"❌ 图像数据不完整，预期{metadata['image_length']}字节，实际{len(img_bytes)}字节")
            return
        
        # 解码图像
        nparr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if img is not None:
            # 计算接收延迟（毫秒）
            receive_time_ms = int(time.time() * 1000)
            delay_ms = receive_time_ms - metadata.get("timestamp_ms", receive_time_ms)
            
            # 将图像和元数据放入队列
            if image_queue.full():
                image_queue.get()
            image_queue.put((img, metadata, delay_ms))

            # 定期打印统计信息
            if hasattr(on_message, 'count'):
                on_message.count += 1
                if on_message.count % 10 == 0:
                    print(f"📊 已接收 {on_message.count} 帧，最新时间: {metadata['timestamp']}")
            else:
                on_message.count = 1
    except Exception as e:
        print(f"处理图像时出错: {e}")

def display_images():
    """显示图像的线程函数"""
    print("开始显示图像，按'q'键退出")
    while True:
        # 从队列获取图像
        img, metadata, delay_ms = image_queue.get()
        if img is not None:
            # 显示图像
            cv2.imshow('MQTT Camera Stream', img)
            
            # 检查是否按下'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        image_queue.task_done()
    
    # 清理资源
    cv2.destroyAllWindows()

def main():
    global args
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='MQTT图像接收与显示程序')
    parser.add_argument('--broker', type=str, default='172.17.0.3', help='MQTT服务器地址')
    parser.add_argument('--port', type=int, default=1883, help='MQTT服务器端口')
    parser.add_argument('--topic', type=str, default='camera/image', help='订阅图像的MQTT话题')
    args = parser.parse_args()

    # 启动显示图像的线程
    display_thread = threading.Thread(target=display_images, daemon=True)
    display_thread.start()

    # 初始化MQTT客户端
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    # 连接到MQTT服务器
    print(f"连接到MQTT服务器: {args.broker}:{args.port}")
    client.connect(args.broker, args.port, 60)

    try:
        # 保持客户端运行，处理网络流量和回调
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        client.disconnect()
        print("程序已退出")

if __name__ == "__main__":
    main()
    