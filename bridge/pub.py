import os
import json
import socket
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from threading import Thread
from time import time

class StandardPublisher(Node):
    def __init__(self):
        super().__init__('standard_publisher')
        self.topic_ids = {}

        with open('ros_allowed_topics.conf', 'r') as f:
            try:
                config = json.load(f)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Invalid JSON: {e}")
                raise

        self.topic_publishers = {}
        for topic_name, topic_id in config['topics'].items():
            self.topic_ids[int(topic_id)] = topic_name
            if topic_id == 100:
                self.pose_sub = self.create_subscription(
                    Pose,
                    '/bluerov2/pose_gt',
                    self.pose_callback,
                    10
                )
                self.last_sent_pose_time = 0
            else:
                self.topic_publishers[int(topic_id)] = self.create_publisher(Float64, topic_name, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('localhost', 12345))
        self.sock.listen(1)
        self.conn = None

    def pose_callback(self, msg):
        now = time()
        if self.conn and now - self.last_sent_pose_time >= 10.0:
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            data = f"100:{x},{y},{z}\n"
            try:
                self.conn.sendall(data.encode())
                self.last_sent_pose_time = now
            except Exception as e:
                self.get_logger().error(f"Error sending pose_gt: {e}")

    def start(self):
        while True:
            self.conn, _ = self.sock.accept()
            buffer = ''
            try:
                while True:
                    ready = select.select([self.conn], [], [], 0.1)
                    if ready[0]:
                        data = self.conn.recv(1024).decode()
                        if not data:
                            self.conn.close()
                            self.conn = None
                            break
                        buffer += data

                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            try:
                                topic_id, value = line.split(':', 1)
                                msg = Float64()
                                msg.data = float(value)
                                self.topic_publishers[int(topic_id)].publish(msg)
                            except Exception as e:
                                self.get_logger().error(f"Error parsing: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")
            finally:
                self.conn.close()
                self.conn = None
                self.get_logger().info("Client disconnected.")

def main():
    os.environ.pop('RMW_IMPLEMENTATION', None)
    rclpy.init()
    node = StandardPublisher()
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    node.start()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
