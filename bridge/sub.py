import os
import json
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

class DesertSubscriber(Node):
    def __init__(self):
        super().__init__('desert_subscriber')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('localhost', 12345))
        self.sock.setblocking(False)

        with open('ros_allowed_topics.conf', 'r') as f:
            try:
                config = json.load(f)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Invalid JSON: {e}")
                raise

        self.topic_ids = {}
        self.pose_publisher = None

        for topic_name, topic_id in config['topics'].items():
            self.topic_ids[int(topic_id)] = topic_name
            if topic_id == 100:
                self.pose_publisher = self.create_publisher(Pose, topic_name, 10)
            else:
                self.create_subscription(
                    Float64,
                    topic_name,
                    lambda msg, t_id=topic_id: self.callback(msg, t_id),
                    10
                )

        self.create_timer(1.0, self.read_socket)

    def callback(self, msg, topic_id):
        try:
            data = f"{topic_id}:{msg.data}\n"
            self.sock.sendall(data.encode())
        except Exception as e:
            self.get_logger().error(f"Error sending: {e}")

    def read_socket(self):
        try:
            data = self.sock.recv(1024).decode()
            if not data:
                return

            for line in data.strip().split('\n'):
                topic_id_str, value = line.split(':', 1)
                topic_id = int(topic_id_str)

                if topic_id == 100 and self.pose_publisher:
                    x, y, z = map(float, value.split(','))
                    msg = Pose()
                    msg.position.x = x
                    msg.position.y = y
                    msg.position.z = z
                    self.pose_publisher.publish(msg)

        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error receiving from socket: {e}")

def main():
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_desert'
    os.environ['DESERT_PORT'] = '4000'
    rclpy.init()
    node = DesertSubscriber()
    rclpy.spin(node)
    node.sock.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
