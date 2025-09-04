import os
import json
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

class DesertSubscriber(Node):
    def __init__(self):
        super().__init__('desert_subscriber')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('localhost', 12345))
        self.sock.setblocking(False)

        # Thruster array subscriber
        self.create_subscription(
            Int32MultiArray,
            '/bluerov2/cmd_thrusters',
            self.thruster_callback,
            10
        )

        # Pose publisher
        self.pose_publisher = self.create_publisher(Pose, '/bluerov2/pose_gt', 10)

        # Timer for reading socket
        self.create_timer(1.0, self.read_socket)

    def thruster_callback(self, msg):
        try:
            data = "200:" + ",".join(str(v) for v in msg.data) + "\n"
            self.sock.sendall(data.encode())
        except Exception as e:
            self.get_logger().error(f"Error sending thrusters: {e}")

    def read_socket(self):
        try:
            data = self.sock.recv(1024).decode()
            if not data:
                return

            for line in data.strip().split('\n'):
                topic_id_str, value = line.split(':', 1)
                topic_id = int(topic_id_str)

                if topic_id == 100:  # pose
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
