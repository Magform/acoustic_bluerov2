import os
import json
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

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

    def thruster_callback(self, msg):
        try:
            data = "200:" + ",".join(str(v) for v in msg.data) + "\n"
            self.sock.sendall(data.encode())
        except Exception as e:
            self.get_logger().error(f"Error sending thrusters: {e}")

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
