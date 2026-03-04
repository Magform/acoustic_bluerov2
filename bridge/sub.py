import os
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

THRUSTER_CMD = "200"
ARM_CMD = "301"
DISARM_CMD = "302"

class DesertSubscriber(Node):
    def __init__(self):
        super().__init__('desert_subscriber')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('localhost', 12345))
        self.sock.setblocking(True)

        self.create_subscription(Int32MultiArray, '/bluerov2/cmd_thrusters', self.thruster_callback, 10)
        self.create_subscription(Bool, '/bluerov2/arm', self.arm_callback, 10)
        self.create_subscription(Bool, '/bluerov2/disarm', self.disarm_callback, 10)

    def thruster_callback(self, msg):
        try:
            data = f"{THRUSTER_CMD}:" + ",".join(str(v) for v in msg.data) + "\n"
            self.sock.sendall(data.encode())
        except Exception as e:
            self.get_logger().error(f"Error sending thrusters: {e}")

    def arm_callback(self, msg):
        try:
            self.sock.sendall(f"{ARM_CMD}:\n".encode())
        except Exception as e:
            self.get_logger().error(f"Error sending arm: {e}")

    def disarm_callback(self, msg):
        try:
            self.sock.sendall(f"{DISARM_CMD}:\n".encode())
        except Exception as e:
            self.get_logger().error(f"Error sending disarm: {e}")

def main():
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_desert'
    os.environ['DESERT_PORT'] = '4000'
    rclpy.init()
    node = DesertSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.sock.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
