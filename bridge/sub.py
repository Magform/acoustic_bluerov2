import os
import json
import socket
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose

class DesertSubscriber(Node):
    def __init__(self):
        super().__init__('desert_subscriber')

        self.sock = self.connect_to_pub()
        self.buffer = ''

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
        self.create_timer(0.1, self.read_socket)

    def connect_to_pub(self):
        # Wait for pub.py instead of crashing if it isn't up yet
        while True:
            try:
                sock = socket.create_connection(('localhost', 12345))
                sock.setblocking(False)
                self.get_logger().info("Connected to pub.py")
                return sock
            except ConnectionRefusedError:
                self.get_logger().warn("pub.py not reachable, retrying in 2s...")
                time.sleep(2)

    def thruster_callback(self, msg):
        self.get_logger().info(f"Received thrusters from acoustic link: {list(msg.data)}")
        try:
            data = "200:" + ",".join(str(v) for v in msg.data) + "\n"
            self.sock.sendall(data.encode())
            self.get_logger().info("Sent thrusters to pub.py")
        except Exception as e:
            self.get_logger().error(f"Error sending thrusters: {e}")

    def read_socket(self):
        try:
            while True:
                data = self.sock.recv(1024).decode()
                if not data:
                    self.get_logger().warn("pub.py disconnected, reconnecting...")
                    self.sock.close()
                    self.sock = self.connect_to_pub()
                    self.buffer = ''
                    return
                self.buffer += data
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error receiving from socket: {e}")

        # Only handle complete lines, keep partial ones for the next read
        while '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            try:
                topic_id_str, value = line.split(':', 1)
                topic_id = int(topic_id_str)

                if topic_id == 100:  # pose
                    x, y, z = map(float, value.split(','))
                    msg = Pose()
                    msg.position.x = x
                    msg.position.y = y
                    msg.position.z = z
                    self.pose_publisher.publish(msg)
                    self.get_logger().info(f"Received pose from pub.py, sent to acoustic link: x={x:.2f} y={y:.2f} z={z:.2f}")
            except Exception as e:
                self.get_logger().error(f"Error parsing '{line}': {e}")

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
