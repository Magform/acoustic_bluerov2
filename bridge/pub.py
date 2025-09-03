import os
import socket
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose
from threading import Thread
from time import time

class StandardPublisher(Node):
    def __init__(self, thruster_count=6):
        super().__init__('standard_publisher')

        self.thruster_count = thruster_count

        # Publisher for the full array
        self.thruster_pub = self.create_publisher(Float64MultiArray, '/bluerov2/cmd_thrusters', 10)

        # Publishers for individual thrusters
        self.thruster_split_pubs = [
            self.create_publisher(Float64, f'/bluerov2/cmd_thruster{i+1}', 10)
            for i in range(thruster_count)
        ]

        # Pose publisher (unchanged)
        self.pose_sub = self.create_subscription(
            Pose,
            '/bluerov2/pose_gt',
            self.pose_callback,
            10
        )
        self.last_sent_pose_time = 0

        # TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('localhost', 12345))
        self.sock.listen(1)
        self.conn = None

    def pose_callback(self, msg):
        now = time()
        if self.conn and now - self.last_sent_pose_time >= 10.0:
            x, y, z = msg.position.x, msg.position.y, msg.position.z
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
                                topic_id, values = line.split(':', 1)
                                if int(topic_id) == 200:  # thruster array
                                    arr = [float(v) for v in values.split(',')]
                                    msg = Float64MultiArray()
                                    msg.data = arr
                                    self.thruster_pub.publish(msg)

                                    # Also publish split values
                                    for i, v in enumerate(arr):
                                        if i < len(self.thruster_split_pubs):
                                            m = Float64()
                                            m.data = v
                                            self.thruster_split_pubs[i].publish(m)
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
    node = StandardPublisher(thruster_count=6)
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    node.start()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
