import os
import socket
import select
import yaml
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

        # Failsafe: stop the thrusters if no command arrives for too long (link lost)
        try:
            with open('config.yaml') as f:
                self.failsafe_timeout = float(yaml.safe_load(f).get('failsafe_timeout', 0))
        except Exception:
            self.failsafe_timeout = 0.0
        self.last_cmd_time = time()
        self.thrusters_active = False
        if self.failsafe_timeout > 0:
            self.create_timer(1.0, self.failsafe_check)

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
                self.get_logger().info(f"Sent pose to sub.py: x={x:.2f} y={y:.2f} z={z:.2f}")
            except Exception as e:
                self.get_logger().error(f"Error sending pose_gt: {e}")

    def publish_thrusters(self, arr):
        for i, v in enumerate(arr):
            if i < len(self.thruster_split_pubs):
                m = Float64()
                m.data = float(v)
                self.thruster_split_pubs[i].publish(m)

    def failsafe_check(self):
        if self.thrusters_active and time() - self.last_cmd_time > self.failsafe_timeout:
            self.get_logger().warn(f"No command for {self.failsafe_timeout:.0f}s, failsafe: stopping all thrusters")
            self.publish_thrusters([0.0] * self.thruster_count)
            self.thrusters_active = False

    def start(self):
        while True:
            self.conn, _ = self.sock.accept()
            self.get_logger().info("Client connected.")
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
                                    self.get_logger().info(f"Received thrusters from sub.py, publishing to Gazebo: {arr}")
                                    self.publish_thrusters(arr)
                                    self.last_cmd_time = time()
                                    self.thrusters_active = any(v != 0 for v in arr)
                            except Exception as e:
                                self.get_logger().error(f"Error parsing: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")
            finally:
                if self.conn:
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
