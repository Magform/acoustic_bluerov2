import os
import socket
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from mavros_msgs.msg import OverrideRCIn
from threading import Thread
from time import time

class StandardPublisher(Node):
    def __init__(self, thruster_count=8):
        super().__init__('standard_publisher')

        self.thruster_count = thruster_count

        # Publishers for individual thrusters
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('localhost', 12345))
        self.sock.listen(1)
        self.conn = None

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
                                if int(topic_id) == 200:
                                    arr = [int(float(v)) for v in values.split(',')]  # your new thruster values
                                    msg = OverrideRCIn()
                                    
                                    # Keep existing channels beyond the first 8 (if any)
                                    existing_channels = [0]*16  # default all 16 channels to 0
                                    existing_channels[11] = 1633
                                    existing_channels[12] = 1100

                                    # Update only the first 8 channels
                                    for i in range(len(arr)):
                                        existing_channels[i] = arr[i]
                                    
                                    msg.channels = existing_channels
                                    msg.is_override = True
                                    self.rc_override_pub.publish(msg)

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
    node = StandardPublisher(thruster_count=8)
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    node.start()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
