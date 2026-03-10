import os
import socket
import select
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from threading import Thread

class StandardPublisher(Node):
    def __init__(self, thruster_count=8):
        super().__init__('standard_publisher')
        self.thruster_count = thruster_count
        self.should_run = True

        # Publishers
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Service client
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.is_armed = False
        self.arm_timer = self.create_timer(0.5, self.arm_timer_callback)

        # TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('localhost', 12345))
        self.sock.listen(1)
        self.sock.settimeout(0.5)  # allow graceful shutdown
        self.conn = None

    def start(self):
        while self.should_run:
            try:
                self.conn, _ = self.sock.accept()
                self.conn.settimeout(0.1)
                buffer = ''
                self.get_logger().info("Client connected.")
                while self.should_run:
                    ready = select.select([self.conn], [], [], 0.02)
                    if ready[0]:
                        data = self.conn.recv(1024).decode()
                        if not data:
                            break
                        buffer += data
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self.handle_line(line)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")
            finally:
                if self.conn:
                    self.conn.close()
                    self.conn = None
                    self.get_logger().info("Client disconnected.")

    def arm_timer_callback(self):
        if self.is_armed:
            self.arm_vehicle(True)

    def handle_line(self, line):
        try:
            topic_id, values = line.split(':', 1)
            topic_id = int(topic_id)
            if topic_id == 200:
                arr = [int(float(v)) for v in values.split(',')]
                msg = OverrideRCIn()
                channels = [1500]*18
                fixed_indices_values = {
                    7: 1500,
                    8: 1500,
                    9: 0,
                    10: 0,
                    11: 1633,
                    12: 1100,
                    13: 0,
                    14: 0,
                    15: 0,
                    16: 0,
                    17: 0,
                }
                for idx, val in fixed_indices_values.items():
                    channels[idx] = val
                for i in range(min(len(arr), len(channels))):
                    channels[i] = 1500 + arr[i]
                msg.channels = channels
                self.rc_override_pub.publish(msg)
                now_ns = self.get_clock().now().nanoseconds
                self.get_logger().info(
                    f"[ROS_TIME_NS={now_ns}] Published RC override"
                    f"topic={topic_id} values={values} "
                    f"channels={channels}"
                )
            elif topic_id == 301:
                self.is_armed = True
                self.arm_vehicle(True)
            elif topic_id == 302:
                self.is_armed = False
                self.arm_vehicle(False)      
        except Exception as e:
            self.get_logger().error(f"Error parsing line '{line}': {e}")

    def arm_vehicle(self, value: bool):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Arming service not available")
            return
        req = CommandBool.Request()
        req.value = value
        future = self.arm_client.call_async(req)
        self.get_logger().info(f"Arming command sent: {value}")

    def destroy_node(self):
        self.should_run = False
        super().destroy_node()

def main():
    os.environ.pop('RMW_IMPLEMENTATION', None)
    rclpy.init()
    node = StandardPublisher(thruster_count=6)
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.start()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down server...")
    finally:
        node.should_run = False
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
