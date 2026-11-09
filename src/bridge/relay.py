import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32MultiArray

# Direct bridge (no DESERT): splits /bluerov2/cmd_thrusters into the
# per-thruster Float64 topics expected by Gazebo, all in one callback.
class ThrusterRelay(Node):
    def __init__(self, thruster_count=6):
        super().__init__('thruster_relay')

        self.thruster_pubs = [
            self.create_publisher(Float64, f'/bluerov2/cmd_thruster{i+1}', 10)
            for i in range(thruster_count)
        ]

        self.create_subscription(
            Int32MultiArray,
            '/bluerov2/cmd_thrusters',
            self.thruster_callback,
            10
        )

    def thruster_callback(self, msg):
        self.get_logger().info(f"Received thrusters, publishing to Gazebo: {list(msg.data)}")
        for pub, v in zip(self.thruster_pubs, msg.data):
            m = Float64()
            m.data = float(v)
            pub.publish(m)

def main():
    rclpy.init()
    node = ThrusterRelay(thruster_count=6)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
