import os
import json
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class DesertSubscriber(Node):
    def __init__(self):
        super().__init__('desert_subscriber')

        # Configura socket persistente
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('localhost', 12345))  # Connessione unica

        # Carica configurazione
        with open('ros_allowed_topics.conf', 'r') as f:
            config = json.load(f)

        # Crea subscribers
        for topic_name, topic_id in config['topics'].items():
            self.create_subscription(
                Float64,
                topic_name,
                lambda msg, t_id=topic_id: self.callback(msg, t_id),
                10
            )

    def callback(self, msg, topic_id):
        try:
            data = f"{topic_id}:{msg.data}"
            self.sock.sendall(data.encode() + b'\n')  # Aggiungi delimitatore
        except Exception as e:
            self.get_logger().error(f"Errore invio: {str(e)}")

def main():
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_desert'
    os.environ['DESERT_PORT'] = '4000'
    rclpy.init()
    node = DesertSubscriber()
    rclpy.spin(node)
    node.sock.close()  # Chiudi socket alla chiusura
    rclpy.shutdown()

if __name__ == '__main__':
    main()