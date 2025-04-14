import os
import json
import socket
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class StandardPublisher(Node):
    def __init__(self):
        super().__init__('standard_publisher')

        # Carica configurazione
        with open('ros_allowed_topics.conf', 'r') as f:
            config = json.load(f)

        # Crea publishers
        self.topic_publishers = {}
        for topic_name, topic_id in config['topics'].items():
            self.topic_publishers[topic_id] = self.create_publisher(
                Float64,
                topic_name,
                10
            )

        # Configura socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('localhost', 12345))
        self.sock.listen(1)
        self.conn = None

    def start(self):
        self.conn, _ = self.sock.accept()
        buffer = ''
        while True:
            # Leggi dati senza bloccare ROS2
            ready = select.select([self.conn], [], [], 0.1)
            if ready[0]:
                data = self.conn.recv(1024).decode()
                if not data:
                    break  # Connessione chiusa
                buffer += data

                # Processa tutti i messaggi nel buffer
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        topic_id, value = line.split(':', 1)
                        msg = Float64()
                        msg.data = float(value)
                        print(f"Publishing: {msg}")
                        self.topic_publishers[int(topic_id)].publish(msg)
                    except Exception as e:
                        self.get_logger().error(f"Errore: {str(e)}")

def main():
    os.environ.pop('RMW_IMPLEMENTATION', None)
    rclpy.init()
    node = StandardPublisher()

    # Usa un thread separato per ROS2
    from threading import Thread
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    node.start()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()