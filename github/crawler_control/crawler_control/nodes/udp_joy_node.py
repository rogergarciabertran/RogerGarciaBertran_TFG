import json
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class UDPJoyNode(Node):
    def __init__(self):
        super().__init__('udp_joy_node')

        self.declare_parameter('port', 5005)
        self.declare_parameter('frame_id', 'udp_joy')

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Joy, '/joy', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.02, self.read_udp)

        self.get_logger().info(f'UDPJoyNode escuchando en puerto UDP {self.port}')

    def read_udp(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(4096)
                msg_json = json.loads(data.decode('utf-8'))

                joy_msg = Joy()
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                joy_msg.header.frame_id = self.frame_id

                joy_msg.axes = [float(x) for x in msg_json.get('axes', [])]
                joy_msg.buttons = [int(x) for x in msg_json.get('buttons', [])]

                self.pub.publish(joy_msg)

        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Error leyendo UDP: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UDPJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
