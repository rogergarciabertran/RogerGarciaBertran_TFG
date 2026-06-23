import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from crawler_control.core.config_loader import load_json_config
from crawler_control.core.fg import FGReader


class FGNode(Node):
    def __init__(self):
        super().__init__("fg_node")

        cfg = load_json_config("fg_config.json")

        self.reader = FGReader(
            fg_pin=cfg["fg_pin"],
            pulses_per_rev=cfg["pulses_per_rev"],
            window_s=cfg["window_s"],
            simulate=cfg.get("simulate", True),
        )

        # Publicadores
        self.pub_rpm = self.create_publisher(Float32, "/crawler/rpm", 10)
        self.pub_freq = self.create_publisher(Float32, "/crawler/fg_freq_hz", 10)

        self.create_timer(0.05, self.update)

        self.get_logger().info(
            f"FGNode iniciado | pin={cfg['fg_pin']} pulses/rev={cfg['pulses_per_rev']} simulate={cfg.get('simulate', True)}"
        )

    def update(self):
        # Necesitamos frecuencia; si tu FGReader solo da rpm, abajo te digo qué cambiar en FGReader
        freq = self.reader.read_freq_hz()
        if freq is None:
            return

        rpm = (freq * 60.0) / float(self.reader.pulses_per_rev)

        m_f = Float32()
        m_f.data = float(freq)
        self.pub_freq.publish(m_f)

        m_r = Float32()
        m_r.data = float(rpm)
        self.pub_rpm.publish(m_r)


def main():
    rclpy.init()
    node = FGNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
