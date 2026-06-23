import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus2

from crawler_control.core.config_loader import load_json_config


def clamp(x, a, b):
    return max(a, min(b, x))


class DacDualNode(Node):
    def __init__(self):
        super().__init__("dac_dual_node")

        cfg = load_json_config("dac_dual_config.json")

        self.simulate = bool(cfg.get("simulate", False))
        self.i2c_bus = int(cfg.get("i2c_bus", 1))
        self.addr_m1 = int(cfg.get("addr_motor1", 0x60))
        self.addr_m2 = int(cfg.get("addr_motor2", 0x61))

        self.vref = float(cfg.get("vref", 5.0))
        self.vmax = float(cfg.get("vmax", 5.0))
        self.gain_m1 = float(cfg.get("gain_motor1", 1.0))  # motor1 (izq)
        self.gain_m2 = float(cfg.get("gain_motor2", 1.0))  # motor2 (der)

        # entradas: comandos [-1..1]
        self.sub_m1 = self.create_subscription(Float32, "/crawler/motor1/cmd", self.cb_m1, 10)
        self.sub_m2 = self.create_subscription(Float32, "/crawler/motor2/cmd", self.cb_m2, 10)

        # salidas: voltajes aplicados
        self.pub_v1 = self.create_publisher(Float32, "/crawler/motor1/dac_v", 10)
        self.pub_v2 = self.create_publisher(Float32, "/crawler/motor2/dac_v", 10)

        self.cmd1 = 0.0
        self.cmd2 = 0.0

        if not self.simulate:
            self.bus = smbus2.SMBus(self.i2c_bus)
        else:
            self.bus = None

        # Publica a 20 Hz (aplica últimos cmds)
        self.create_timer(0.05, self.update)

        self.get_logger().info(
            f"DacDualNode listo | bus={self.i2c_bus} addr1=0x{self.addr_m1:02X} addr2=0x{self.addr_m2:02X} vref={self.vref} vmax={self.vmax} simulate={self.simulate}"
        )

    def cb_m1(self, msg: Float32):
        self.cmd1 = float(msg.data)

    def cb_m2(self, msg: Float32):
        self.cmd2 = float(msg.data)

    def _write_dac(self, addr: int, volts: float) -> float:
        volts = clamp(volts, 0.0, self.vmax)
        code = int((volts / self.vref) * 4095)
        code = clamp(code, 0, 4095)
        high = (code >> 8) & 0x0F
        low = code & 0xFF
        self.bus.write_i2c_block_data(addr, high, [low])
        return float(volts)

    def update(self):
        # cmd [-1..1] -> volts [0..vmax]
        v1 = abs(self.cmd1) * self.vmax * self.gain_m1
        v2 = abs(self.cmd2) * self.vmax * self.gain_m2

        if self.simulate:
            v1_applied, v2_applied = v1, v2
        else:
            try:
                v1_applied = self._write_dac(self.addr_m1, v1)
                v2_applied = self._write_dac(self.addr_m2, v2)
            except Exception as e:
                self.get_logger().error(f"Error escribiendo DAC: {e}")
                return

        m = Float32()
        m.data = float(v1_applied)
        self.pub_v1.publish(m)
        m.data = float(v2_applied)
        self.pub_v2.publish(m)

    def destroy_node(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DacDualNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()    
