import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from crawler_control.core.config_loader import load_json_config
from crawler_control.core.dac import DacMCP4725


class DacNode(Node):
    def __init__(self):
        super().__init__('dac_node')

        # ---- Cargar config del DAC ----
        try:
            cfg = load_json_config("dac_config.json")
            simulate = bool(cfg.get("simulate", False))
            i2c_bus = int(cfg.get("i2c_bus", 1))
            i2c_address = int(cfg.get("i2c_address", 96))  # 96 = 0x60
            vref = float(cfg.get("vref", 5.0))
            v_min = float(cfg.get("v_min", 0.0))
            v_max = float(cfg.get("v_max", vref))
        except Exception as e:
            self.get_logger().fatal(f"Error cargando dac_config.json: {e}")
            raise

        # ---- Inicializar DAC ----
        try:
            self.dac = DacMCP4725(
                i2c_bus=i2c_bus,
                address=i2c_address,
                vref=vref,
                v_min=v_min,
                v_max=v_max,
                simulate=simulate
            )
        except Exception as e:
            self.get_logger().fatal(f"Error inicializando DAC MCP4725: {e}")
            raise

        # ---- Subscripción a comando motor ----
        self.sub_cmd = self.create_subscription(
            Float32,
            'motor_cmd',          # topic de entrada
            self.cmd_callback,
            10
        )

        # ---- Publicador de voltaje aplicado ----
        self.pub_voltage = self.create_publisher(
            Float32,
            'crawler/motor/analogic_signal',
            10
        )

        self.get_logger().info(
            f"DacNode iniciado | bus={i2c_bus}, addr=0x{i2c_address:02X}, "
            f"vref={vref}, v_min={v_min}, v_max={v_max} | sub=/motor_cmd pub=/crawler/dac_voltage"
        )

    def cmd_callback(self, msg: Float32):
        cmd = float(msg.data)

        # Enviar comando al DAC
        try:
            v_applied = self.dac.set_from_cmd(cmd)
        except Exception as e:
            self.get_logger().error(f"Error escribiendo al DAC: {e}")
            return

        # Publicar voltaje aplicado
        out = Float32()
        out.data = float(v_applied)
        self.pub_voltage.publish(out)

        self.get_logger().info(f"motor_cmd={cmd:+.3f} -> V_DAC={v_applied:.3f} V")


def main(args=None):
    rclpy.init(args=args)
    node = DacNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
