# crawler_control/nodes/motor_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from crawler_control.core.motor import MotorController
from crawler_control.core.config_loader import load_json_config


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # 1) Cargar configuración desde motor_config.json
        try:
            cfg = load_json_config("motor_config.json")
            pwm_pin = int(cfg.get("pwm_pin", 18))
            dir_pin = int(cfg.get("dir_pin", 23))
            max_duty = float(cfg.get("max_duty", 1.0))
        except Exception as e:
            self.get_logger().fatal(f"Error cargando motor_config.json: {e}")
            raise

        # 2) Crear el controlador del motor
        self.motor = MotorController(pwm_pin, dir_pin, max_duty)

        # 3) Suscribirse al topic /motor_cmd
        self.sub_cmd = self.create_subscription(
            Float32,
            'motor_cmd',
            self.cmd_callback,
            10
        )

        self.get_logger().info(
            f"MotorNode iniciado (pwm_pin={pwm_pin}, dir_pin={dir_pin}, max_duty={max_duty})"
        )

    def cmd_callback(self, msg: Float32):
        """Callback que recibe la consigna de velocidad."""
        value = msg.data
        self.get_logger().info(f"motor_cmd recibido: {value:.2f}")
        self.motor.set_speed(value)


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
