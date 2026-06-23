import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio


class PowerRelayNode(Node):
    def __init__(self):
        super().__init__("power_relay_node")

        # GPIO reales usados para los relés Panasonic
        # Cambia estos pines si físicamente usas otros GPIO
        self.relay1_pin = 6
        self.relay2_pin = 5

        # En Raspberry Pi 5 normalmente se usa gpiochip4
        self.chip = 4
        self.h = lgpio.gpiochip_open(self.chip)

        # Relés apagados al arrancar por seguridad
        lgpio.gpio_claim_output(self.h, self.relay1_pin, 0)
        lgpio.gpio_claim_output(self.h, self.relay2_pin, 0)

        self.relay_state = False

        self.sub = self.create_subscription(
            Bool,
            "/crawler/power_enable",
            self.power_callback,
            10
        )

        self.get_logger().info(
            f"PowerRelayNode iniciado | relés apagados por defecto | "
            f"GPIO relés: {self.relay1_pin}, {self.relay2_pin}"
        )

    def set_relays(self, enabled: bool):
        state = 1 if enabled else 0

        lgpio.gpio_write(self.h, self.relay1_pin, state)
        lgpio.gpio_write(self.h, self.relay2_pin, state)

        self.relay_state = enabled

        if enabled:
            self.get_logger().info("Relés Panasonic ACTIVADOS. Controladores alimentados.")
        else:
            self.get_logger().info("Relés Panasonic DESACTIVADOS. Controladores sin alimentación.")

    def power_callback(self, msg: Bool):
        enabled = bool(msg.data)

        # Evita escribir continuamente si el estado no cambia
        if enabled == self.relay_state:
            return

        self.set_relays(enabled)

    def destroy_node(self):
        # Por seguridad, apaga relés al cerrar el nodo
        self.set_relays(False)
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PowerRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()  