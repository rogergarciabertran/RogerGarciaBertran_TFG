import time
import lgpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool

from crawler_control.core.config_loader import load_json_config


def clamp(x, a, b):
    return max(a, min(b, x))


class JoyDiffNode(Node):
    def __init__(self):
        super().__init__("joy_diff_node")

        cfg = load_json_config("joy_config.json")

        self.axis_throttle = int(cfg.get("axis_throttle", 1))
        self.axis_turn = int(cfg.get("axis_turn", 0))
        self.deadman_button = int(cfg.get("deadman_button", 4))
        self.deadzone = float(cfg.get("deadzone", 0.08))

        self.chip = int(cfg.get("gpiochip", 4))
        self.dir1_pin = int(cfg.get("dir1_pin", 23))
        self.dir2_pin = int(cfg.get("dir2_pin", 24))
        self.timeout_s = float(cfg.get("timeout_s", 0.6))

        # GPIO DIR motores
        self.h = lgpio.gpiochip_open(self.chip)
        lgpio.gpio_claim_output(self.h, self.dir1_pin, 1)
        lgpio.gpio_claim_output(self.h, self.dir2_pin, 1)

        # Publicadores de comando [-1..1]
        self.pub_m1 = self.create_publisher(Float32, "/crawler/motor1/cmd", 10)
        self.pub_m2 = self.create_publisher(Float32, "/crawler/motor2/cmd", 10)

        # Publicador para activar/desactivar relés Panasonic
        self.pub_power = self.create_publisher(Bool, "/crawler/power_enable", 10)
        self.power_enabled = False

        # Subscripción mando
        self.create_subscription(Joy, "/joy", self.on_joy, 10)

        self.last_joy = time.time()
        self.create_timer(0.05, self.watchdog)

        self.get_logger().info(
            f"JoyDiffNode listo | axis_thr={self.axis_throttle} "
            f"axis_turn={self.axis_turn} deadman={self.deadman_button} | "
            f"DIR pins: {self.dir1_pin},{self.dir2_pin}"
        )

    def set_dir(self, motor: int, forward: bool):
        pin = self.dir1_pin if motor == 1 else self.dir2_pin
        lgpio.gpio_write(self.h, pin, 1 if forward else 0)

    def publish_cmds(self, c1: float, c2: float):
        m1 = Float32()
        m2 = Float32()

        m1.data = float(c1)
        m2.data = float(c2)

        self.pub_m1.publish(m1)
        self.pub_m2.publish(m2)

    def set_power(self, enabled: bool):
        """
        Publica el estado de alimentación de los controladores.
        El power_relay_node recibe este tópico y activa/desactiva los relés.
        """
        if self.power_enabled == enabled:
            return

        self.power_enabled = enabled

        msg = Bool()
        msg.data = bool(enabled)
        self.pub_power.publish(msg)

        if enabled:
            self.get_logger().info("POWER ENABLE: relés Panasonic activados")
        else:
            self.get_logger().info("POWER DISABLE: relés Panasonic desactivados")

    def stop(self):
        """
        Parada segura:
        - comandos de motor a 0
        - dirección forward por defecto
        - relés apagados
        """
        self.publish_cmds(0.0, 0.0)
        self.set_dir(1, True)
        self.set_dir(2, True)
        self.set_power(False)

    def watchdog(self):
        """
        Si se pierde la señal del mando durante más de timeout_s,
        se paran los motores y se apagan los relés.
        """
        if time.time() - self.last_joy > self.timeout_s:
            self.stop()

    def on_joy(self, msg: Joy):
        self.last_joy = time.time()

        # Botón de seguridad / deadman
        deadman_pressed = (
            self.deadman_button < len(msg.buttons)
            and msg.buttons[self.deadman_button] == 1
        )

        if not deadman_pressed:
            self.stop()
            return

        # Si el botón de seguridad está pulsado, activa alimentación
        self.set_power(True)

        thr = msg.axes[self.axis_throttle] if self.axis_throttle < len(msg.axes) else 0.0
        trn = msg.axes[self.axis_turn] if self.axis_turn < len(msg.axes) else 0.0

        if abs(thr) < self.deadzone:
            thr = 0.0

        if abs(trn) < self.deadzone:
            trn = 0.0

        # Mezcla diferencial [-1..1]
        left = clamp(thr + trn, -1.0, 1.0)
        right = clamp(thr - trn, -1.0, 1.0)

        # DIR por signo
        self.set_dir(1, forward=(left >= 0.0))
        self.set_dir(2, forward=(right >= 0.0))

        # Publica comandos con signo
        # El DAC node usará abs() para el voltaje si así lo tienes programado
        self.publish_cmds(left, right)

        self.get_logger().info(
            f"thr={thr:+.2f} trn={trn:+.2f} | "
            f"M1={left:+.2f} M2={right:+.2f} | "
            f"POWER={'ON' if self.power_enabled else 'OFF'}"
        )

    def destroy_node(self):
        self.stop()
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyDiffNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()