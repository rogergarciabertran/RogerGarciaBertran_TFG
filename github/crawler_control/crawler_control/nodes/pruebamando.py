#!/usr/bin/env python3
import time
import smbus2
import lgpio

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


def clamp(x, a, b):
    return max(a, min(b, x))


class JoyOneDacTest(Node):
    def __init__(self):
        super().__init__("joy_one_dac_test")

        # ===== Ajusta según tu mando mirando /joy =====
        self.axis_throttle = 1   # stick izq vertical suele ser 1
        self.axis_turn = 0       # stick izq horizontal suele ser 0
        self.deadman_button = 4  # LB suele ser 4 (ajusta)
        self.deadzone = 0.08
        # =============================================

        # ===== Hardware =====
        self.vmax = 5.0
        self.vref = 5.0

        self.i2c_bus = 1
        self.dac_addr = 0x60

        self.chip = 4
        self.dir1_pin = 23
        self.dir2_pin = 24
        # ====================

        # I2C DAC
        self.bus = smbus2.SMBus(self.i2c_bus)

        # GPIO DIR
        self.h = lgpio.gpiochip_open(self.chip)
        lgpio.gpio_claim_output(self.h, self.dir1_pin, 1)  # HIGH = forward
        lgpio.gpio_claim_output(self.h, self.dir2_pin, 1)

        # === Publicadores para TelemetryNode ===
        self.pub_motor_cmd = self.create_publisher(Float32, "/motor_cmd", 10)
        self.pub_dac_voltage = self.create_publisher(Float32, "/crawler/motor/analogic_signal", 10)

        # seguridad: si se pierde /joy, parar
        self.last_joy = time.time()
        self.timeout_s = 0.6
        self.create_timer(0.05, self.watchdog)

        self.create_subscription(Joy, "/joy", self.on_joy, 10)

        self.set_dac(0.0)
        self.publish_telemetry(cmd_norm=0.0, v_applied=0.0)

        self.get_logger().info("Nodo listo: /joy -> DIR1/DIR2 + DAC(0-5V) + publica telemetría.")

    def publish_telemetry(self, cmd_norm: float, v_applied: float):
        m = Float32()
        m.data = float(cmd_norm)
        self.pub_motor_cmd.publish(m)

        vmsg = Float32()
        vmsg.data = float(v_applied)
        self.pub_dac_voltage.publish(vmsg)

    def set_dac(self, volts: float):
        volts = clamp(volts, 0.0, self.vmax)
        code = int((volts / self.vref) * 4095)
        high = (code >> 8) & 0x0F
        low = code & 0xFF
        self.bus.write_i2c_block_data(self.dac_addr, high, [low])
        return volts

    def set_dir(self, motor: int, forward: bool):
        val = 1 if forward else 0  # HIGH=forward, LOW=reverse
        pin = self.dir1_pin if motor == 1 else self.dir2_pin
        lgpio.gpio_write(self.h, pin, val)

    def stop_all(self):
        try:
            self.set_dac(0.0)
        except Exception:
            pass
        self.set_dir(1, True)
        self.set_dir(2, True)

        # Publica telemetría de parada
        self.publish_telemetry(cmd_norm=0.0, v_applied=0.0)

    def watchdog(self):
        if time.time() - self.last_joy > self.timeout_s:
            self.stop_all()

    def on_joy(self, msg: Joy):
        self.last_joy = time.time()

        # Deadman: si no lo pulsas => paro
        if 0 <= self.deadman_button < len(msg.buttons) and msg.buttons[self.deadman_button] == 0:
            self.stop_all()
            return

        thr = msg.axes[self.axis_throttle] if self.axis_throttle < len(msg.axes) else 0.0
        trn = msg.axes[self.axis_turn] if self.axis_turn < len(msg.axes) else 0.0

        # deadzone
        if abs(thr) < self.deadzone:
            thr = 0.0
        if abs(trn) < self.deadzone:
            trn = 0.0

        # mezcla diferencial
        left = clamp(thr + trn, -1.0, 1.0)
        right = clamp(thr - trn, -1.0, 1.0)

        # dirección individual
        self.set_dir(1, forward=(left >= 0.0))
        self.set_dir(2, forward=(right >= 0.0))

        # 1 DAC => velocidad común (máximo de ambos)
        v = max(abs(left), abs(right)) * self.vmax

        # Publica el comando normalizado que verá TelemetryNode
        cmd_norm = float(thr)  # [-1..1]

        try:
            v_applied = self.set_dac(v)
        except Exception as e:
            self.get_logger().error(f"Error DAC: {e}")
            self.stop_all()
            return

        # Publicar telemetría (cmd y voltaje aplicado)
        self.publish_telemetry(cmd_norm=cmd_norm, v_applied=v_applied)

        self.get_logger().info(
            f"thr={thr:+.2f} trn={trn:+.2f} | L={left:+.2f} R={right:+.2f} | V={v_applied:.2f}V"
        )

    def destroy_node(self):
        self.stop_all()
        self.bus.close()
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main():
    rclpy.init()
    node = JoyOneDacTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
