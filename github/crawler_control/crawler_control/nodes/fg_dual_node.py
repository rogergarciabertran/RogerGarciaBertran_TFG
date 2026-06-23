#!/usr/bin/env python3
import time
import lgpio

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from crawler_control.core.config_loader import load_json_config


def clamp(x, a, b):
    return max(a, min(b, x))


class FGDualNode(Node):
    def __init__(self):
        super().__init__("fg_dual_node")

        cfg = load_json_config("fg_dual_config.json")

        self.simulate = bool(cfg.get("simulate", False))
        self.window = float(cfg.get("window_s", 0.1))

        self.fg1_pin = int(cfg["fg1_pin"])
        self.ppr1 = float(cfg.get("pulses_per_rev_1", 6))

        self.fg2_pin = int(cfg["fg2_pin"])
        self.ppr2 = float(cfg.get("pulses_per_rev_2", 6))

        self.chip = 4
        self.h = lgpio.gpiochip_open(self.chip)

        # Entradas FG (si simulate=True no hace falta, pero no molesta)
        lgpio.gpio_claim_input(self.h, self.fg1_pin)
        lgpio.gpio_claim_input(self.h, self.fg2_pin)

        # Publishers
        self.pub_f1 = self.create_publisher(Float32, "/crawler/motor1/fg_hz", 10)
        self.pub_r1 = self.create_publisher(Float32, "/crawler/motor1/rpm", 10)
        self.pub_f2 = self.create_publisher(Float32, "/crawler/motor2/fg_hz", 10)
        self.pub_r2 = self.create_publisher(Float32, "/crawler/motor2/rpm", 10)

        # Timer: publica cada ventana (aprox)
        self.create_timer(self.window, self.update)

        self.get_logger().info(
            f"FGDualNode | fg1_pin={self.fg1_pin} ppr1={self.ppr1} | fg2_pin={self.fg2_pin} ppr2={self.ppr2} | window={self.window}s | simulate={self.simulate}"
        )

        # para sim
        self._t0 = time.time()

    def measure_freq(self, pin: int) -> float:
        t_start = time.time()
        last = lgpio.gpio_read(self.h, pin)
        edges = 0

        while time.time() - t_start < self.window:
            v = lgpio.gpio_read(self.h, pin)
            if last == 0 and v == 1:
                edges += 1
            last = v
            time.sleep(0.00001)

        return edges / self.window

    def sim_freq(self, base_hz: float, amp_hz: float, period_s: float) -> float:
        t = time.time() - self._t0
        # simulación simple (sube/baja)
        import math
        return max(0.0, base_hz + amp_hz * math.sin(2 * math.pi * t / period_s))

    def update(self):
        if self.simulate:
            f1 = self.sim_freq(50.0, 20.0, 5.0)
            f2 = self.sim_freq(45.0, 15.0, 6.0)
        else:
            f1 = self.measure_freq(self.fg1_pin)
            f2 = self.measure_freq(self.fg2_pin)

        rpm1 = (f1 * 60.0) / self.ppr1 if self.ppr1 > 0 else 0.0
        rpm2 = (f2 * 60.0) / self.ppr2 if self.ppr2 > 0 else 0.0

        m = Float32()

        m.data = float(f1)
        self.pub_f1.publish(m)
        m.data = float(rpm1)
        self.pub_r1.publish(m)

        m.data = float(f2)
        self.pub_f2.publish(m)
        m.data = float(rpm2)
        self.pub_r2.publish(m)

    def destroy_node(self):
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main():
    rclpy.init()
    node = FGDualNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
