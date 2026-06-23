#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from crawler_control.core.config_loader import load_json_config


def deg2rad(x):
    return x * math.pi / 180.0


class BNO055Node(Node):
    def __init__(self):
        super().__init__("imu_node")

        cfg = load_json_config("imu_config.json")

        self.rate_hz = float(cfg.get("rate_hz", 50.0))
        self.frame_id = str(cfg.get("frame_id", "imu_link"))
        self.i2c_bus = int(cfg.get("i2c_bus", 1))
        self.i2c_addr = int(cfg.get("i2c_address", 0x28))
        self.simulate = bool(cfg.get("simulate", False))

        self.pub_imu = self.create_publisher(Imu, "/crawler/imu/data", 10)
        self.pub_mag = self.create_publisher(MagneticField, "/crawler/imu/mag", 10)

        self.sensor = None
        if not self.simulate:
            try:
                import board
                import busio
                import adafruit_bno055

                i2c = busio.I2C(board.SCL, board.SDA)
                self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_addr)
            except Exception as e:
                self.get_logger().error(f"No puedo inicializar BNO055: {e}")
                self.get_logger().error("Pon simulate=true o revisa I2C/alimentación.")
                self.simulate = True

        self.timer = self.create_timer(1.0 / self.rate_hz, self.update)

        self.get_logger().info(
            f"BNO055Node listo | rate={self.rate_hz}Hz frame_id={self.frame_id} "
            f"i2c_bus={self.i2c_bus} addr=0x{self.i2c_addr:02X} simulate={self.simulate}"
        )

    def update(self):
        now = self.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.frame_id

        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self.frame_id

        if self.simulate:
            imu.orientation.w = 1.0
            imu.angular_velocity.z = 0.0
            imu.linear_acceleration.z = 9.81
            self.pub_imu.publish(imu)
            self.pub_mag.publish(mag)
            return

        q = self.sensor.quaternion
        if q is not None:
            w, x, y, z = q
            imu.orientation.x = float(x)
            imu.orientation.y = float(y)
            imu.orientation.z = float(z)
            imu.orientation.w = float(w)

        g = self.sensor.gyro
        if g is not None:
            imu.angular_velocity.x = float(deg2rad(g[0]))
            imu.angular_velocity.y = float(deg2rad(g[1]))
            imu.angular_velocity.z = float(deg2rad(g[2]))

        a = self.sensor.acceleration
        if a is not None:
            imu.linear_acceleration.x = float(a[0])
            imu.linear_acceleration.y = float(a[1])
            imu.linear_acceleration.z = float(a[2])

        m = self.sensor.magnetic
        if m is not None:
            mag.magnetic_field.x = float(m[0] * 1e-6)
            mag.magnetic_field.y = float(m[1] * 1e-6)
            mag.magnetic_field.z = float(m[2] * 1e-6)

        self.pub_imu.publish(imu)
        self.pub_mag.publish(mag)


def main():
    rclpy.init()
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()