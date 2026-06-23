#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Imu, MagneticField

class TelemetryNode(Node):
    def __init__(self):
        super().__init__("telemetry_node")

        # Motores
        self.cmd1 = self.cmd2 = None
        self.v1 = self.v2 = None
        self.f1 = self.f2 = None
        self.rpm1 = self.rpm2 = None

        # IMU
        self.mag = None
        self.imu_q = None      # (x,y,z,w)
        self.imu_gyr = None    # (x,y,z) rad/s
        self.imu_acc = None    # (x,y,z) m/s^2
        self.rpy_deg = None    # (roll,pitch,yaw) grados

        # Subs motores
        self.create_subscription(Float32, "/crawler/motor1/cmd", self.cb_cmd1, 10)
        self.create_subscription(Float32, "/crawler/motor2/cmd", self.cb_cmd2, 10)
        self.create_subscription(Float32, "/crawler/motor1/dac_v", self.cb_v1, 10)
        self.create_subscription(Float32, "/crawler/motor2/dac_v", self.cb_v2, 10)
        self.create_subscription(Float32, "/crawler/motor1/fg_hz", self.cb_f1, 10)
        self.create_subscription(Float32, "/crawler/motor2/fg_hz", self.cb_f2, 10)
        self.create_subscription(Float32, "/crawler/motor1/rpm", self.cb_rpm1, 10)
        self.create_subscription(Float32, "/crawler/motor2/rpm", self.cb_rpm2, 10)
        self.create_subscription(MagneticField, "/crawler/imu/mag", self.cb_mag, 10)

        # Sub IMU  (CAMBIA el topic si el tuyo es otro)
        self.create_subscription(Imu, "/crawler/imu", self.cb_imu, 10)
        self.create_subscription(Imu, "/crawler/imu/data", self.cb_imu, 10)

        self.period = 0.5  # prueba 0.2 o 0.1 si quieres
        self.create_timer(self.period, self.tick)

        self.get_logger().info("TelemetryNode compact + IMU iniciado")

    # ---- callbacks motores ----
    def cb_cmd1(self, m): self.cmd1 = m.data
    def cb_cmd2(self, m): self.cmd2 = m.data
    def cb_v1(self, m): self.v1 = m.data
    def cb_v2(self, m): self.v2 = m.data
    def cb_f1(self, m): self.f1 = m.data
    def cb_f2(self, m): self.f2 = m.data
    def cb_rpm1(self, m): self.rpm1 = m.data
    def cb_rpm2(self, m): self.rpm2 = m.data

    # ---- IMU ----
    def cb_imu(self, msg: Imu):
        q = msg.orientation
        self.imu_q = (q.x, q.y, q.z, q.w)

        g = msg.angular_velocity
        self.imu_gyr = (g.x, g.y, g.z)

        a = msg.linear_acceleration
        self.imu_acc = (a.x, a.y, a.z)

        # convertir quaternion -> roll/pitch/yaw (rad) y pasar a grados
        roll, pitch, yaw = self.quat_to_rpy(q.x, q.y, q.z, q.w)
        self.rpy_deg = (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def cb_mag(self, msg: MagneticField):
        self.mag = (
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        )

    @staticmethod
    def quat_to_rpy(x, y, z, w):
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # clamp
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _fmt(self, x, f):
        return "--" if x is None else f.format(x)

    def _fmt3(self, v, f):
        if v is None:
            return "--/--/--"
        return f.format(v[0]) + "/" + f.format(v[1]) + "/" + f.format(v[2])

    def tick(self):
        imu_rpy = "--/--/--" if self.rpy_deg is None else f"{self.rpy_deg[0]:+.1f}/{self.rpy_deg[1]:+.1f}/{self.rpy_deg[2]:+.1f}"
        imu_gyr = self._fmt3(self.imu_gyr, "{:+.2f}")
        imu_acc = self._fmt3(self.imu_acc, "{:+.2f}")
        mag_txt = self._fmt3(self.mag, "{:+.6f}")

        line = (
            f"cmd L/R: {self._fmt(self.cmd1,'{:+.2f}')}/{self._fmt(self.cmd2,'{:+.2f}')} | "
            f"V L/R: {self._fmt(self.v1,'{:.2f}')}/{self._fmt(self.v2,'{:.2f}')} | "
            f"FG Hz L/R: {self._fmt(self.f2,'{:.0f}')}/{self._fmt(self.f1,'{:.0f}')} | "
            f"RPM L/R: {self._fmt(self.rpm2,'{:.0f}')}/{self._fmt(self.rpm1,'{:.0f}')} | "
            f"IMU RPY(deg): {imu_rpy} | "
            f"GYR(rad/s): {imu_gyr} | "
            f"ACC(m/s2): {imu_acc} | "
            f"{time.strftime('%H:%M:%S')}"
            f"MAG(T): {mag_txt} | "
        )
        print(line, flush=True)


def main():
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    