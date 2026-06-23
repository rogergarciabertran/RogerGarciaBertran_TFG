from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_joy = LaunchConfiguration("use_joy")
    use_udp_joy = LaunchConfiguration("use_udp_joy")
    use_joy_diff = LaunchConfiguration("use_joy_diff")
    use_dac = LaunchConfiguration("use_dac")
    use_fg = LaunchConfiguration("use_fg")
    use_telemetry = LaunchConfiguration("use_telemetry")
    use_imu = LaunchConfiguration("use_imu")

    return LaunchDescription([
        DeclareLaunchArgument("use_joy", default_value="true"),
        DeclareLaunchArgument("use_udp_joy", default_value="false"),
        DeclareLaunchArgument("use_joy_diff", default_value="true"),
        DeclareLaunchArgument("use_dac", default_value="true"),
        DeclareLaunchArgument("use_fg", default_value="true"),
        DeclareLaunchArgument("use_telemetry", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="true"),

        # 0) IMU (usa venv vía script)
        ExecuteProcess(
            cmd=["bash", "-lc", "/home/crawler/crawler_ws/scripts/run_imu.sh"],
            output="screen",
            condition=IfCondition(use_imu),
        ),

        # 1A) Driver mando conectado directamente a Raspberry -> /joy
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="log",
            condition=IfCondition(use_joy),
        ),

        # 1B) Receptor mando desde Windows por UDP -> /joy
        Node(
            package="crawler_control",
            executable="udp_joy_node",
            name="udp_joy_node",
            output="screen",
            condition=IfCondition(use_udp_joy),
        ),

        # 2) Mando -> DIR + /crawler/motor1/cmd /crawler/motor2/cmd
        Node(
            package="crawler_control",
            executable="joy_diff_node",
            name="joy_diff_node",
            output="log",
            condition=IfCondition(use_joy_diff),
        ),

        # 3) Dual DAC -> voltajes a 0x60/0x61
        Node(
            package="crawler_control",
            executable="dac_dual_node",
            name="dac_dual_node",
            output="log",
            condition=IfCondition(use_dac),
        ),

        # 4) Dual FG -> Hz y RPM
        Node(
            package="crawler_control",
            executable="fg_dual_node",
            name="fg_dual_node",
            output="log",
            condition=IfCondition(use_fg),
        ),

        # 5) Telemetry
        Node(
            package="crawler_control",
            executable="telemetry_node",
            name="telemetry_node",
            output="screen",
            condition=IfCondition(use_telemetry),
        ),
        # 1C) Relés Panasonic alimentación controladores
        Node(
            package="crawler_control",
            executable="power_relay_node",
            name="power_relay_node",
            output="screen",
),
    ])