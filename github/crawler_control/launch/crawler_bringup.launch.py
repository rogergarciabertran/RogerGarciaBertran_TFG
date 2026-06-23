from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node = Node(
        package='crawler_control',
        executable='motor_node',
        name='motor_node',
        output='log'
    )

    dac_node = Node(
        package='crawler_control',
        executable='dac_node',
        name='dac_node',
        output='log'
    )

    imu_node = Node(
        package="crawler_control",
        executable="imu_node",
        name="imu_node",
        output="log",
    )

    return LaunchDescription([
        motor_node,
        dac_node,
        imu_node
    ])
