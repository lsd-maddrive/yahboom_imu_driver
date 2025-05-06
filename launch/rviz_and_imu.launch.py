from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu_driver_node',
        output='screen',
        parameters=[
            {'port': '/dev/imu_usb'},   # поменяй на актуальный порт при необходимости
            {'baud': 115200}              # или 115200 — зависит от настроек модуля
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        imu_node,
        # rviz_node  # Раскомментируй, если хочешь запускать RViz вместе
    ])