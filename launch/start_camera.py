from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_camera/point_cloud']
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_camera/color_point_cloud']
        ),
        Node(
            package="mecheye_ros_interface",
            executable="start",
            name="mechmind_camera_publisher_service",
            output="screen",
            prefix="xterm -e",
            parameters=[
                {"save_file": False},
                {"camera_ip": "172.20.112.1"},
                {"user_external_intri": False},
                {"fx": 1727.4641025602748},
                {"fy": 1727.4586926701952},
                {"u": 655.8180825729554},
                {"v": 516.6306500606158}
            ]
        )
    ])
