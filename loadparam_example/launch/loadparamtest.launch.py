import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    test_node = Node(
        package="loadparam_example",
        executable="loadparam_example",
        name="loadparam_example",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(get_package_share_directory("loadparam_example"),
                         "config", "config.yaml")
        ],
    )
    ld.add_action(test_node)
    return ld
