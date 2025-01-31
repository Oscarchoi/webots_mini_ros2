import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    webots_home = "/snap/webots/current/usr/share/webots"

    package_share_dir = get_package_share_directory("webots_mini_ros2")
    world_file = os.path.join(package_share_dir, "worlds", "mini.wbt")

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="WEBOTS_HOME", value=webots_home),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=f"{os.environ.get('LD_LIBRARY_PATH', '')}:{webots_home}/lib/controller",
            ),
            SetEnvironmentVariable(
                name="WEBOTS_CONTROLLER_URL", value="tcp://localhost:1234/mini"
            ),
            ExecuteProcess(cmd=["webots", "--batch", world_file], output="screen"),
            Node(
                package="webots_mini_ros2",
                executable="webots_node",
                name="webots_ros2_node",
                output="screen",
            ),
        ]
    )
