import os
import launch

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    package_dir = get_package_share_directory('webots_mini_ros2')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'mini.wbt']),
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resources', 'mini.urdf')
    ros2_control_params = os.path.join(package_dir, 'resources', 'ros2_control.yml')


    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': robot_desc,
    #     }],
    # )

    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    mini_driver = WebotsController(
        robot_name='mini',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )


    waiting_nodes = WaitForControllerConnection(
        target_driver=mini_driver,
        nodes_to_start=ros_control_spawners
    )

    controller_manager_node = Node(
      package="controller_manager",
      executable="ros2_control_node",
      output="screen",
      parameters=[
          ros2_control_params
      ],
  )

    return LaunchDescription([
        webots,
        # webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        mini_driver,
        waiting_nodes,
        controller_manager_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
