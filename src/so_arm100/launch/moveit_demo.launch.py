"""
Demo using MoveIt & Perception
"""

from pathlib import Path

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = Path(launch_ros.substitutions.FindPackageShare(package='so_arm100').find('so_arm100'))
    default_model_path = pkg_share / 'urdf/so_arm100.urdf'
    default_controller_config_path = pkg_share / 'config/controllers.yaml'


    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('so_arm100'),
                'launch',
                'description.launch.py'
                ])
        ])
    )

    use_serial_arg = DeclareLaunchArgument(
        'use_serial',
        default_value='false',
        description='Whether to launch serial control node'
    )


    st3215_control = Node(
        condition=IfCondition(LaunchConfiguration('use_serial')),
        package='st3215_ros2_control',
        executable='st3215_control',
        name='st3215_control',
        parameters=[LaunchConfiguration('controlconfig')],
        remappings=[('/current_joint_states','/joint_states')]
    )


    use_microros_arg = DeclareLaunchArgument(
        'use_microros',
        default_value='true',
        description='Whether to launch micro-ROS node'
    )

    microros_node = Node(
        condition=IfCondition(LaunchConfiguration('use_microros')),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        remappings=[('/joint_states', '/current_joint_states')]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[LaunchConfiguration('controlconfig') ],
        output="screen",
        remappings=[('/controller_manager/robot_description','/robot_description')]
    )

    my_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller","jaw_controller", "joint_state_broadcaster" ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('so_arm100_moveit'),
                'launch',
                'move_group.launch.py'
                ])
        ])
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('so_arm100_moveit'),
                'launch',
                'moveit_rviz.launch.py'
                ])
        ])
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('so_arm100'),
                'launch',
                'perception.launch.py'
                ])
        ])
    )

    

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='model',
                                             default_value=str(default_model_path),
                                             description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='controlconfig',
                                             default_value=str(default_controller_config_path),
                                             description='Absolute path to controller config file'),
        use_microros_arg,
        microros_node,
        use_serial_arg,
        control_node,
        my_controller_spawner,
        robot_state_publisher_node,
        st3215_control,
        move_group,
        perception,
        moveit_rviz
    ])