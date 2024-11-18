"""
Demo of using Joint State Publisher to drive joints. 

"""

from pathlib import Path

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = Path(launch_ros.substitutions.FindPackageShare(package='st3215_ros2_control').find('st3215_ros2_control'))
    default_model_path = pkg_share / 'urdf/so_arm100.urdf'
    default_controller_config_path = pkg_share / 'config/controllers.yaml'


    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('st3215_ros2_control'),
                'launch',
                'description.launch.py'
                ])
        ])
    )

    st3215_control = Node(
        package='st3215_ros2_control',
        executable='st3215_control',
        name='st3215_control',
        parameters=[LaunchConfiguration('controlconfig')]
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
                FindPackageShare('ro_arm100_moveit'),
                'launch',
                'move_group.launch.py'
                ])
        ])
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ro_arm100_moveit'),
                'launch',
                'moveit_rviz.launch.py'
                ])
        ])
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('st3215_ros2_control'),
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

        control_node,
        my_controller_spawner,
        robot_state_publisher_node,
        st3215_control,
        move_group,
        perception,
        moveit_rviz
    ])