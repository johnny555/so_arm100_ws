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
    default_rviz_config_path = pkg_share / 'rviz/roam100.rviz'

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
        parameters=[LaunchConfiguration('controlconfig')],
        remappings=[('/current_joint_states','/joint_states')]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[('/joint_states','/desired_joint_states')]
        )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen' ,
        arguments=['-d', LaunchConfiguration('rvizconfig')],   )

    return launch.LaunchDescription([

        DeclareLaunchArgument(name='model',
                                             default_value=str(default_model_path),
                                             description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig',
                                             default_value=str(default_rviz_config_path),
                                             description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='controlconfig',
                                             default_value=str(default_controller_config_path),
                                             description='Absolute path to controller config file'),

        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        st3215_control
    ])