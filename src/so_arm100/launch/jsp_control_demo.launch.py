"""
Demo of using Joint State Publisher to drive joints. 

"""

from pathlib import Path

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command



def generate_launch_description():
    pkg_share = Path(launch_ros.substitutions.FindPackageShare(package='so_arm100').find('so_arm100'))
    default_model_path = pkg_share / 'urdf/so_arm100.urdf'
    default_rviz_config_path = pkg_share / 'rviz/so_arm100.rviz'

    default_controller_config_path = pkg_share / 'config/controllers.yaml'

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                # ParameterValue is required to avoid being interpreted as YAML.
                'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str),
            },
            ],
                remappings=[('/joint_states','/current_joint_states')]

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
        remappings=[('/current_joint_states','/joint_states')]
    )

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
        use_microros_arg,
        microros_node,
        use_serial_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        st3215_control
    ])