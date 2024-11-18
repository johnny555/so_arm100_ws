
from pathlib import Path
import math

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = Path(launch_ros.substitutions.FindPackageShare(package='st3215_ros2_control').find('st3215_ros2_control'))
    default_model_path = pkg_share / 'urdf/so_arm100.urdf'
    default_controller_config_path = pkg_share / 'config/controllers.yaml'


    
def generate_launch_description():
    # Define parameters for the AprilTag node
    apriltag_params = {
        'size': 0.016,
        'family': '25h9'
    }

    realsense_params = {
        'camera_name': 'realsense',
        'camera_namespace': '',
        'pointcloud.enable': 'true',
        'enable_sync': 'true',
        'enable_rgbd': 'true',
        'align_depth.enable':'true',
        'depth_module.depth_profile':'640x480x15',
        'rgb_camera.color_profile':'640x480x15',
        'pointcloud.ordered_pc': 'true'

    }

    return launch.LaunchDescription([
        # RealSense Camera Node
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('realsense2_camera')+'/launch/rs_launch.py'),
                                 launch_arguments=realsense_params.items()
                                 ),


        # AprilTag Node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('/image_rect', '/realsense/color/image_raw')
            ],
            parameters=[apriltag_params]
        ),

        # TF Publisher for AprilTags
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tag_transform',
            arguments=['0.01', '-0.02', '-0.0', str(math.pi),str(-math.pi/2.0),str(math.pi/2.0),  'tag25h9:0',  'base_footprint'],
        )
    ])