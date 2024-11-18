from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("st3215_ros2_control", package_name="ro_arm100_moveit").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
