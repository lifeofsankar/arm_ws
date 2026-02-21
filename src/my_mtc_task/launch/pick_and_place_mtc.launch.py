from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the MoveIt Configuration
    # We still need this to pass the parameters (robot description) to your node
    moveit_config = MoveItConfigsBuilder(
        "panda", package_name="arm_moveit_config"
    ).to_moveit_configs()

    # 2. Your MTC Node ONLY
    # We do NOT launch move_group or rviz here because they are already running in Terminal 2.
    mtc_node = Node(
        package="my_mtc_task",

        executable="pick_and_place",

        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}, # Sync with Gazebo time
            {"start_state_max_bounds_error": 0.1},
        ],
    )

    return LaunchDescription([
        mtc_node
    ])