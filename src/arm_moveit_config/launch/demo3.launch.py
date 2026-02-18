from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the MoveIt Configuration
    moveit_config = MoveItConfigsBuilder(
        "panda", package_name="arm_moveit_config"
    ).to_moveit_configs()

    # 2. Move Group Node
    # This acts as the "brain" that calculates paths.
    # It communicates with the controllers already running in Gazebo.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},  # CRITICAL: Sync with Gazebo time
        ],
    )

    # 3. RViz Node
    # Visualization tool.
    rviz_config_file = moveit_config.package_path / "config/moveit.rviz"
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(rviz_config_file)],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}, # CRITICAL: Sync with Gazebo time
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])