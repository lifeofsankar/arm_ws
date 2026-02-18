from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the MoveIt Configuration
    moveit_config = MoveItConfigsBuilder(
        "panda", package_name="arm_moveit_config"
    ).to_moveit_configs()

    # Define the list of capabilities MoveGroup should offer.
    # We MUST include 'ExecuteTaskSolutionCapability' for MTC to work.
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability "
                        "move_group/MoveGroupKinematicsService "
                        "move_group/MoveGroupMoveAction "
                        "move_group/MoveGroupPlanService "
                        "move_group/MoveGroupQueryPlannersService "
                        "move_group/MoveGroupStateValidationService "
                        "move_group/MoveGroupGetPlanningSceneService "
                        "move_group/ApplyPlanningSceneService "
                        "move_group/ClearOctomapService "
    }

    # 2. Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities, # <--- Added this!
            {"use_sim_time": True},
        ],
    )

    # 3. RViz Node
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
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])