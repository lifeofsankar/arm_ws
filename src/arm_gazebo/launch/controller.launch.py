from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Path to your empty_world launch file
    gazeboLaunchFile = "/home/admin/First_ROS2/arm_ws/src/arm_gazebo/launch/empty_world.launch.py"
    
    # 1. Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 2. Arm Controller (RENAMED)
    # Changed from 'panda_arm_controller' to 'arm_controller' to match your YAML/Gazebo config
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # 3. Hand/Gripper Controller (RENAMED)
    # Changed from 'panda_hand_controller' to 'gripper_controller' to match your YAML/Gazebo config
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        # Launch Gazebo and the Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazeboLaunchFile)
        ),
        
        # Spawn the controllers
        arm_controller_spawner,
        gripper_controller_spawner,
        joint_state_broadcaster_spawner
    ])