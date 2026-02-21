from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "panda", package_name="arm_moveit_config"
    ).to_moveit_configs()

    # Extract robot description content
    robot_description_content = moveit_config.robot_description["robot_description"]
    robot_description = {"robot_description": robot_description_content}
    
    # Robot State Publisher - CRITICAL: Must publish robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "publish_frequency": 30.0,
            }
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # ROS2 Control Node
    ros2_controllers_path = str(moveit_config.package_path / "config/ros2_controllers.yaml")
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
        ],
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Arm Controller Spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Hand Controller Spawner  
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # RViz
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
        ],
    )

    return LaunchDescription([
        # Start these immediately
        robot_state_publisher,
        static_tf,
        
        # Wait for robot_state_publisher, then start ros2_control after 3 sec
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=[ros2_control_node],
                    )
                ],
            )
        ),
        
        # Spawn joint_state_broadcaster after 5 sec
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher,
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[joint_state_broadcaster_spawner],
                    )
                ],
            )
        ),
        
        # Spawn controllers after 7 sec
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher,
                on_start=[
                    TimerAction(
                        period=7.0,
                        actions=[
                            arm_controller_spawner,
                            hand_controller_spawner,
                        ],
                    )
                ],
            )
        ),
        
        # Start move_group and rviz after 9 sec
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher,
                on_start=[
                    TimerAction(
                        period=9.0,
                        actions=[
                            move_group_node,
                            rviz_node,
                        ],
                    )
                ],
            )
        ),
    ])