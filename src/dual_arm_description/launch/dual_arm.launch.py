import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Dynamically find the package directory (Crucial for mesh resolution)
    pkg_arm_description = get_package_share_directory('dual_arm_description')
    
    # 2. Set dynamic paths
    default_urdf = os.path.join(pkg_arm_description, 'urdf', 'ur.urdf.xacro')
    # It's highly recommended to save an RViz config file so you don't have to setup displays manually
    default_rviz_config = os.path.join(pkg_arm_description, 'rviz', 'rviz2.rviz')

    # Launch arguments
    urdf_path_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value=default_urdf,
        description='Path to the URDF/XACRO file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz configuration file'
    )

    # Launch configurations
    urdf_path = LaunchConfiguration('urdf_path')
    rviz_config = LaunchConfiguration('rviz_config')

    # Xacro command
    robot_description = {
        'robot_description': Command(['xacro ', urdf_path])
    }
    
    # Nodes
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config], # Loads your saved RViz settings
        output='screen'
    )

    return LaunchDescription([
        urdf_path_arg,
        rviz_config_arg,
        robot_state_pub,
        joint_state_gui,
        rviz
    ])