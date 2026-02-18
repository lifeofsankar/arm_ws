#!/usr/bin/env python3

import rclpy
# from rclpy.node import Node
from geometry_msgs.msg import Pose

from moveit_py import MoveItPy
# from moveit_py.planning import MultiPipelinePlanRequestParameters


def generate_painting_waypoints(
        wall_center,
        wall_width,
        wall_height,
        offset=0.05,
        stripe_spacing=0.15):

    waypoints = []

    start_y = wall_center[1] - wall_width / 2
    end_y   = wall_center[1] + wall_width / 2
    start_z = wall_center[2] + wall_height / 2

    num_stripes = int(wall_height / stripe_spacing) + 1

    for i in range(num_stripes):

        z_pos = start_z - i * stripe_spacing

        if z_pos < wall_center[2] - wall_height / 2:
            break

        if i % 2 == 0:
            y_start, y_end = start_y, end_y
        else:
            y_start, y_end = end_y, start_y

        pose_start = Pose()
        pose_start.position.x = wall_center[0] - offset
        pose_start.position.y = y_start
        pose_start.position.z = z_pos
        pose_start.orientation.w = 1.0

        pose_end = Pose()
        pose_end.position.x = wall_center[0] - offset
        pose_end.position.y = y_end
        pose_end.position.z = z_pos
        pose_end.orientation.w = 1.0

        waypoints.append(pose_start)
        waypoints.append(pose_end)

    return waypoints


def main():

    rclpy.init()

    moveit = MoveItPy(node_name="paint_wall_node")
    planning_component = moveit.get_planning_component("panda_arm")

    wall_center = [1.4, 0.0, 0.75]
    wall_width = 2.0
    wall_height = 1.5

    waypoints = generate_painting_waypoints(
        wall_center,
        wall_width,
        wall_height
    )

    print("Planning Cartesian path...")

    plan_request_params = MultiPipelinePlanRequestParameters(
        moveit.get_robot_model(),
        ["ompl"]
    )

    planning_component.set_start_state_to_current_state()

    planning_component.set_goal_state(pose_stamped_msg=None)

    trajectory = planning_component.plan_cartesian_path(
        waypoints=waypoints,
        max_step=0.01
    )

    if trajectory:
        print("Executing trajectory...")
        moveit.execute(trajectory.trajectory)
    else:
        print("Planning failed.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()