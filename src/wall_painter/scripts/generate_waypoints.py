import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose

from generate_waypoints import generate_painting_waypoints


def main():
    rclpy.init()

    moveit = MoveItPy(node_name="paint_test")
    planning_component = moveit.get_planning_component("panda_arm")

    # Match your wall in SDF
    wall_center = [1.4, 0.0, 0.75]
    wall_width = 2.0
    wall_height = 1.5

    waypoints = generate_painting_waypoints(
        wall_center,
        wall_width,
        wall_height
    )

    # 👉 TEST ONLY FIRST WAYPOINT
    target_pose = waypoints[0]

    planning_component.set_goal_state(pose=target_pose)

    plan = planning_component.plan()

    if plan:
        print("Plan found. Executing...")
        moveit.execute(plan)
    else:
        print("Planning failed.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()