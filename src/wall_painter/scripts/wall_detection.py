#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class WallTracker(Node):

    def __init__(self):
        super().__init__('wall_tracker')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.cloud_callback,
            10
        )

        self.get_logger().info("Wall tracker started...")


    def cloud_callback(self, msg):

        xs = []
        ys = []
        zs = []

        for i, point in enumerate(
            pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if i > 5000:
                break

            xs.append(point[0])
            ys.append(point[1])
            zs.append(point[2])

        if len(xs) == 0:
            self.get_logger().warn("Empty cloud")
            return

        x_vals = np.array(xs)

        # Remove far noise
        x_vals = x_vals[np.abs(x_vals) < 2.0]

        if len(x_vals) == 0:
            self.get_logger().warn("No valid wall points")
            return

        wall_distance = np.median(x_vals)

        self.get_logger().info(
            f"Estimated Wall Distance (X): {wall_distance:.3f} meters"
        )


def main():
    rclpy.init()
    node = WallTracker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()