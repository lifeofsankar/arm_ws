import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
import tf2_ros # We use this to find the camera in the world

class WallDetector(Node):
    def __init__(self):
        super().__init__('wall_detector')
        
        self.subscription = self.create_subscription(
            PointCloud2, '/depth_camera/points', self.listener_callback, 10)
            
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.wall_found = False

        # Setup TF listener to find the camera's location
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg):
        if self.wall_found:
            return

        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return

        # X is the forward depth from the camera lens
        x_vals = [p[0] for p in points if not math.isinf(p[0]) and 0.2 < p[0] < 3.0]
        if not x_vals:
            return

        avg_depth = np.median(x_vals)

        try:
            # Ask TF: "Where is the camera in the world?"
            trans = self.tf_buffer.lookup_transform('world', msg.header.frame_id, rclpy.time.Time())
            camera_x = trans.transform.translation.x

            # Math: Camera Position + Depth = Wall Position
            wall_world_x = camera_x + avg_depth

            self.publish_wall(wall_world_x)
            self.wall_found = True
            
            self.get_logger().info(f'Camera is at World X: {camera_x:.2f}m')
            self.get_logger().info(f'Detected Depth: {avg_depth:.2f}m')
            self.get_logger().info(f'Spawning wall in WORLD at X: {wall_world_x:.2f}m')

        except Exception as e:
            self.get_logger().warn(f'Waiting for TF transform... {e}')

    def publish_wall(self, world_x):
        wall = CollisionObject()
        wall.header.frame_id = "world" # WE ARE BACK IN THE WORLD FRAME!
        wall.id = "wall"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 2.0, 2.0] # Guaranteed to be vertical in 'world'

        pose = Pose()
        pose.position.x = world_x - 0.02
        pose.position.y = 0.0
        pose.position.z = 0.5 # Centered vertically
        pose.orientation.w = 1.0

        wall.primitives.append(box)
        wall.primitive_poses.append(pose)
        wall.operation = CollisionObject.ADD

        self.publisher.publish(wall)

def main(args=None):
    rclpy.init(args=args)
    detector = WallDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()