import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
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
        # NEW: Create a topic to shout the wall's X position to C++
        self.target_pub = self.create_publisher(Float64, '/wall_target_x', 10)
        self.wall_found = False

        # Setup TF listener to find the camera's location
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return

        x_vals = [p[0] for p in points if not math.isinf(p[0]) and 0.2 < p[0] < 3.0]
        if not x_vals:
            return

        avg_depth = np.median(x_vals)

        try:
            trans = self.tf_buffer.lookup_transform('world', msg.header.frame_id, rclpy.time.Time())
            camera_x = trans.transform.translation.x
            wall_world_x = camera_x + avg_depth

            # NEW: ALWAYS shout the coordinate so C++ can hear it anytime!
            x_msg = Float64()
            x_msg.data = wall_world_x
            self.target_pub.publish(x_msg)

            # Only spawn the green box the first time so RViz doesn't flicker
            if not self.wall_found:
                self.publish_wall(wall_world_x)
                self.wall_found = True
                self.get_logger().info(f'Spawning wall in WORLD at X: {wall_world_x:.2f}m')

        except Exception as e:
            # Hide the TF warning so it doesn't spam your terminal
            pass
            
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
        
        # NEW: Publish the exact X coordinate for C++
        # x_msg = Float64()
        # x_msg.data = world_x
        # self.target_pub.publish(x_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = WallDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()