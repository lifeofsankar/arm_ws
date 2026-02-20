import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Vector3  # NEW: Vector3 to send 3 numbers!
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
import tf2_ros

class WallDetector(Node):
    def __init__(self):
        super().__init__('wall_detector')
        
        self.subscription = self.create_subscription(
            PointCloud2, '/depth_camera/points', self.listener_callback, 10)
            
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        # NEW: Changed to Vector3 to send Depth, Width, and Height
        self.target_pub = self.create_publisher(Vector3, '/wall_dimensions', 10) 
        
        self.wall_found = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return

        # 1. Grab all points in front of the camera (0.2m to 3.0m)
        valid_points = [p for p in points if not math.isinf(p[0]) and 0.2 < p[0] < 3.0]
        if not valid_points:
            return

        # 2. Find the average depth of whatever is right in front of the camera (the wall)
        all_x = [p[0] for p in valid_points]
        avg_depth = np.median(all_x)

        # 3. --- THE FIX: STRICT FILTERING ---
        # Only keep points that are ON the wall (within 10cm of the avg_depth)
        # AND ignore points that belong to the floor (z > -0.3 relative to the camera)
        wall_points = [p for p in valid_points if abs(p[0] - avg_depth) < 0.10 and p[2] > -0.3]
        
        if not wall_points:
            return

        # 4. Now calculate width and height using ONLY the clean wall points!
        y_vals = [p[1] for p in wall_points]
        z_vals = [p[2] for p in wall_points]

        wall_width = (max(y_vals) - min(y_vals)) * 0.9 
        wall_height = (max(z_vals) - min(z_vals)) * 0.9 

        try:
            trans = self.tf_buffer.lookup_transform('world', msg.header.frame_id, rclpy.time.Time())
            camera_x = trans.transform.translation.x
            wall_world_x = camera_x + avg_depth

            dim_msg = Vector3()
            dim_msg.x = wall_world_x
            dim_msg.y = wall_width
            dim_msg.z = wall_height
            self.target_pub.publish(dim_msg)

            if not self.wall_found:
                self.publish_wall(wall_world_x, wall_width, wall_height)
                self.wall_found = True
                self.get_logger().info(f'Wall Found! Depth: {wall_world_x:.2f}m, Width: {wall_width:.2f}m, Height: {wall_height:.2f}m')

        except Exception as e:
            pass

    # NEW: Accept dynamic width and height
    def publish_wall(self, world_x, width, height):
        wall = CollisionObject()
        wall.header.frame_id = "world" 
        wall.id = "wall"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # Use the real width and height we just calculated!
        box.dimensions = [0.1, width, height] 

        pose = Pose()
        pose.position.x = world_x + 0.05
        pose.position.y = 0.0
        pose.position.z = 0.5 
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