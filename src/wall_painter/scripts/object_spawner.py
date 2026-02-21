import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        
        # Publisher to spawn objects in RViz
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        # Publisher to send the exact grasp coordinate to C++
        self.target_pub = self.create_publisher(Pose, '/grasp_target', 10)
        
        # Give ROS a second to connect publishers
        time.sleep(1.0)
        
        self.spawn_environment()

    def spawn_environment(self):
        # 1. SPAWN THE TABLE
        table = CollisionObject()
        table.header.frame_id = "world"
        table.id = "table"
        
        table_box = SolidPrimitive()
        table_box.type = SolidPrimitive.BOX
        table_box.dimensions = [0.4, 0.5, 0.1] # Length, Width, Height
        
        table_pose = Pose()
        table_pose.position.x = 0.50  # 50cm in front of robot
        table_pose.position.y = 0.0
        table_pose.position.z = 0.10  # Table height
        table_pose.orientation.w = 1.0
        
        table.primitives.append(table_box)
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD
        self.collision_pub.publish(table)

        # 2. SPAWN THE GRASPABLE OBJECT (A small cylinder)
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = "object"
        
        obj_shape = SolidPrimitive()
        obj_shape.type = SolidPrimitive.CYLINDER
        obj_shape.dimensions = [0.12, 0.02] # Height: 12cm, Radius: 2cm
        
        obj_pose = Pose()
        obj_pose.position.x = 0.50  # Dead center on the table
        obj_pose.position.y = 0.0
        obj_pose.position.z = 0.21  # Sitting exactly on top of the table
        obj_pose.orientation.w = 1.0
        
        obj.primitives.append(obj_shape)
        obj.primitive_poses.append(obj_pose)
        obj.operation = CollisionObject.ADD
        self.collision_pub.publish(obj)
        
        self.get_logger().info("Spawned Table and Object in RViz!")

        # 3. BROADCAST THE COORDINATES CONTINUOUSLY
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.broadcast_target)
        self.target_pose = obj_pose

    def broadcast_target(self):
        # Send the object's exact pose to the C++ node
        self.target_pub.publish(self.target_pose)

def main(args=None):
    rclpy.init(args=args)
    spawner = ObjectSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()