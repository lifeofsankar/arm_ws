#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace moveit::task_constructor;

// Update signature to accept the Node
Task createTask(rclcpp::Node::SharedPtr node) {
    Task t;
    t.stages()->setName("Trace Wall Task");
    t.loadRobotModel(node); // Helpful to explicitly load the model using the node

    // 1. Setup Solvers
    // FIX 1: Pass the 'node' to the PipelinePlanner constructor
    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

    // For linear movement (tracing the wall)
    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    
    // FIX 2: Use the new 'Factor' function names
    cartesian_planner->setMaxVelocityScalingFactor(0.1); 
    cartesian_planner->setMaxAccelerationScalingFactor(0.1);
    cartesian_planner->setStepSize(0.01); 

    // 2. Start from Current State
    t.add(std::make_unique<stages::CurrentState>("current_state"));

    // 3. Add the Wall to the Environment
    // {
    //     auto stage = std::make_unique<stages::ModifyPlanningScene>("spawn_wall");

    //     moveit_msgs::msg::CollisionObject wall_obj;
    //     wall_obj.id = "wall";
    //     wall_obj.header.frame_id = "world";
    //     wall_obj.operation = wall_obj.ADD;

    //     shape_msgs::msg::SolidPrimitive box;
    //     box.type = shape_msgs::msg::SolidPrimitive::BOX;
    //     box.dimensions = { 0.1, 1.0, 1.0 }; 
    //     wall_obj.primitives.push_back(box);

    //     geometry_msgs::msg::Pose pose;
    //     pose.position.x = 0.7; 
    //     pose.position.y = 0.0;
    //     pose.position.z = 0.5;
    //     pose.orientation.w = 1.0;
    //     wall_obj.primitive_poses.push_back(pose);

    //     stage->addObject(wall_obj);
    //     t.add(std::move(stage));
    // }

// 4. Move to "Safe Start" Pose
    {
        auto stage = std::make_unique<stages::MoveTo>("move_to_start", sampling_planner);
        stage->setGroup("arm");

        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.frame_id = "world";
        
        // POSITION: Comfortable reaching distance
        start_pose.pose.position.x = 0.5; 
        start_pose.pose.position.y = 0.0;
        start_pose.pose.position.z = 0.5;
        
        // ORIENTATION: Rotated 90 degrees around Y (Pointing Forward +X)
        // This is the "magic number" to point the gripper at the wall
        start_pose.pose.orientation.w = 0.707;
        start_pose.pose.orientation.x = 0.0;
        start_pose.pose.orientation.y = 0.707;
        start_pose.pose.orientation.z = 0.0;

        stage->setGoal(start_pose);
        t.add(std::move(stage));
    }

    // 5. Approach Wall
    {
        auto stage = std::make_unique<stages::MoveRelative>("approach_wall", cartesian_planner);
        stage->setGroup("arm");
        stage->setMinMaxDistance(0.05, 0.2); 
        
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.x = 1.0; 
        stage->setDirection(direction);
        t.add(std::move(stage));
    }

    // 6. Trace/Paint the Wall (Linear Motion)
    {
        auto stage = std::make_unique<stages::MoveRelative>("paint_stripe", cartesian_planner);
        stage->setGroup("arm");
        
        // CHANGE THIS LINE: Reduce the required distance
        // Old: stage->setMinMaxDistance(0.3, 0.5);
        stage->setMinMaxDistance(0.15, 0.3); 
        
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.y = -1.0; 
        stage->setDirection(direction);

        t.add(std::move(stage));
    }

    // 7. Retreat
    {
        auto stage = std::make_unique<stages::MoveRelative>("retreat", cartesian_planner);
        stage->setGroup("arm");
        stage->setMinMaxDistance(0.1, 0.2);
        
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.x = -1.0; 
        stage->setDirection(direction);
        t.add(std::move(stage));
    }

    return t;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hard_code_demo");
    std::thread spinning_thread([node] { rclcpp::spin(node); });

    spinning_thread.detach();
    
    // Pass the node to the function
    auto task = createTask(node);

    try {
        // FIX 3: init() is now void. It will throw an exception if it fails.
        // We do not put it inside an 'if' statement anymore.
        task.init(); 
        
        if (task.plan(5)) { 
            RCLCPP_INFO(node->get_logger(), "Task planning succeeded");
            task.introspection().publishSolution(*task.solutions().front());
            
            auto result = task.execute(*task.solutions().front());
            if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "Task execution failed");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Task planning failed");
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", ex.what());
    }

    std::this_thread::sleep_for(std::chrono::seconds(5)); 
    rclcpp::shutdown();
    return 0;
}