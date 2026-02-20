#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <geometry_msgs/msg/vector3.hpp> // NEW: To receive X, Y, Z

using namespace moveit::task_constructor;

// NEW: Function accepts Depth (X), Width (Y), and Height (Z)
Task createTask(rclcpp::Node::SharedPtr node, double wall_x, double wall_y, double wall_z) {
    Task t;
    t.stages()->setName("Auto Zigzag Task");
    t.loadRobotModel(node); 

    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.1); 
    cartesian_planner->setMaxAccelerationScalingFactor(0.1);
    cartesian_planner->setStepSize(0.01); 

    t.add(std::make_unique<stages::CurrentState>("current_state"));

    // --- NEW MATH: FIND THE TOP LEFT CORNER ---
    // The camera centers the wall at Y=0.0 and Z=0.5. 
    // To start at the Top-Left, we divide the width and height by 2.
    double start_y = wall_y / 2.0;         // Shift Left
    double start_z = 0.5 + (wall_z / 2.0); // Shift Up

    // 1. Move to "Top Left" Start Pose
    {
        auto stage = std::make_unique<stages::MoveTo>("move_to_start", sampling_planner);
        stage->setGroup("arm");

        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.frame_id = "world";
        start_pose.pose.position.x = wall_x - 0.30; // Start 30cm away
        start_pose.pose.position.y = start_y; 
        start_pose.pose.position.z = start_z;
        
        start_pose.pose.orientation.w = 0.707;
        start_pose.pose.orientation.x = 0.0;
        start_pose.pose.orientation.y = 0.707;
        start_pose.pose.orientation.z = 0.0;

        stage->setGoal(start_pose);
        t.add(std::move(stage));
    }

    // 2. Approach Wall (Leaves 1cm gap for fingers)
    {
        auto stage = std::make_unique<stages::MoveRelative>("approach_wall", cartesian_planner);
        stage->setGroup("arm");
        stage->setMinMaxDistance(0.03, 0.10); 
        
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.x = 1.0; 
        stage->setDirection(direction);
        t.add(std::move(stage));
    }

    // --- NEW: THE DYNAMIC ZIGZAG LOOP ---
    double drop_distance = 0.10; // Drop 10cm for each new stroke
    
    // Calculate how many strokes we need to reach the bottom
    int num_strokes = std::ceil(wall_z / drop_distance);
    if (num_strokes < 1) num_strokes = 1; // Always do at least 1 stroke

    for (int i = 0; i < num_strokes; i++) {
        
        // A. Paint Stroke (Width of the wall)
        auto paint_stage = std::make_unique<stages::MoveRelative>("paint_stroke_" + std::to_string(i), cartesian_planner);
        paint_stage->setGroup("arm");
        paint_stage->setMinMaxDistance(wall_y * 0.5, wall_y); 
        
        geometry_msgs::msg::Vector3Stamped paint_dir;
        paint_dir.header.frame_id = "world";
        // If 'i' is even, go Right (-Y). If 'i' is odd, go Left (+Y).
        paint_dir.vector.y = (i % 2 == 0) ? -1.0 : 1.0; 
        paint_stage->setDirection(paint_dir);
        t.add(std::move(paint_stage));

        // B. Move Down (Skip this on the very last stroke)
        if (i < num_strokes - 1) {
            auto drop_stage = std::make_unique<stages::MoveRelative>("move_down_" + std::to_string(i), cartesian_planner);
            drop_stage->setGroup("arm");
            drop_stage->setMinMaxDistance(drop_distance * 0.9, drop_distance); 
            
            geometry_msgs::msg::Vector3Stamped drop_dir;
            drop_dir.header.frame_id = "world";
            drop_dir.vector.z = -1.0; // Move Down
            drop_stage->setDirection(drop_dir);
            t.add(std::move(drop_stage));
        }
    }

    // 3. Retreat
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
    auto node = rclcpp::Node::make_shared("auto_painter_node");
    std::thread spinning_thread([node] { rclcpp::spin(node); });
    spinning_thread.detach();
    
    // NEW: Listen for the Vector3 message (X, Y, Z)
    double w_x = 0.0, w_y = 0.0, w_z = 0.0;
    bool received = false;
    
    auto sub = node->create_subscription<geometry_msgs::msg::Vector3>(
        "/wall_dimensions", 10,
        [&](const geometry_msgs::msg::Vector3::SharedPtr msg) {
            w_x = msg->x;
            w_y = msg->y;
            w_z = msg->z;
            received = true;
        });

    RCLCPP_INFO(node->get_logger(), "Waiting for Python to send wall dimensions...");
    
    while (!received && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
   RCLCPP_INFO(node->get_logger(), "Received Wall! X: %.2f, Width: %.2f, Height: %.2f", w_x, w_y, w_z);

    // --- NEW: WORKSPACE SAFETY LIMITS ---
    // Prevent the robot from stretching past its physical 0.85m reach!
    if (w_y > 0.30) w_y = 0.30; // Cap stroke width to 50cm maximum
    if (w_z > 0.30) w_z = 0.30; // Cap paint height to 50cm maximum

    // Pass all 3 variables to the task!
    auto task = createTask(node, w_x, w_y, w_z);

    try {
        task.init(); 
        if (task.plan(5)) { 
            RCLCPP_INFO(node->get_logger(), "Task planning succeeded!");
            task.introspection().publishSolution(*task.solutions().front());
            task.execute(*task.solutions().front());
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