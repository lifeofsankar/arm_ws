#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <geometry_msgs/msg/pose.hpp>

using namespace moveit::task_constructor;

Task createTask(rclcpp::Node::SharedPtr node, geometry_msgs::msg::Pose target_pose) {
    Task t;
    t.stages()->setName("Pick and Place Task");
    t.loadRobotModel(node); 

    // 1. Setup Solvers
    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.01); 
    cartesian_planner->setMaxAccelerationScalingFactor(0.1);
    cartesian_planner->setStepSize(0.01); 

    auto joint_planner = std::make_shared<solvers::JointInterpolationPlanner>();

    // 2. Start State
    t.add(std::make_unique<stages::CurrentState>("current_state"));

    // --- THE PICK SEQUENCE ---

    {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("snap_bounds");
        stage->setCallback([](const auto& scene, const auto& /*p*/) {
            scene->getCurrentStateNonConst().enforceBounds();
        });
        t.add(std::move(stage));
    }

    // 3. Open the Gripper
    {
        auto stage = std::make_unique<stages::MoveTo>("open_hand", joint_planner);
        stage->setGroup("gripper"); 
        stage->setGoal("open");     
        t.add(std::move(stage));
    }

    // 4. Move to Pre-Grasp Pose (Hovering above target)
    {
        auto stage = std::make_unique<stages::MoveTo>("move_to_pre_grasp", sampling_planner);
        stage->setGroup("arm");

        geometry_msgs::msg::PoseStamped hover_pose;
        hover_pose.header.frame_id = "world";
        
        hover_pose.pose.position.x = target_pose.position.x; 
        hover_pose.pose.position.y = target_pose.position.y;
        hover_pose.pose.position.z = target_pose.position.z + 0.15; 
        
        hover_pose.pose.orientation.w = 0.0;
        hover_pose.pose.orientation.x = 1.0;
        hover_pose.pose.orientation.y = 0.0;
        hover_pose.pose.orientation.z = 0.0;

        stage->setGoal(hover_pose);
        t.add(std::move(stage));
    }

    // 5. Approach (Move straight down)
    {
        auto stage = std::make_unique<stages::MoveRelative>("approach_object", cartesian_planner);
        stage->setGroup("arm");
        stage->setMinMaxDistance(0.01, 0.15); 
        
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.z = -1.0; 
        stage->setDirection(direction);
        t.add(std::move(stage));
    }

    // --- THE OFFICIAL GRASP SEQUENCE ---

    // 6. Allow Collisions (Turn off alarms between fingers and object!)
    {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("allow_collision");
        stage->allowCollisions("object", 
            t.getRobotModel()->getJointModelGroup("gripper")->getLinkModelNamesWithCollisionGeometry(), 
            true);
        t.add(std::move(stage));
    }

    // 7. Close the Gripper (Squeeze!)
    {
        auto stage = std::make_unique<stages::MoveTo>("close_hand", joint_planner);
        stage->setGroup("gripper"); 
        stage->setGoal("closed");   
        t.add(std::move(stage));
    }

    // 8. Attach Object ("Digital Glue")
    // {
    //     auto stage = std::make_unique<stages::ModifyPlanningScene>("attach_object");
    //     stage->attachObject("object", "panda_hand"); 
    //     t.add(std::move(stage));
    // }

    // -----------------------------------

    // 9. Lift Object (Move straight up)
    {
        auto stage = std::make_unique<stages::MoveRelative>("lift_object", cartesian_planner);
        stage->setGroup("arm");
        stage->setMinMaxDistance(0.05, 0.15); 
        
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.z = 1.0; 
        stage->setDirection(direction);
        t.add(std::move(stage));
    }

    // --- THE PLACE SEQUENCE ---

    // 10. Move to Drop-off Position
    {
        auto stage = std::make_unique<stages::MoveTo>("move_to_place", sampling_planner);
        stage->setGroup("arm");

        geometry_msgs::msg::PoseStamped place_pose;
        place_pose.header.frame_id = "world";
        place_pose.pose.position.x = target_pose.position.x; 
        place_pose.pose.position.y = target_pose.position.y + 0.30; // Move 30cm to the left!
        place_pose.pose.position.z = target_pose.position.z + 0.15; // Keep hover height
        
        // Keep the gripper pointing straight down
        place_pose.pose.orientation.w = 0.0;
        place_pose.pose.orientation.x = 1.0;
        place_pose.pose.orientation.y = 0.0;
        place_pose.pose.orientation.z = 0.0;

        stage->setGoal(place_pose);
        t.add(std::move(stage));
    }

   // 11. Lower the Object
    {
        auto stage = std::make_unique<stages::MoveRelative>("lower_object", cartesian_planner);
        stage->setGroup("arm");
        // NEW: Only lower by 8cm max so the fingers don't hit the table when opening!
        stage->setMinMaxDistance(0.01, 0.08); 
        
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.z = -1.0; // Straight down
        stage->setDirection(direction);
        t.add(std::move(stage));
    }

    // 12. Open the Gripper (Release!)
    {
        auto stage = std::make_unique<stages::MoveTo>("release_object", joint_planner);
        stage->setGroup("gripper"); 
        stage->setGoal("open");   
        t.add(std::move(stage));
    }

    // 13. Retreat
    {
        auto stage = std::make_unique<stages::MoveRelative>("retreat", cartesian_planner);
        stage->setGroup("arm");
        stage->setMinMaxDistance(0.05, 0.15); 
        
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.z = 1.0; // Straight up
        stage->setDirection(direction);
        t.add(std::move(stage));
    }

    return t;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true); 
    options.parameter_overrides({{"start_state_max_bounds_error", 0.1}});
    
    auto node = rclcpp::Node::make_shared("pick_place_node", options);

    // Setup an executor to spin the node in the background
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinning_thread([&executor] { executor.spin(); });

    geometry_msgs::msg::Pose target_pose;
    bool target_received = false;
    auto sub = node->create_subscription<geometry_msgs::msg::Pose>(
        "/grasp_target", 10,
        [&](const geometry_msgs::msg::Pose::SharedPtr msg) {
            target_pose = *msg;
            target_received = true;
        });

    RCLCPP_INFO(node->get_logger(), "Waiting for Python vision system...");
    while (rclcpp::ok() && !target_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_INFO(node->get_logger(), "Target Acquired! X: %.2f, Y: %.2f", target_pose.position.x, target_pose.position.y);

    // CREATE AND PLAN THE TASK
    auto task = createTask(node, target_pose);

    try {
        task.init(); 
        if (task.plan(5)) { 
            RCLCPP_INFO(node->get_logger(), "Task planning succeeded!");
            task.introspection().publishSolution(*task.solutions().front());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Task planning failed");
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", ex.what());
    }

    // --- THIS IS THE MAGIC ---
    // Keep the main thread alive indefinitely until you press Ctrl+C
    RCLCPP_INFO(node->get_logger(), "Plan published to RViz! Press Ctrl+C to shut down.");
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Clean up
    executor.cancel();
    spinning_thread.join();
    rclcpp::shutdown();
    return 0;
}