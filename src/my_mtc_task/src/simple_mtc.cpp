#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

namespace mtc = moveit::task_constructor;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_mtc_node");

  mtc::Task task;
  task.stages()->setName("simple_task");
  task.loadRobotModel(node);

  const std::string group = "arm";   // ⚠️ change to your planning group

  // Planner
  auto sampling_planner =
      std::make_shared<mtc::solvers::PipelinePlanner>(node);

  // Current State
  auto current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(current));

  // MoveTo Stage
  auto move = std::make_unique<mtc::stages::MoveTo>("move_to_ready", sampling_planner);
  move->setGroup(group);
  move->setGoal("pose1");  // ⚠️ change to your named target
  task.add(std::move(move));

  try
  {
    task.plan();
    task.execute(*task.solutions().front());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "MTC failed: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}