#include <test_behavior/test_behavior.hpp>

#include "spdlog/spdlog.h"

namespace test_behavior
{
TestBehavior::TestBehavior(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}


BT::PortsList TestBehavior::providedPorts()
{
  // TODO: Define any input/output ports required for your behavior.
  return BT::PortsList({});
}

BT::KeyValueVector TestBehavior::metadata()
{
  // TODO: Define your behavior here.
  return { {"description", "test_behavior"},{"subcategory", "User Created Behaviors"} };
}

BT::NodeStatus TestBehavior::onStart()
{
  // TODO: Implement your synchronous initialization logic here (load/verify ports and initialize a task).
  // Publish messages to the UI like this:
  // shared_resources_->logger->publishInfoMessage(name(), "Hello, MoveIt Pro.");
  // shared_resources_->logger->publishWarnMessage(name(), "Hello, MoveIt Pro!");
  // shared_resources_->logger->publishFailureMessage(name(), "Goodbye, MoveIt Pro.");
  
  // Log messages to the console like this:
  // spdlog::info("Hello, MoveIt Pro.");
  // spdlog::warn("Hello, MoveIt Pro!");
  // spdlog::error("Goodbye, MoveIt Pro.");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TestBehavior::onRunning()
{
  // TODO: check if the task is still running, return SUCCESS if it is complete, otherwise return RUNNING.
  return BT::NodeStatus::SUCCESS;
}

void TestBehavior::onHalted()
{
  // OPTIONAL: Implement additional logic to handle cancellation of the behavior or objective running it.
}

}  // namespace test_behavior
