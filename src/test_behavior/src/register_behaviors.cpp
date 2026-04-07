#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <test_behavior/test_behavior.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace test_behavior
{
class TestBehaviorBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<TestBehavior>(factory, "TestBehavior", shared_resources);
    
  }
};
}  // namespace test_behavior

PLUGINLIB_EXPORT_CLASS(test_behavior::TestBehaviorBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
