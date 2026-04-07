#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <sun_pose_finder/sun_pose_finder.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace sun_pose_finder
{
class SunPoseFinderBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<SunPoseFinder>(factory, "SunPoseFinder", shared_resources);
    
  }
};
}  // namespace sun_pose_finder

PLUGINLIB_EXPORT_CLASS(sun_pose_finder::SunPoseFinderBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
