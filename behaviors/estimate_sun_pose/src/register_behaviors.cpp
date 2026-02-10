#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <estimate_sun_pose/estimate_sun_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace estimate_sun_pose
{
class EstimateSunPoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<EstimateSunPose>(factory, "EstimateSunPose", shared_resources);
    
  }
};
}  // namespace estimate_sun_pose

PLUGINLIB_EXPORT_CLASS(estimate_sun_pose::EstimateSunPoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
