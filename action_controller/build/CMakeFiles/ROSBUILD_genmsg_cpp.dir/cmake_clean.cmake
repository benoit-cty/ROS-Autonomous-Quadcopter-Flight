FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/action_controller/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/action_controller/MultiDofFollowJointTrajectoryAction.h"
  "../msg_gen/cpp/include/action_controller/MultiDofFollowJointTrajectoryGoal.h"
  "../msg_gen/cpp/include/action_controller/MultiDofFollowJointTrajectoryActionGoal.h"
  "../msg_gen/cpp/include/action_controller/MultiDofFollowJointTrajectoryResult.h"
  "../msg_gen/cpp/include/action_controller/MultiDofFollowJointTrajectoryActionResult.h"
  "../msg_gen/cpp/include/action_controller/MultiDofFollowJointTrajectoryFeedback.h"
  "../msg_gen/cpp/include/action_controller/MultiDofFollowJointTrajectoryActionFeedback.h"
  "../msg/MultiDofFollowJointTrajectoryAction.msg"
  "../msg/MultiDofFollowJointTrajectoryGoal.msg"
  "../msg/MultiDofFollowJointTrajectoryActionGoal.msg"
  "../msg/MultiDofFollowJointTrajectoryResult.msg"
  "../msg/MultiDofFollowJointTrajectoryActionResult.msg"
  "../msg/MultiDofFollowJointTrajectoryFeedback.msg"
  "../msg/MultiDofFollowJointTrajectoryActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
