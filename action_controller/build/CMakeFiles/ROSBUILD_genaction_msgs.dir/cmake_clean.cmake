FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/action_controller/msg"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
