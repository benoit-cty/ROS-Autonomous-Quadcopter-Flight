FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/action_controller/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/action_controller/msg/__init__.py"
  "../src/action_controller/msg/_MultiDofFollowJointTrajectoryAction.py"
  "../src/action_controller/msg/_MultiDofFollowJointTrajectoryGoal.py"
  "../src/action_controller/msg/_MultiDofFollowJointTrajectoryActionGoal.py"
  "../src/action_controller/msg/_MultiDofFollowJointTrajectoryResult.py"
  "../src/action_controller/msg/_MultiDofFollowJointTrajectoryActionResult.py"
  "../src/action_controller/msg/_MultiDofFollowJointTrajectoryFeedback.py"
  "../src/action_controller/msg/_MultiDofFollowJointTrajectoryActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
