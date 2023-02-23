cmake_minimum_required(VERSION 3.0)
project(crazyflie_online_tracker)
find_package(catkin REQUIRED COMPONENTS rospy message_generation dynamic_reconfigure geometry_msgs)
catkin_python_setup()
generate_dynamic_reconfigure_options(
  cfg/HoverController.cfg
)

add_message_files(
  FILES 
  Mode.msg
  SetpointHL.msg
  CommandCF.msg
  TargetState.msg
  CrazyflieState.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)

