cmake_minimum_required(VERSION 3.16)
project(encos_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg" "msg/MotorState.msg" "msg/MotorFault.msg" "msg/MotorProfile.msg"
  "msg/RawFrame.msg" "msg/ControlSample.msg" "msg/CommunicationStats.msg" "msg/ParameterValue.msg"
  "srv/Arm.srv" "srv/Stop.srv" "srv/ResetFault.srv" "srv/GetProfile.srv" "srv/DiscoverMotor.srv"
  "srv/ReadParameter.srv" "srv/WriteParameter.srv" "srv/SetZero.srv" "srv/SetMotorId.srv"
  "srv/ResetMotorId.srv" "srv/SetBrake.srv" "action/RunExperiment.action"
  DEPENDENCIES std_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
