cmake_minimum_required(VERSION 2.8.3)
project(ipc_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
    MultiArrayDimension.msg
    MultiArrayLayout.msg
    gpu_handle.msg
    data_size.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
