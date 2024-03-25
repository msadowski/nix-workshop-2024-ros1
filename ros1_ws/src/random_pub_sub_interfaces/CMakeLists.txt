cmake_minimum_required(VERSION 3.0.2)
project(random_pub_sub_interfaces)

find_package(catkin REQUIRED message_generation)

add_message_files(
  FILES
  RandomRange.msg
)

add_service_files(
  FILES
  SetInt32.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
)
