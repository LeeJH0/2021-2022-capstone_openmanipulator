cmake_minimum_required(VERSION 2.8.3)
project(img_window)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation

)

find_package (OpenCV REQUIRED)


add_message_files(DIRECTORY msg FILES gripper_state.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
# DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(img_window src/img_window.cpp)
target_link_libraries(img_window ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

