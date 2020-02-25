cmake_minimum_required(VERSION 2.8.3)
project(dist_autonomy)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  user_input
  # VR_exp_ROS/user_input
)

add_message_files(
  FILES
  Target_dist.msg
  object_position.msg
  #person_position.msg
)

add_message_files(
  FILES
  Target_dist.msg
  object_position.msg
  #person_position.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  #user_input
)


catkin_package( CATKIN_DEPENDS message_runtime )

include_directories(include ${catkin_INCLUDE_DIRS} ${user_input_INCLUDE_DIRS})

add_executable(visual_occlusion src/visual_occlusion.cpp)
target_link_libraries(visual_occlusion ${catkin_LIBRARIES})
add_dependencies(visual_occlusion ${catkin_EXPORTED_TARGETS})
add_dependencies(visual_occlusion visual_occlusion_generate_messages_cpp)


#add_dependencies(vr_exp_ros ${catkin_EXPORTED_TARGETS})

# add_executable(test_publish2 src/test_publish2.py)
# target_link_libraries(test_publish2 ${catkin_LIBRARIES})
# add_dependencies(test_publish2 ${catkin_EXPORTED_TARGETS})
# add_dependencies(test_publish2 test_publish2_generate_messages_cpp)
#
#
# add_executable(obj_detect_dist src/obj_detect_dist.py)
# target_link_libraries(obj_detect_dist ${catkin_LIBRARIES})
# add_dependencies(obj_detect_dist ${catkin_EXPORTED_TARGETS})
