cmake_minimum_required(VERSION 3.0.2)

set (CMAKE_CXX_STANDARD 17)

project(car)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_runtime geometry_msgs sensor_msgs OpenCV cv_bridge image_transport message_generation)

find_package(Torch REQUIRED)
catkin_python_setup()

add_message_files(
  DIRECTORY
  msg
  FILES
  CarObject.msg
  CarObjects.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
  ${Torch_INCLUDE_DIRS}
)

add_executable(semantic_segmentation src/semantic_segmentation/semantic_segmentation.cpp)
add_dependencies(semantic_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_segmentation ${catkin_LIBRARIES} ${TORCH_LIBRARIES}) 

add_executable(simple_lane_follow src/simple_lane_follow/simple_lane_follow.cpp)
add_dependencies(simple_lane_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_lane_follow ${catkin_LIBRARIES} ${TORCH_LIBRARIES}) 

add_executable(end_to_end_lane_follow src/end_to_end_lane_follow/end_to_end_lane_follow.cpp)
add_dependencies(end_to_end_lane_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(end_to_end_lane_follow ${catkin_LIBRARIES} ${TORCH_LIBRARIES}) 

add_executable(manual_lane_follow src/manual_lane_follow/manual_lane_follow.cpp)
add_dependencies(manual_lane_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manual_lane_follow ${catkin_LIBRARIES} ${TORCH_LIBRARIES})

add_executable(overtaking src/overtaking/overtaking.cpp)
add_dependencies(overtaking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(overtaking ${catkin_LIBRARIES} ${TORCH_LIBRARIES}) 


# add_executable(car_tracker src/car_tracker/car_tracker.cpp)
# add_dependencies(car_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(car_tracker ${catkin_LIBRARIES} ${TORCH_LIBRARIES}) 


# catkin_install_python(PROGRAMS
#   scripts/car/object_detection/object_detection.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/car/semantic_segmentation/semantic_segmentation.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
