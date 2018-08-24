set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(ipa_aruco_marker_detection)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  OpenCV
)

#catkin_package(
#INCLUDE_DIRS include
#LIBRARIES ${PROJECT_NAME}
#CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#)

include_directories(
include

  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
src/detect_markers.cpp
)

add_executable(${PROJECT_NAME}_node src/main.cpp )

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}

   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   )

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
 #  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 #)

## Mark cpp header files for installation
#install(DIRECTORY include/
 #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #  )

add_executable(camera_calibration src/camera_calibration.cpp)
target_link_libraries(camera_calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(create_aruco_markers_4x4 src/create_aruco_markers_4x4.cpp)
target_link_libraries(create_aruco_markers_4x4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(create_aruco_grid_board src/create_aruco_grid_board.cpp)
target_link_libraries(create_aruco_grid_board ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

