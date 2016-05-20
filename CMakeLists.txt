cmake_minimum_required(VERSION 2.8.3)
project(weed_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

find_package(OpenCV REQUIRED)


add_message_files(
  FILES
  detected_balls.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(weed_detection
#   src/${PROJECT_NAME}/weed_detection.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(weed_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(front_camera_node src/front_camera_node.cpp src/ball_detector.cpp src/Ctracker.cpp src/HungarianAlg.cpp src/Kalman.cpp)

add_executable(arm_camera_node src/arm_camera_node.cpp src/ball_detector.cpp src/Ctracker.cpp src/HungarianAlg.cpp src/Kalman.cpp)

add_executable(detection_manager src/detection_manager.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(front_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(arm_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(detection_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(front_camera_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
 )

 target_link_libraries(arm_camera_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
 )

 target_link_libraries(detection_manager
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
 )

# target_compile_options(detection_manager PRIVATE "-std=c++11")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS weed_detection weed_detection_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
