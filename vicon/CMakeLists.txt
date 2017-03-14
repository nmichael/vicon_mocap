cmake_minimum_required(VERSION 2.8.3)
project(vicon)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake")
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  nav_msgs
  std_srvs
  vicon_driver
)

add_message_files(
  FILES
  Marker.msg
  Markers.msg
  Subject.msg
)

add_service_files(
  FILES
  SetPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs nav_msgs std_srvs vicon_driver
  DEPENDS Eigen3 Boost
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${BOOST_LIBRARY_DIRS}
  # Below is horrible hack to find the boost libs needed for the ViconSDK
  ${catkin_INCLUDE_DIRS}
)

add_executable(vicon src/vicon.cpp)
add_dependencies(vicon vicon_driver)
add_dependencies(vicon ${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(vicon
  vicon_driver
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_THREAD_LIBRARY}
)

add_executable(vicon_calibrate src/vicon_calibrate.cpp)
add_dependencies(vicon_calibrate vicon_driver)
add_dependencies(vicon_calibrate ${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(vicon_calibrate
  vicon_driver
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_THREAD_LIBRARY}
)

install(TARGETS vicon vicon_calibrate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY calib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)
