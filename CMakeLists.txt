cmake_minimum_required(VERSION 2.8.3)
project(vicon_mocap)
find_package(catkin REQUIRED)
catkin_metapackage()

add_subdirectory(libvicon_driver)
add_subdirectory(vicon)
add_subdirectory(vicon_odom)