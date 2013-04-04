cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
rosbuild_add_library(${PROJECT_NAME} src/LaneFinder.cpp)
rosbuild_add_library(${PROJECT_NAME} src/CollissionFinder.cpp)
rosbuild_add_library(${PROJECT_NAME} src/ApplyIPM.cpp)
rosbuild_add_library(${PROJECT_NAME} src/Movement.cpp)
rosbuild_add_executable(main src/main.cpp src/LaneFinder.cpp src/CollissionFinder.cpp src/ApplyIPM.cpp src/Movement.cpp)
target_link_libraries(main ${OpenCV_LIBRARIES})
rosbuild_add_executable(ipm src/IPM.cpp)
target_link_libraries(ipm ${OpenCV_LIBRARIES})
