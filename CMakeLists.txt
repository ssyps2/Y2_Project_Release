cmake_minimum_required(VERSION 3.21)
project(visual_test)

set(CMAKE_CXX_STANDARD 14)

add_executable(visual_test main.cpp)

#------ opencv related config ------#
find_package(OpenCV REQUIRED core highgui imgproc)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(visual_test ${OpenCV_LIBS})