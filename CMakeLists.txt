cmake_minimum_required(VERSION 3.21)
project(Y2_Project_Release)

set(CMAKE_CXX_STANDARD 14)

add_executable(Y2_Project_Release main.cpp)

#------ opencv related config ------#
find_package(OpenCV REQUIRED core highgui imgproc)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(Y2_Project_Release ${OpenCV_LIBS})