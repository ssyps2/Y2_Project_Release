cmake_minimum_required(VERSION 3.5)
project(vehicle_final)

set(CMAKE_CXX_STANDARD 14)

add_executable(vehicle_final main.cpp devices.cpp lineTrack.cpp visualScan.cpp opencv_aee.cpp)

#------ opencv related config ------#
find_library(WIRINGPI_LIBRARIES NMAES wiringPi)
find_path(WIRINGPI_INCLUDE_DIRS NMAES wiringPi.h)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wiringPi DEFAULT_MSG WIRINGPI_LIBRARIES WIRINGPI_INCLUDE_DIRS)

find_package(OpenCV REQUIRED)
find_package(WiringPi REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${WIRINGPI_INCLUDE_DIRS})
target_link_libraries(vehicle_final ${OpenCV_LIBS} ${WIRINGPI_LIBRARIES})

add_definitions(/home/pi/wiringPi-8d188fa/wiringPi -lwiringPi)