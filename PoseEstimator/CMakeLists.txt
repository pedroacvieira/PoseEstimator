# CMakeList.txt : CMake project for PoseEstimator, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("PoseEstimator")

# Add source to this project's executable.
add_executable (PoseEstimator "PoseEstimator.cpp" "PoseEstimator.h")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(DisplayImage ${OpenCV_LIBS})

# TODO: Add tests and install targets if needed.
