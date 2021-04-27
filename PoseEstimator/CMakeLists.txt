# CMakeList.txt : CMake project for PoseEstimator, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.9)

project ("PoseEstimator")

# Add source to this project's executable.
add_executable (PoseEstimator "PoseEstimator.cpp" "PoseEstimator.h")

# set(OPENCV_DIR C:/OpenCV/opencv4/install/x64/vc16)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(PoseEstimator ${OpenCV_LIBS})

# TODO: Add tests and install targets if needed.
