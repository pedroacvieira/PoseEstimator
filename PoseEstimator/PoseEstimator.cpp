// PoseEstimator.cpp : Defines the entry point for the application.

#include "PoseEstimator.h"

#include <iostream>

#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>

namespace DLR {

/**
  * C'Tor of PoseEstimator. It shall initialize all ressources
  * needed for the estimation process.
  * \param camera_matrix intrinsics of the given camera
  */
PoseEstimator::PoseEstimator(const cv::Mat& camera_matrix) : m_camera_matrix(camera_matrix)
{
  m_odometry = cv::rgbd::RgbdICPOdometry(m_camera_matrix);
}

/**
  * Estimate the pose between both given image pairs.
  * \param source source image pair
  * \param target target image pair
  * \return 4x4 pose representing relative pose from
  *         source to target
  */
Eigen::Matrix4d PoseEstimator::estimate_pose(const rgbd_pair_t& source,
                                             const rgbd_pair_t& target)
{
  cv::rgbd::OdometryFrame source_rgbd_frame(source.first, source.second);
  cv::rgbd::OdometryFrame target_rgbd_frame(target.first, target.second);

  cv::Mat output_pose = cv::Mat(4, 4, CV_64FC1);

  m_odometry.compute(cv::Ptr<cv::rgbd::OdometryFrame>(&source_rgbd_frame),
                     cv::Ptr<cv::rgbd::OdometryFrame>(&target_rgbd_frame),
                     output_pose);

  Eigen::Matrix4d target_pose;
  cv::cv2eigen(output_pose, target_pose);

  return target_pose;
}

}  // namespace DLR

int main()
{
  std::cout << "Hello CMake." << std::endl;
  return 0;
}
