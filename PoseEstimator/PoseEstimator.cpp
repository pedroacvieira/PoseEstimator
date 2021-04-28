/*! PoseEstimator.cpp : Defines the entry point for the application */

#include "PoseEstimator.h"

#include <filesystem>
#include <iostream>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
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

  bool is_valid = m_odometry.compute(cv::Ptr<cv::rgbd::OdometryFrame>(&source_rgbd_frame),
                                     cv::Ptr<cv::rgbd::OdometryFrame>(&target_rgbd_frame),
                                     output_pose);

  Eigen::Matrix4d target_pose;
  if (is_valid)
  {
    cv::cv2eigen(output_pose, target_pose);
  }

  return target_pose;
}

}  /*! namespace DLR */

int main()
{
  /*! Set input filenames */
  /*! TODO: Read filenames and path from config file */
  std::string source_rgb{ "1341839327.392692.png" };
  std::string source_depth{ "1341839327.392719.png" };
  std::string target_rgb{ "1341839330.905589.png" };
  std::string target_depth{ "1341839330.905598.png" };

  std::string path{ "C:/Users/Pedro/pose-estimator/data/images/fr3_sample/" };

  /*! Read source frame pair */
  DLR::rgbd_pair_t source;
  source.first = cv::imread(path + source_rgb);
  source.second = cv::imread(path + source_depth, cv::IMREAD_ANYDEPTH);

  /*! Read target frame pair */
  DLR::rgbd_pair_t target;
  target.first = cv::imread(path + target_rgb);
  target.second = cv::imread(path + target_depth, cv::IMREAD_ANYDEPTH);

  /*! Initialize camera matrix */
  double fx = 535.4;
  double fy = 539.2;
  double cx = 320.1;
  double cy = 247.6;
  Eigen::Matrix3d intrinsics;
  intrinsics << fx, 0, cx, 0, fy, cy, 0, 0, 1;

  /*! Initialize pose estimator */
  cv::Mat intrinsics_cv;
  cv::eigen2cv(intrinsics, intrinsics_cv);
  DLR::PoseEstimator pose_estimator(intrinsics_cv);
  // Eigen::Matrix4d pose = pose_estimator.estimate_pose(source, target);

  // cv::imshow("Display Test", source.second);
  // cv::waitKey(0); /*! Wait for a keystroke in the window */

  std::cout << "Hello CMake." << std::endl;
  return 0;
}
