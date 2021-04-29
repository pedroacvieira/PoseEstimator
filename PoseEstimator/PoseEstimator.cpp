/*! PoseEstimator.cpp : Defines the entry point for the application */

#include "PoseEstimator.h"

#include <iostream>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Core>

#define SHOW_IMAGES false

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
  /*! Needs to be allocated dynamically */
  cv::Ptr<cv::rgbd::OdometryFrame>
    source_frame_ptr = new cv::rgbd::OdometryFrame(source.first, source.second);
  cv::Ptr<cv::rgbd::OdometryFrame>
    target_frame_ptr = new cv::rgbd::OdometryFrame(target.first, target.second);


  cv::Mat output_pose = cv::Mat(4, 4, CV_64FC1);

  /*! TODO: Search the scale factor used in the cv::rgbd functions */
  bool is_valid = m_odometry.compute(source_frame_ptr, target_frame_ptr, output_pose);

  Eigen::Matrix4d target_pose;
  /*! Use zero matrices to verify if result is valid */
  if (!is_valid)
  {
    target_pose.setZero();
  }
  else
  {
    cv::cv2eigen(output_pose, target_pose);
    target_pose /= target_pose(3, 3); /*! Just in case */
  }

  return target_pose;
}

} /*! namespace DLR */

int main()
{
  /*! Set input filenames */
  /*! TODO: Read filenames and path from config file */
  std::string source_rgb{ "1341839327.392692.png" };
  std::string source_depth{ "1341839327.392719.png" };
  std::string target_rgb{ "1341839332.820842.png" };
  std::string target_depth{ "1341839332.820854.png" };

  /*! TODO: Use boost to get cwd or change to C++17 */
  std::string repo_path { "C:/Users/Pedro/pose-estimator/" };
  std::string dir_path{ "data/images/fr3_sample/" };
  std::string path = repo_path + dir_path;

  /*! Scale depth images from 16 bit int to float in [0, 1] */
  double scale_factor = 1.0 / (255.0 * 255.0);
  
  /*! Read source frame pair */
  DLR::rgbd_pair_t source;
  source.first = cv::imread(path + source_rgb, cv::IMREAD_GRAYSCALE);
  source.second = cv::imread(path + source_depth, cv::IMREAD_ANYDEPTH);
  source.second.convertTo(source.second, CV_32FC1, scale_factor);

  /*! Read target frame pair */
  DLR::rgbd_pair_t target;
  target.first = cv::imread(path + target_rgb, cv::IMREAD_GRAYSCALE);
  target.second = cv::imread(path + target_depth, cv::IMREAD_ANYDEPTH);
  target.second.convertTo(target.second, CV_32FC1, scale_factor);

  /*! Show image for debugging */
  if (SHOW_IMAGES)
  {
    cv::imshow("Display Test", target.second);
    cv::waitKey(0); /*! Wait for a keystroke in the window */
  }
  
  /*! Initialize camera matrix */
  double fx = 535.4;
  double fy = 539.2;
  double cx = 320.1;
  double cy = 247.6;
  Eigen::Matrix3d intrinsics;
  intrinsics << fx, 0, cx, 0, fy, cy, 0, 0, 1; /*! It's simpler to intialize an Eigen matrix*/

  /*! Initialize pose estimator */
  cv::Mat intrinsics_cv = cv::Mat(3, 3, CV_64FC1, intrinsics.data());
  DLR::PoseEstimator pose_estimator(intrinsics_cv);
  Eigen::Matrix4d pose = pose_estimator.estimate_pose(source, target);

  if (pose.isZero())
  {
    std::cout << "Pose estimation unsuccessful" << std::endl;
  }
  else
  {
    std::cout << "Relative pose:\n" << pose << std::endl;
  }

  return 0;
}
