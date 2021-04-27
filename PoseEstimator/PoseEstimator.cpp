// PoseEstimator.cpp : Defines the entry point for the application.

#include "PoseEstimator.h"

#include <iostream>

#include <opencv2/nonfree/features2d.hpp>

namespace DLR {
  /**
    * C'Tor of PoseEstimator. It shall initialize all ressources
    * needed for the estimation process.
    * \param camera_matrix intrinsics of the given camera
    */
  PoseEstimator::PoseEstimator(const cv::Mat& camera_matrix) : m_camera_matrix(camera_matrix)
  {
    // write additional code here
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
    // Detect SIFT features on source RGB image
    cv::SiftFeatureDetector source_detector;
    std::vector<cv::KeyPoint> source_keypoints;
    source_detector.detect(source.first, source_keypoints);

    // Detect SIFT features on target RGB image
    cv::SiftFeatureDetector target_detector;
    std::vector<cv::KeyPoint> target_keypoints;
    target_detector.detect(target.first, target_keypoints);
  }

}

int main()
{
	std::cout << "Hello CMake." << std::endl;
	return 0;
}
