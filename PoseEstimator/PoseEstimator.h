/* ************************************
 * Pose Estimation
 *
 * This class shall be able to estimate
 * the pose between two given images
 * without scale error.
 *
 * For implementation expect all common
 * 3rdparty libraries to be available
 * (e.g. Eigen, OpenCV, Boost) and
 * already embedded into the environment
 *
 * ************************************/

#pragma once

/*! Avoid missing header error */
#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT

#include <opencv2/core.hpp>
#include <opencv2/rgbd.hpp>
#include <Eigen/Core>

#include <utility>

namespace DLR {

/**
  * typedef for syntactic sugar
  */
using rgbd_pair_t = std::pair<cv::Mat, cv::Mat>;

class PoseEstimator {
public:
  /**
    * C'Tor of PoseEstimator. It shall initialize all ressources
    * needed for the estimation process.
    * \param camera_matrix intrinsics of the given camera
    */
  PoseEstimator(const cv::Mat& camera_matrix);

  /**
    * Estimate the pose between both given image pairs.
    * \param source source image pair
    * \param target target image pair
    * \return 4x4 pose representing relative pose from
    *         source to target
    */
  Eigen::Matrix4d estimate_pose(const rgbd_pair_t& source,
                                const rgbd_pair_t& target);

private:
  cv::rgbd::RgbdICPOdometry m_odometry; /*!< Odometry object to perform the pose estimation */

  cv::Mat m_camera_matrix; /*!< Camera intrinsic calibration matrix */
};

} /*! namespace DLR */
