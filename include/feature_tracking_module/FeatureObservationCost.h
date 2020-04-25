
#ifndef FTMODULE_FEATUREOBSERVATIONCOST_H_
#define FTMODULE_FEATUREOBSERVATIONCOST_H_

#include <Eigen/Core>
#include <array>

#include "confusion/UpdateMeasurement.h"
#include "confusion/utilities/Pose.h"
#include "confusion/utilities/ceres_utils.h"

#include <cv_bridge/cv_bridge.h>

namespace ftmodule {

struct PointFeatureCalibration {
  Eigen::Matrix<double, 3, 4> projMat_;
  double w_; //Weighting [pixels]. Inverse stddev.
  bool useLossFunction_ = true;
  double lossCoefficient_ = 5.0;
  double keypointMultilpierForVisualization_ = 1.0; //To allow visualization to be done at a different image size
};

class PointFeatureMeas;

class PointFeatureCost {
 public:
  PointFeatureCost(PointFeatureMeas *pointFeatureMeas) : pointFeatureMeas_(pointFeatureMeas) {}

  template <typename T>
  bool operator()(T const* t_w_i_, T const* q_w_i_,
                  T const* t_c_i_, T const* q_c_i_,
                  T const* p_w_, T* residual_) const;

  PointFeatureMeas *pointFeatureMeas_;
};

template <typename T>
bool PointFeatureCost::operator()(T const* t_w_i_, T const* q_w_i_,
                T const* t_c_i_, T const* q_c_i_,
                T const* p_w_, T* residual_) const {
#ifdef COST_DEBUG
  ros::Time t0 = ros::Time::now();
	std::cout << "Starting PointFeatureCost computation for t=" << pointFeatureMeas_->t() << " and feature " << pointFeatureMeas_->featureId_ << std::endl;
  std::cout << "Feature pos: " << pointFeatureMeas_->mapPoint_.t_w_f().transpose() << std::endl;
#endif

  //todo Duplicate computation for multiple feature obsevations from the same frame
  Eigen::Matrix<T, 3, 1> t_w_i(t_w_i_);
  Eigen::Quaternion<T> q_w_i(q_w_i_);
  confusion::Pose<T> T_w_i(t_w_i, q_w_i);

  Eigen::Matrix<T, 3, 1> t_c_i(t_c_i_);
  Eigen::Quaternion<T> q_c_i(q_c_i_);
  confusion::Pose<T> T_c_i(t_c_i, q_c_i);

  Eigen::Matrix<T, 3, 1> p_w(p_w_);

  //Get the estimated feature position in the camera frame
  //todo Could make this a little more efficient by doing the individual operations independently
  Eigen::Matrix<T, 3, 1> p_c_est = T_c_i * T_w_i.inverse() * p_w;
//  Eigen::VectorXd p_w_d(3);
//  confusion::getDoubles(p_w_,3,p_w_d.data());
//  std::cout << "p_w=" << p_w_d.transpose() << std::endl;

  //Project to image plane
  T zx_est = T(pointFeatureMeas_->pointFeatureCalibration_.projMat_(0, 0)) * (p_c_est(0) / p_c_est(2))
      + T(pointFeatureMeas_->pointFeatureCalibration_.projMat_(0, 2));
  T zy_est = T(pointFeatureMeas_->pointFeatureCalibration_.projMat_(1, 1)) * (p_c_est(1) / p_c_est(2))
      + T(pointFeatureMeas_->pointFeatureCalibration_.projMat_(1, 2));

  //Compute error
  residual_[0] = T(pointFeatureMeas_->pointFeatureCalibration_.w_) * (T(pointFeatureMeas_->detectedKeyPoint_.pt.x) - zx_est);
  residual_[1] = T(pointFeatureMeas_->pointFeatureCalibration_.w_) * (T(pointFeatureMeas_->detectedKeyPoint_.pt.y) - zy_est);

  //todo Temp for logging and analysis
  Eigen::VectorXd res(2);
  confusion::getDoubles(residual_,2,res.data());
  pointFeatureMeas_->error_ = res.norm();

#ifdef COST_DEBUG
  Eigen::VectorXd res(2);
  confusion::getDoubles(residual_,2,res.data());
//  std::cout << pointFeatureMeas_->name() << " cost: " << res.transpose() << std::endl;

  std::cout << "Feature " << pointFeatureMeas_->mapPoint_.mnId << ", check: " << pointFeatureMeas_->featureId_ << ". z_meas: " <<
            pointFeatureMeas_->detectedKeyPoint_.pt.x << "," << pointFeatureMeas_->detectedKeyPoint_.pt.y <<
            "; z_est: " << confusion::getDouble(zx_est) << "," << confusion::getDouble(zy_est) <<
            "; e: " << res.transpose() * res << std::endl;
#endif
//  std::cout << "pfc computation took " << (ros::Time::now()-t0).toSec() << " sec" << std::endl;

  return true;
}

} // namespace ftmodule

#endif /* FTMODULE_FEATUREOBSERVATIONCOST_H_ */
