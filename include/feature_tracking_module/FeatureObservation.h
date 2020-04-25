
#ifndef FTMODULE_FEATUREOBSERVATION_H_
#define FTMODULE_FEATUREOBSERVATION_H_

#include <Eigen/Core>
#include <array>

#include <confusion/UpdateMeasurement.h>
#include <confusion/utilities/Pose.h>
#include <confusion/utilities/ceres_utils.h>
#include <cv_bridge/cv_bridge.h>

#include "feature_tracking_module/FeatureObservationCost.h"

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

class PointFeatureMeas : public confusion::UpdateMeasurement {
  friend PointFeatureCost;

 public:
  PointFeatureMeas(double t,
                   ORB_SLAM2::MapPoint &mapPoint,
                   const cv::KeyPoint &detectedKeyPoint,
                   confusion::Pose<double> &T_c_i,
                   const PointFeatureCalibration &pointFeatureCalibration,
                   bool inTrackingProblem,
                   int measurementType, //todo No default value for this for now to make sure I'm not creating any without specifying Tracking/Smoothing
                   int keyPointIndex, // This is needed just to associate observed map points to the current place in the linked frame when solving the batch problem
                   int startingStateParamIndex = 0) :
      UpdateMeasurement(measurementType, t, "Pf", true), //todo Temp Check if the map point is valid before solving each time
      mapPoint_(mapPoint), detectedKeyPoint_(detectedKeyPoint),
      pointFeatureCalibration_(pointFeatureCalibration), T_c_i_(T_c_i),
      startingStateParamIndex_(startingStateParamIndex),
      inTrackingProblem_(inTrackingProblem),
      keyPointIndex_(keyPointIndex) {
    featureId_ = mapPoint_.mnId;

    if (!inTrackingProblem_)
      std::cout << "WARNING: PointFeatureMeas created for use in smoothing!!!!!!!!\n\n\n" << std::endl;
  }

  ~PointFeatureMeas() {
//    if (initialized() && isEnabled()) {
//      mapPoint_.removeTrackingObservation(t());
//    }
//    std::cout << "Removing feature observation in meas destructor for feature " << featureId_ << " at t=" << t() << std::endl;
  }

  //todo Another check to test that the residual function will be used in the next optimization, without having to first build the problem
  bool isMapPointGood() {
    return !mapPoint_.isBad() &&
        ((inTrackingProblem_ && mapPoint_.activeForTracking()) ||
           (!inTrackingProblem_ && mapPoint_.activeForSmoothing()));
  }

  bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                          std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                          std::vector<size_t> &stateParameterIndexVector,
                          std::vector<double *> &staticParameterDataVector) override {
//    if (!mapPoint_.activeForTracking() || mapPoint_.isBad()) {
//      std::cout << "--> Not creating PointFeatureCost for feature " << mapPoint_.mnId <<
//          " because activeForTracking=" << mapPoint_.activeForTracking() <<
//          " and bad=" << mapPoint_.isBad() << std::endl;
//      return false;
//    }

    if (!isMapPointGood())
      return false;

//    if (mapPoint_.isBad() ||
//        (inTrackingProblem_ && !mapPoint_.activeForTracking()) ||
//        (!inTrackingProblem_ && !mapPoint_.activeForSmoothing())) {
////      std::cout << "Not creating PointFeatureCost for feature " << mapPoint_.mnId << " at t=" << t() << "; isBad=" << mapPoint_.isBad() << "; inTrackingProblem=" << inTrackingProblem_ << std::endl;
//      return false;
//    }
//    std::cout << "Creating PointFeatureCost for feature " << mapPoint_.mnId << " at t=" << t() << "; isBad=" << mapPoint_.isBad() << "; inTrackingProblem=" << inTrackingProblem_ << std::endl;

    std::unique_ptr<ceres::CostFunction> costFunctionPtr_(
        new ceres::AutoDiffCostFunction<PointFeatureCost, 2, 3, 4, 3, 4, 3>(new PointFeatureCost(this)));

    stateParameterIndexVector.push_back(0);
    stateParameterIndexVector.push_back(1);

    staticParameterDataVector.push_back(T_c_i_.trans.data());
    staticParameterDataVector.push_back(T_c_i_.rot.coeffs().data());
    staticParameterDataVector.push_back(mapPoint_.getPositionDataPtrForTracking());

    costFunctionPtr = std::move(costFunctionPtr_);

    if (!pointFeatureCalibration_.useLossFunction_)
      lossFunctionPtr.reset();
    else {
      std::unique_ptr<ceres::LossFunction>
          lossFunctionPtr_(new ceres::HuberLoss(pointFeatureCalibration_.lossCoefficient_));
      lossFunctionPtr = std::move(lossFunctionPtr_);
    }

    return true;
  }

  void disable() {
//    mapPoint_.removeTrackingObservation(t());
    enable_ = false;
    initialized_ = false;
  }

  void updateMapPointPositionFromOptimization() {
    mapPoint_.setPositionFromTracker();
  }

  void switchToSmoothing() {
    if (!inTrackingProblem_)
      std::cout << "WARNING: PointFeatureCost::switchToSmoothing called on an observation already in the Smoothing problem." << std::endl;

    inTrackingProblem_ = false;
  }

  int residualDimension() override { return 2; }
  long unsigned int featureId() { return featureId_; }
  const confusion::Pose<double>& T_c_i() const { return T_c_i_; }
  const cv::KeyPoint &keyPoint() const { return detectedKeyPoint_; }
  const cv::Point2f &featureLocationForVisualization() const { return detectedKeyPoint_.pt; }
  const double &getPreviousResidualNorm() { return error_; }
  int keyPointIndex() const { return keyPointIndex_; }

  ORB_SLAM2::MapPoint &mapPoint_;

protected:
  const cv::KeyPoint &detectedKeyPoint_;
  const PointFeatureCalibration &pointFeatureCalibration_;

  confusion::Pose<double> &T_c_i_;
  const int startingStateParamIndex_; ///< Indicates the index of the state parameters which contains the position of the measured body pose. It is assumed that the orientation is then the next parameter.

  long unsigned int featureId_;
  bool inTrackingProblem_; //True means it is in the tracking problem. False means it is in the smoothing problem.
  const int keyPointIndex_;

  mutable double error_; ///< Used for logging and analysis only
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

#endif /* FTMODULE_FEATUREOBSERVATION_H_ */
