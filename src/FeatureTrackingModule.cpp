//
// Created by tim on 25.04.20.
//

#include "feature_tracking_module/FeatureTrackingModule.h"

namespace ftmodule {

FeatureTrackingModule::FeatureTrackingModule(
    ros::NodeHandle &nh,
    confusion::ConFusor &conFusor,
    confusion::Pose<double> &T_c_i,
    boost::property_tree::ptree& pt):
          conFusor_(conFusor), T_c_i_(T_c_i) {
  calibration_.w_ = 1.0 / pt.get<double>("ftmodule.feature_observation_stddev");

  //todo Create subscriber for adding observations
}

void FeatureTrackingModule::SetCameraCalibration(const Eigen::Matrix<double,3,4> &proj_mat) {
  calibration_.projMat_ = proj_mat;
}

void FeatureTrackingModule::AddFeatureObservationsCallback(/*todo data in */) {
  //TODO This stuff should come from the input args------------------------
  double t = 0.0;
  MapPoint map_point(0, Eigen::Vector3d::Zero());
  cv::KeyPoint detected_keypoint;
  //----------------------------------------------------------------------------

  conFusor_.addUpdateMeasurement(std::make_shared<FeatureObservation>(
      t, map_point, detected_keypoint, T_c_i_, calibration_, FEATUREOBS));
}

void FeatureTrackingModule::ProcessingAfterOptimization() {
  AddAndRemoveMapPoints();
  CopyOutMapAfterOptimization();
  Visualize();
}

void FeatureTrackingModule::AddAndRemoveMapPoints() {
  //TODO
}

void FeatureTrackingModule::CopyOutMapAfterOptimization() {
  //TODO
}

void FeatureTrackingModule::Visualize() {
  //TODO tfs, point clouds, etc (?)
}

} // namespace ftmodule