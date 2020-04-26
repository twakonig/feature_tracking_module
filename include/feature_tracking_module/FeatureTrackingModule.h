//
// Created by tim on 25.04.20.
//

#ifndef FTMODULE_FEATURETRACKINGMODULE_H
#define FTMODULE_FEATURETRACKINGMODULE_H

#include <boost/property_tree/ptree.hpp>
#include <confusion/ConFusor.h>

#include "feature_tracking_module/SensorEnumDefinition.h"
#include "feature_tracking_module/FeatureObservation.h"


namespace ftmodule {

class FeatureTrackingModule {
public:
  FeatureTrackingModule(ros::NodeHandle &nh, confusion::ConFusor &conFusor, confusion::Pose<double> &T_c_i, boost::property_tree::ptree& pt);

  void SetCameraCalibration(const Eigen::Matrix<double,3,4> &proj_mat);

  void AddFeatureObservationsCallback(/*todo data in */);

  void ProcessingAfterOptimization();


private:
  void AddAndRemoveMapPoints();
  void CopyOutMapAfterOptimization();
  void Visualize();


  confusion::ConFusor &conFusor_;
  confusion::Pose<double> &T_c_i_;
  PointFeatureCalibration calibration_;

  //TODO Need a reference to the Map
};

} // namespace ftmodule

#endif //FTMODULE_FEATURETRACKINGMODULE_H
