//
// Created by tim on 25.04.20.
//

#ifndef FTMODULE_FEATURETRACKINGMODULE_H
#define FTMODULE_FEATURETRACKINGMODULE_H

#include <boost/property_tree/ptree.hpp>
#include <thread>
#include <mutex>
#include <confusion/ConFusor.h>
#include "target_tracking/Map.hpp"
#include "target_tracking/RosModule.hpp"
#include "target_tracking/FeatureTriangulation.hpp"

#include "feature_tracking_module/SensorEnumDefinition.h"
#include "feature_tracking_module/FeatureObservation.h"


namespace ftmodule {

class FeatureTrackingModule {
public:
  FeatureTrackingModule(ros::NodeHandle &nh, confusion::ConFusor &conFusor, confusion::Pose<double> &T_c_i, boost::property_tree::ptree& pt, target_tracking::RosModule &rosObj);

  void SetCameraCalibration(const Eigen::Matrix<double,3,4> &proj_mat);

  void AddFeatureObservationsCallback(const std_msgs::Empty &msg);

  void ProcessingAfterOptimization();


private:
  void AddAndRemoveMapPoints();
  void CopyOutMapAfterOptimization();
  void Visualize();

  ros::NodeHandle &node_;
  ros::Subscriber subObservations_;

  confusion::ConFusor &conFusor_;
  confusion::Pose<double> &T_c_i_;
  PointFeatureCalibration calibration_;

  target_tracking::Map map_;
  target_tracking::FeatureTriangulation triObj_;
  target_tracking::RosModule &rosObj_;

  //std::thread observationThread_;
  std::mutex keypointMutex_;


  //TODO Need a reference to the Map

};

} // namespace ftmodule

#endif //FTMODULE_FEATURETRACKINGMODULE_H
