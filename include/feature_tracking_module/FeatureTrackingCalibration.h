//
// Created by tim on 25.04.20.
//

#ifndef FTMODULE_FEATURETRACKINGCALIBRATION_H
#define FTMODULE_FEATURETRACKINGCALIBRATION_H

#include <Eigen/Core>

struct PointFeatureCalibration {
  Eigen::Matrix<double, 3, 4> projMat_;
  double w_; // Weighting [pixels]. Inverse stddev.
  bool useLossFunction_ = true;
  double lossCoefficient_ = 5.0;
};


#endif //FTMODULE_FEATURETRACKINGCALIBRATION_H
