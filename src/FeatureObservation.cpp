//
// Created by tim on 25.04.20.
//

#include "feature_tracking_module/FeatureObservation.h"
#include "feature_tracking_module/FeatureObservationCost.h"

namespace ftmodule {

FeatureObservation::FeatureObservation(double t,
                   MapPoint &mapPoint,
                   const cv::KeyPoint &detectedKeyPoint,
                   confusion::Pose<double> &T_c_i,
                   const PointFeatureCalibration &pointFeatureCalibration,
                   int measurementType,
                   int startingStateParamIndex) :
    UpdateMeasurement(measurementType, t, "featureobs", false),
    mapPoint_(mapPoint), detectedKeyPoint_(detectedKeyPoint),
    pointFeatureCalibration_(pointFeatureCalibration), T_c_i_(T_c_i),
    startingStateParamIndex_(startingStateParamIndex) { }

FeatureObservation::~FeatureObservation() { }

bool FeatureObservation::createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                        std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                        std::vector<size_t> &stateParameterIndexVector,
                        std::vector<double *> &staticParameterDataVector) {
  std::unique_ptr<ceres::CostFunction> costFunctionPtr_(
      new ceres::AutoDiffCostFunction<FeatureObservationCost, 2, 3, 4, 3, 4, 3>(
          new FeatureObservationCost(this)));

  stateParameterIndexVector.push_back(startingStateParamIndex_);
  stateParameterIndexVector.push_back(startingStateParamIndex_ + 1);

  staticParameterDataVector.push_back(T_c_i_.trans.data());
  staticParameterDataVector.push_back(T_c_i_.rot.coeffs().data());
  staticParameterDataVector.push_back(mapPoint_.GetPositionDataPointer());

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

} // namespace ftmodule