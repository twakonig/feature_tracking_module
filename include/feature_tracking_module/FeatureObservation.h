
#ifndef FTMODULE_FEATUREOBSERVATION_H_
#define FTMODULE_FEATUREOBSERVATION_H_

#include <Eigen/Core>
#include <array>

#include <confusion/UpdateMeasurement.h>
#include <confusion/utilities/Pose.h>
#include <confusion/utilities/ceres_utils.h>
#include <cv_bridge/cv_bridge.h>

#include "feature_tracking_module/MapPoint.h"
#include "feature_tracking_module/FeatureObservationCost.h"

namespace ftmodule {

class FeatureObservation : public confusion::UpdateMeasurement {
  friend class FeatureObservationCost;

 public:
  FeatureObservation(double t,
                   MapPoint &mapPoint,
                   const cv::KeyPoint &detectedKeyPoint,
                   confusion::Pose<double> &T_c_i,
                   const PointFeatureCalibration &pointFeatureCalibration,
                   int measurementType, //todo No default value for this for now to make sure I'm not creating any without specifying Tracking/Smoothing
                   int startingStateParamIndex = 0) :
      UpdateMeasurement(measurementType, t, "Pf", true), //todo Temp Check if the map point is valid before solving each time
      mapPoint_(mapPoint), detectedKeyPoint_(detectedKeyPoint),
      pointFeatureCalibration_(pointFeatureCalibration), T_c_i_(T_c_i),
      startingStateParamIndex_(startingStateParamIndex) { }

  bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                          std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                          std::vector<size_t> &stateParameterIndexVector,
                          std::vector<double *> &staticParameterDataVector) override {
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

  int residualDimension() override { return 2; }
  const confusion::Pose<double>& T_c_i() const { return T_c_i_; }
  const cv::KeyPoint &keyPoint() const { return detectedKeyPoint_; }
  const cv::Point2f &featureLocationForVisualization() const { return detectedKeyPoint_.pt; }
  const double &getPreviousResidualNorm() { return error_; }

  MapPoint &mapPoint_;

protected:
  const cv::KeyPoint &detectedKeyPoint_;
  const PointFeatureCalibration &pointFeatureCalibration_;

  confusion::Pose<double> &T_c_i_;
  const int startingStateParamIndex_; ///< Indicates the index of the state parameters which contains the position of the measured body pose. It is assumed that the orientation is then the next parameter.

  long unsigned int featureId_;

  mutable double error_; ///< Used for logging and analysis only
};

} // namespace ftmodule

#endif /* FTMODULE_FEATUREOBSERVATION_H_ */
