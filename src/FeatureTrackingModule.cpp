//
// Created by tim on 25.04.20.
//

#include "feature_tracking_module/FeatureTrackingModule.h"

namespace ftmodule {

//______________________________________________________________________________________________________________________
//constructor, initialization
FeatureTrackingModule::FeatureTrackingModule(ros::NodeHandle &nh, confusion::ConFusor &conFusor,
        confusion::Pose<double> &T_c_i, boost::property_tree::ptree& pt, target_tracking::RosModule &rosObj)
            :node_(nh), conFusor_(conFusor), T_c_i_(T_c_i), rosObj_(rosObj)
{

  calibration_.w_ = 1.0 / pt.get<double>("ftmodule.feature_observation_stddev");
  std::cout << "Address of featureTriangulation in FeatureTrackingModule constructor: " << &(rosObj_.featureTriangulation_) << std::endl;

  subObservations_ = node_.subscribe("/new_observations", 2, &FeatureTrackingModule::AddFeatureObservationsCallback, this);

/*
     //OR: call "startFunction" which then starts a thread (AddFeatureObservation) where things are being calculated
    observationThread_ = std::thread(&FeatureTrackingModule::AddFeatureObservationsCallback, this);
    observationThread_.detach();
*/

}


//______________________________________________________________________________________________________________________


//When is this function used?
void FeatureTrackingModule::SetCameraCalibration(const Eigen::Matrix<double,3,4> &proj_mat) {
  calibration_.projMat_ = proj_mat;
}


//______________________________________________________________________________________________________________________



void FeatureTrackingModule::AddFeatureObservationsCallback(const std_msgs::Empty &msg) {

    std::cout << "FEATURE OBSERVATION CALLBACK WAS CALLED" << std::endl;
    std::cout << "Address of featureTriangulation in Callback function: " << &(rosObj_.featureTriangulation_) << std::endl;

    //works
    rosObj_.featureTriangulation_.test_ += 1;

    /*
    double t = 0.0;
    MapPoint map_point(0, Eigen::Vector3d::Zero());
    cv::Point2f detected_keypoint;

    //feed pointer of type FeatureObservation into confusor; (initialization of FeatureObservation object)
    conFusor_.addUpdateMeasurement(std::make_shared<FeatureObservation>(
            t, map_point, detected_keypoint, T_c_i_, calibration_, FEATUREOBS));
    */



    //time of matching
    double t = rosObj_.featureTriangulation_.matchedObservations_.t_sec;

    //2D coordinates of matched points
    std::vector<cv::Point2f> detected_keyPts = rosObj_.featureTriangulation_.matchedObservations_.coord_2d;

    //gives IDs of points -> that is (position in array + 1)
    std::vector<int> indices_map = rosObj_.featureTriangulation_.matchedObservations_.id_list;

    for (int i = 0; i < indices_map.size(); i++) {
        int s = (indices_map[i]-1);
        conFusor_.addUpdateMeasurement(std::make_shared<FeatureObservation>(
                t, rosObj_.featureTriangulation_.map_.registeredPoints_[s].landmark, detected_keyPts[i], T_c_i_, calibration_, FEATUREOBS));
        std::cout << "ADDING UPDATE MEASUREMENT WORKED" << std::endl;
    }



    //USE address of 3D coordinates directly or do that with the copying out?

    //NEED THE RIGHT ADDRESS OF 3D COORDINATES. map_.registeredPoints[i].landmark
    /*
    std::vector<target_tracking::MapPoint> map_points;
    //need to query right positions in map array
    for (int i = 0; i < indices_map.size(); i++) {
        int s = (indices_map[i]-1);
        //3d coordinates they were matched to
        map_points.push_back(rosObj_.featureTriangulation_.map_.registeredPoints_[s].landmark);
    }
     */



}



//______________________________________________________________________________________________________________________



//called in estimator thread: runEstimator
void FeatureTrackingModule::ProcessingAfterOptimization() {
  AddAndRemoveMapPoints();
  CopyOutMapAfterOptimization();
  Visualize();
}



void FeatureTrackingModule::AddAndRemoveMapPoints() {
    //Depends on criteria we want to use, e.g. number of times a point has been viewed
    //if too many points in optimization it will be slow -> check how often point is viewed
}



void FeatureTrackingModule::CopyOutMapAfterOptimization() {
    //locks variables in scope while copying out -> automatically releases mutex

    //std::lock_guard<std::mutex> lg(keypointMutex_);
    //I thought: copy optimized value to address of 3D coordinates in map
    //in callback would not need address then?



}



void FeatureTrackingModule::Visualize() {
  //TODO tfs, point clouds, etc (?)
}



} // namespace ftmodule