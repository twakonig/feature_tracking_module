//
// Created by tim on 10.10.18.
//

#include "feature_tracking_module/TargetTracker.h"


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "TargetTracker");
  ros::NodeHandle nh;

  TargetTracker targetTracker(nh);

  ros::spin();

  return 0;
}
