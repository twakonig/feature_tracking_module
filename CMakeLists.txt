cmake_minimum_required(VERSION 2.8.3)
project(feature_tracking_module)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wfatal-errors")

set(PACKAGE_DEPENDENCIES
        confusion
        cv_bridge
        image_transport
        roscpp
        std_msgs
        tf
        target_tracking
        )

find_package(catkin REQUIRED COMPONENTS
        ${PACKAGE_DEPENDENCIES}
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES FeatureMatcher
  CATKIN_DEPENDS confusion cv_bridge image_transport roscpp std_msgs target_tracking
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIR}
)

add_library(feature_tracking_module
  src/FeatureTrackingModule.cpp
  src/FeatureObservation.cpp
  )

add_executable(target_tracker_node src/TargetTrackerNode.cpp src/TargetTracker.cpp)
target_link_libraries(target_tracker_node feature_tracking_module ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})
