//
//  FactorGraph.cpp
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/11/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "FactorGraph.hpp"

#include <gtsam/geometry/Pose3.h>           //camera position
#include <gtsam/geometry/Point3.h>          //landmark coordinate
#include <gtsam/geometry/Point2.h>          //camera observation
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Symbol.h>//using symbols to identify factors

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "BetweenThree.h"

using namespace std;

const vector<string> FactorGraph::keys= {
    "PRIOR_P_X", "PRIOR_P_Y", "PRIOR_P_Z", "PRIOR_P_ROLL", "PRIOR_P_PITCH", "PRIOR_P_YAW",
    "PRIOR_V_X", "PRIOR_V_Y", "PRIOR_V_Z", "PRIOR_V_ROLL", "PRIOR_V_PITCH", "PRIOR_V_YAW",
    "DELTA_P_X", "DELTA_P_Y", "DELTA_P_Z", "DELTA_P_ROLL", "DELTA_P_PITCH", "DELTA_P_YAW",
    "DELTA_V_X", "DELTA_V_Y", "DELTA_V_Z", "DELTA_V_ROLL", "DELTA_V_PITCH", "DELTA_V_YAW",
    "LANDMARK_DEVIATION", "MAX_LANDMARK_DIST", "MAX_ALLOWED_OUTLIER_NOISE"
};
//"CAM_PARAM_FX", "CAM_PARAM_FY", "CAM_PARAM_S", "CAM_PARAM_U", "CAM_PARAM_V"

void FactorGraph::InitializeNoiseModels(){
    gtsam::Vector6 v6p;
	v6p(0,0) = vals[Param::PRIOR_P_X];
	v6p(1,0) = vals[Param::PRIOR_P_Y];
	v6p(2,0) = vals[Param::PRIOR_P_Z];
	v6p(3,0) = vals[Param::PRIOR_P_ROLL];
	v6p(4,0) = vals[Param::PRIOR_P_PITCH];
	v6p(5,0) = vals[Param::PRIOR_P_YAW];

    poseNoise = gtsam::noiseModel::Diagonal::Sigmas(v6p);

    gtsam::Vector6 v6v;
    v6v(0,0) = vals[Param::PRIOR_V_X];
    v6v(1,0) = vals[Param::PRIOR_V_Y];
    v6v(2,0) = vals[Param::PRIOR_V_Z];
    v6v(3,0) = vals[Param::PRIOR_V_ROLL];
    v6v(4,0) = vals[Param::PRIOR_V_PITCH];
    v6v(5,0) = vals[Param::PRIOR_V_YAW];

    velNoise = gtsam::noiseModel::Diagonal::Sigmas(v6v);

    gtsam::Vector6 v6k;
    v6k(0,0) = vals[Param::DELTA_P_X];
    v6k(1,0) = vals[Param::DELTA_P_Y];
    v6k(2,0) = vals[Param::DELTA_P_Z];
    v6k(3,0) = vals[Param::DELTA_P_ROLL];
    v6k(4,0) = vals[Param::DELTA_P_PITCH];
    v6k(5,0) = vals[Param::DELTA_P_YAW];

    kinNoise = gtsam::noiseModel::Diagonal::Sigmas(v6k);

    gtsam::Vector6 v6d;
    v6d(0,0) = vals[Param::DELTA_V_X];
    v6d(1,0) = vals[Param::DELTA_V_Y];
    v6d(2,0) = vals[Param::DELTA_V_Z];
    v6d(3,0) = vals[Param::DELTA_V_ROLL];
    v6d(4,0) = vals[Param::DELTA_V_PITCH];
    v6d(5,0) = vals[Param::DELTA_V_YAW];

    dVNoise = gtsam::noiseModel::Diagonal::Sigmas(v6d);
    SetLandmarkDeviation(vals[Param::LANDMARK_DEVIATION]);
}

void FactorGraph::SetLandmarkDeviation(double dev) {
    double dimensions = 2.0; //there are two dimensions to an image observation
    vals[Param::LANDMARK_DEVIATION] = dev;
    pixelNoise = gtsam::noiseModel::Isotropic::Sigma(dimensions, dev);
//    gtsam::noiseModel::Isotropic::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(1, 1);
//    gtsam::noiseModel::mEstimator::Tukey::shared_ptr robust = gtsam::noiseModel::mEstimator::Tukey::Create(15);
//    pixelNoise = gtsam::noiseModel::Robust::Create(robust, gaussian);
//    gtsam::noiseModel::Isotropic::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(dimensions, dev);
//    pixelNoise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
}

void FactorGraph::AddCamera(int camera_key, gtsam::Pose3 cam_est){
    /*Adds a camera prior factor*/
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(key[(int) var::X], camera_key), cam_est, poseNoise));
}

void FactorGraph::AddVelocity(int camera_key, gtsam::Pose3 vel_est){
    /*Adds a velocity prior factor
     >With velocity in only one direction, the pose may have to be constrained in a special way.
     */
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(key[(int)var::V], camera_key), vel_est, velNoise));
}

void FactorGraph::AddKinematicConstraint(int camera_key, double delta_time){
    /*
     SmoothDeltaPoseFactor - Use only with the boat.
     adds the delta_pose constraint on the boat (x,y,yaw)
     delta_time ensures that the delta_pose and the constant velocity are proportional.
     */
    gtsam::Pose3 p(gtsam::Rot3::RzRyRx(0,0,0), gtsam::Point3(0,0,0));
    graph.add(BetweenThree<gtsam::Pose3>(gtsam::Symbol(key[(int) var::X], camera_key), gtsam::Symbol(key[(int) var::X], camera_key-1), gtsam::Symbol(key[(int) var::V], camera_key-1), delta_time, p, kinNoise));
}

void FactorGraph::AddSmoothVelocityConstraint(int camera_key){
    //adds a smoothness constraint on the velocity factor
    gtsam::Pose3 p(gtsam::Rot3::RzRyRx(0,0,0), gtsam::Point3(0,0,0));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(key[(int)var::V], camera_key), gtsam::Symbol(key[(int) var::V], camera_key-1), p, dVNoise));
}

void FactorGraph::AddOdomFactor(int camera_key, gtsam::Pose3 delta_pose){
    /*Adds an odom factor.
     Meant to be used when odometry measurements are available.
     */
    
    if(camera_key <= 0) {
        printf("FactorGraph::AddOdomFactor() Error: Cannot create a between factor with a non-existent camera. Start using this factor when there are at least two cameras.\n");
        exit(-1);
    }
    
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(key[(int) var::X], camera_key-1), gtsam::Symbol(key[(int) var::X], camera_key), delta_pose, kinNoise));
}

gtsam::Symbol FactorGraph::GetSymbol(int survey, int pnum){
    //std::cout << "calling bfg getsymbol()"<<std::endl;
    return gtsam::Symbol(survey, pnum);
}

void FactorGraph::AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, LandmarkTrack& landmark){
    /*Add the landmark track to the graph.*/    

    //    SmartProjectionPoseFactor<Pose3, Point3, Cal3_S2> sppf(1, -1, false, false, boost::none, HESSIAN, 1e10,20);
    //    SmartProjectionPoseFactor(const double rankTol = 1,
    //                              const double linThreshold = -1, const bool manageDegeneracy = false,
    //                              const bool enableEPI = false, boost::optional<POSE> body_P_sensor = boost::none,
    //                              LinearizationMode linearizeTo = HESSIAN, double landmarkDistanceThreshold = 1e10,
    //                              double dynamicOutlierRejectionThreshold = -1) :
    //LinearizationMode linearizeTo = HESSIAN, double landmarkDistanceThreshold = 1e10, double dynamicOutlierRejectionThreshold = -1
    //landmarkDistanceThreshold - if the landmark is triangulated at a distance larger than that the factor is considered degenerate
    //dynamicOutlierRejectionThreshold - if this is nonnegative the factor will check if the average reprojection error is smaller than this threshold after triangulation,
    //  and the factor is disregarded if the error is large
    int ldist = (int) vals[Param::MAX_LANDMARK_DIST]; //this threshold specifies the distance between the camera and the landmark.
    int onoise = (int) vals[Param::MAX_ALLOWED_OUTLIER_NOISE]; //the threshold specifies at what point factors are discarded due to reprojection error.
    gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> sppf(1, -1, false, false, boost::none, gtsam::HESSIAN, ldist, onoise);
    
    for(int i=0; i<landmark.points.size(); i++) {
        if(landmark.constraint_on[i])
            sppf.add(landmark.points[i], landmark.camera_keys[i], pixelNoise, k);
    }
    
    landmark_factors[active_landmark_set].push_back(sppf);
    landmark_keys[active_landmark_set].push_back(landmark.key);
    if(landmark.used) graph.add(sppf);
}

void FactorGraph::Clear(){
    graph.resize(0);
    next_camera_key = 0;
    landmark_factors.clear();
    landmark_keys.clear();
    ChangeLandmarkSet(0);
}

void FactorGraph::PrintFactorGraph() {
    graph.print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Factor Graph:<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
}

void FactorGraph::ChangeLandmarkSet(int set){
    if(set == landmark_factors.size()){
        landmark_factors.push_back({});
        landmark_keys.push_back({});
    }
    else if(set<0 || set > landmark_factors.size()){
        std::cout << "FactorGraph::ChangeLandmarkSet() Error. Specify an existing set or the next one. " << std::endl;
        std::exit(-1);
    }
    active_landmark_set = set;
}
