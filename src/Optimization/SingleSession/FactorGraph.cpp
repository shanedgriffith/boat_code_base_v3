//
//  FactorGraph.cpp
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/11/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "FactorGraph.hpp"

#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Symbol.h>//using symbols to identify factors

#include <gtsam/slam/EssentialMatrixConstraint.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "BetweenThree.h"
#include "OrientationConstraint.h"

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
    gtsam::Vector6 v6p = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&vals[Param::PRIOR_P_X]), 6, 1);
    poseNoise = gtsam::noiseModel::Diagonal::Sigmas(v6p);
    
    gtsam::Vector6 v6v = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&vals[Param::PRIOR_V_X]), 6, 1);
    velNoise = gtsam::noiseModel::Diagonal::Sigmas(v6v);

    gtsam::Vector6 v6k = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&vals[Param::DELTA_P_X]), 6, 1);
    kinNoise = gtsam::noiseModel::Diagonal::Sigmas(v6k);
    
    gtsam::Vector6 v6kt = (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished();
    tightKinNoise = gtsam::noiseModel::Diagonal::Sigmas(v6kt);
    
//    gtsam::Vector3 v3orei = (gtsam::Vector(3) << 0.002, 0.002, 0.01).finished();
    gtsam::Vector3 v3orei = (gtsam::Vector(3) << 0.002, 0.002, 0.1).finished();
    orientNoise = gtsam::noiseModel::Diagonal::Sigmas(v3orei);
    
    gtsam::Vector6 v6d = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&vals[Param::DELTA_V_X]), 6, 1);
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

void FactorGraph::AddEssentialMatrix(int camera_key, gtsam::EssentialMatrix& E)
{
    gtsam::Vector5 v5p;
    v5p = (gtsam::Vector(5) << 0.05, 0.05, 0.0017, 0.0017, 0.0017).finished();
    auto noisemodel = gtsam::noiseModel::Diagonal::Sigmas(v5p);
    graph.add(gtsam::EssentialMatrixConstraint(gtsam::Symbol(key[(int) var::X], camera_key-1), gtsam::Symbol(key[(int) var::X], camera_key), E, noisemodel));
}

void FactorGraph::AddCamera(int camera_key, gtsam::Pose3 cam_est){
    /*Adds a camera prior factor*/
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(key[(int) var::X], camera_key), cam_est, poseNoise));
    variables++;
}

void FactorGraph::AddVelocity(int camera_key, gtsam::Pose3 vel_est){
    /*Adds a velocity prior factor
     >With velocity in only one direction, the pose may have to be constrained in a special way.
     */
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(key[(int)var::V], camera_key), vel_est, velNoise));
    variables++;
}

void FactorGraph::AddKinematicConstraint(int camera_key, double delta_time){
    /*
     SmoothDeltaPoseFactor - Use only with the boat.
     adds the delta_pose constraint on the boat (x,y,yaw)
     delta_time ensures that the delta_pose and the constant velocity are proportional.
     */
    gtsam::Pose3 p(gtsam::Rot3::RzRyRx(0,0,0), gtsam::Point3(0,0,0));
    graph.add(BetweenThree<gtsam::Pose3>(gtsam::Symbol(key[(int) var::X], camera_key), gtsam::Symbol(key[(int) var::X], camera_key-1), gtsam::Symbol(key[(int) var::V], camera_key-1), delta_time, p, kinNoise));
    variable_constraints++;
}

void FactorGraph::AddOrientationConstraint(int camera_key1, int camera_key2, const gtsam::Rot3& rot)
{
    graph.add(OrientationConstraint(gtsam::Symbol(key[(int) var::X], camera_key1), gtsam::Symbol(key[(int) var::X], camera_key2), rot, orientNoise));
    variable_constraints++;
}

void FactorGraph::AddSmoothVelocityConstraint(int camera_key){
    //adds a smoothness constraint on the velocity factor
    gtsam::Pose3 p(gtsam::Rot3::RzRyRx(0,0,0), gtsam::Point3(0,0,0));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(key[(int)var::V], camera_key), gtsam::Symbol(key[(int) var::V], camera_key-1), p, dVNoise));
    variable_constraints++;
}

void FactorGraph::AddOdomFactor(int camera_key, gtsam::Pose3 delta_pose, bool tight){
    /*Adds an odom factor.
     Meant to be used when odometry measurements are available.
     */
    
    if(camera_key <= 0) {
        printf("FactorGraph::AddOdomFactor() Error: Cannot create a between factor with a non-existent camera. Start using this factor when there are at least two cameras.\n");
        exit(-1);
    }
    
    if(tight)
    {
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(key[(int) var::X], camera_key-1), gtsam::Symbol(key[(int) var::X], camera_key), delta_pose, tightKinNoise));
    }
    else
    {
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(key[(int) var::X], camera_key-1), gtsam::Symbol(key[(int) var::X], camera_key), delta_pose, kinNoise));
    }
    variable_constraints++;
}

gtsam::Symbol FactorGraph::GetSymbol(int survey, int pnum){
    //std::cout << "calling bfg getsymbol()"<<std::endl;
    return gtsam::Symbol(survey, pnum);
}

void FactorGraph::AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, LandmarkTrack& landmark){
    /*Add the landmark track to the graph.*/
    
    if(landmark.Length() < 2)
    {
        landmark.used = false;
    }
    
    int ldist = (int) vals[Param::MAX_LANDMARK_DIST]; //this threshold specifies the distance between the camera and the landmark.
    int onoise = (int) vals[Param::MAX_ALLOWED_OUTLIER_NOISE]; //the threshold specifies at what point factors are discarded due to reprojection error.

    //landmarkDistanceThreshold - if the landmark is triangulated at a distance larger than that the factor is considered degenerate
    //dynamicOutlierRejectionThreshold - if this is nonnegative the factor will check if the average reprojection error is smaller than this threshold after triangulation,
    //  and the factor is disregarded if the error is large
    gtsam::SmartProjectionParams params;
    params.setLandmarkDistanceThreshold(ldist);
    params.setDynamicOutlierRejectionThreshold(onoise);
    params.setDegeneracyMode(gtsam::DegeneracyMode::ZERO_ON_DEGENERACY); //either this, or non-hessian linearization.
    //params.setLinearizationMode(gtsam::LinearizationMode::JACOBIAN_Q); //either this, or zero_on_degeneracy.
    
    gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> sppf(pixelNoise, k, boost::none, params);

    if(landmark.used)
    {
        for(int i=0; i<landmark.points.size(); i++) {
            landmark_constraints++;
            sppf.add(landmark.points[i], landmark.camera_keys[i]); //GTSAM 4.0
        }
    }
    
    landmark_factors[active_landmark_set].push_back(sppf);
    landmark_keys[active_landmark_set].push_back(landmark.key);
    
    if(landmark.used)
    {
        landmark_to_graph_index[active_landmark_set].push_back(graph.size());
        graph.add(sppf);
        landmarks++;
    }
}

void FactorGraph::AddToExistingLandmark(gtsam::Point2& point, int survey, int camera_key, int smart_factor_idx)
{
    gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>& sppf = landmark_factors[active_landmark_set][smart_factor_idx];
    gtsam::Symbol S((char) survey, camera_key);
    sppf.add(point, S);
    landmark_to_graph_index[active_landmark_set][smart_factor_idx] = graph.size();
    graph.add(sppf);
}

int FactorGraph::GraphHasLandmark(int landmark_key)
{
    int s = 0;
    int e = landmark_keys[active_landmark_set].size();
    while(e - s > 0)
    {
        int med = s + (e-s)/2;
        if(landmark_keys[active_landmark_set][med] < landmark_key) s = med + 1;
        else if(landmark_keys[active_landmark_set][med] > landmark_key) e = med;
        else return med;
    }
    return -1;
}

void FactorGraph::Clear(){
    graph.resize(0);
    next_camera_key = 0;
    landmark_factors.clear();
    landmark_keys.clear();
    landmark_to_graph_index.clear();
    ChangeLandmarkSet(0);
    landmarks = 0;
    landmark_constraints = 0;
    variables = 0;
    variable_constraints = 0;
}

void FactorGraph::PrintFactorGraph() {
    graph.print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Factor Graph:<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
}

void FactorGraph::ChangeLandmarkSet(int set){
    if(set == landmark_factors.size()){
        landmark_factors.push_back({});
        landmark_keys.push_back({});
        landmark_to_graph_index.push_back({});
    }
    else if(set<0 || set > landmark_factors.size()){
        std::cout << "FactorGraph::ChangeLandmarkSet() Error. Specify an existing set or the next one. ("<<set<< ", size: " << landmark_factors.size()<< ")" << std::endl;
        std::exit(-1);
    }
    active_landmark_set = set;
}

void FactorGraph::PrintStats() {
    std::cout << "--Factor Graph Stats--" << std::endl;
    std::cout << " variables: \t\t" << variables << std::endl;
    std::cout << " variable constraints: \t" << variable_constraints << std::endl;
    std::cout << " landmarks: \t\t" << landmarks << std::endl;
    std::cout << " landmark constraints: \t" << landmark_constraints << std::endl;
    std::cout << "----------------------" << std::endl;
}
