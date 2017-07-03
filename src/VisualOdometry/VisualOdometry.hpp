//
//  VisualOdometry.hpp
//  boat_code_base
//
//  Created by Shane Griffith on 3/18/17.
//  Copyright © 2017 shane. All rights reserved.
//

#ifndef SRC_VISUALODOMETRY_VISUALODOMETRY_HPP_
#define SRC_VISUALODOMETRY_VISUALODOMETRY_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>


#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/SimpleCamera.h>    //calibration and performs projections
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <FileParsing/ParseFeatureTrackFile.h>
#include <DataTypes/LandmarkTrack.h>
#include <DataTypes/Camera.hpp>
#include <FileParsing/SaveOptimizationResults.h>

class VisualOdometry {
protected:
    int N_PAST_SETS = 10;
    bool initialized = false;
    bool debug = false;
    bool dry_run = false;
    bool verbose = false;
    bool cache_landmarks = false;
    
    gtsam::noiseModel::Isotropic::shared_ptr pixelNoise;
    int active_landmark_set = 0;
    
    double AngleDistance(double a, double b);
    void RemoveLandmarkFromList(int list_idx);
    void AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, int landmark_key, std::vector<gtsam::Point2>& points, std::vector<gtsam::Symbol> camera_keys, bool used);
    void AddLandmarkTracks(std::vector<LandmarkTrack>& inactive);
    std::vector<LandmarkTrack> ProcessNewPoints(int survey, int ckey, ParseFeatureTrackFile& pft);
    bool OptimizeThisIteration(int camera_key);
    void SaveResults(int iteration = -1, double percent_completed = -1);
    void RunGTSAM();
    ParseFeatureTrackFile LoadVisualFeatureTracks(int& index);
    void AddPoseConstraints(double delta_time, gtsam::Pose3 btwn_pos, gtsam::Pose3 vel_est, int camera_key, bool flipped);
    int AddCamera(gtsam::Pose3 cam);
    bool CheckCameraTransition(gtsam::Pose3 cam, gtsam::Pose3 last_cam);
    void ConstructGraph(gtsam::Pose3 cam, ParseFeatureTrackFile& PFT);
    int FindSynchronizedAUXIndex(std::vector<double>& timings, double pft_time, int from_idx);
    void Reset();
    gtsam::Values RunBA();
    
    void PrintVec(std::vector<double> p);
    std::vector<double> PoseToVector(gtsam::Pose3& cam);

    void AddPose(gtsam::Symbol symb, gtsam::Pose3 pguess);
    void AddOdom(gtsam::Symbol symb0, gtsam::Pose3 pguess0, gtsam::Symbol symb1, gtsam::Pose3 pguess1);
    
    gtsam::Pose3 PnP(gtsam::Values& result, gtsam::Pose3 est, ParseFeatureTrackFile& latest);
    
    int num_cameras_in_traj = 0;
    
    ParseFeatureTrackFile lastPFT;
    void CopyLast(ParseFeatureTrackFile& PFT);

    gtsam::NonlinearFactorGraph graph;
    std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> > landmark_factors;
    std::vector<int> landmark_keys;
    std::vector<gtsam::Pose3> poses;
    int posenum = 0;
    std::vector<LandmarkTrack> active;
    std::vector<std::vector<LandmarkTrack> > landmark_sets;
    
    gtsam::Values initEst;
    gtsam::Pose3 _prior;
    gtsam::Pose3 last_odom;
    gtsam::Values result;
    bool clean_up = false;
    Camera& _cam;
public:
    std::string _date;
    
    VisualOdometry(Camera& cam):
    _cam(cam), lastPFT(cam){
        double deviation = 3.0;
        double dimensions = 2.0;
        pixelNoise = gtsam::noiseModel::Isotropic::Sigma(dimensions, deviation);
    }
    
    void SetDryRun(){dry_run = true;}
    void SetVerbose(){verbose = true;}
    
    //align the image tracking dataset with the interpolated data in the csv file.
    //this method assumes the image_auxiliary file starts before the csv sift file.
    void SetPrior(gtsam::Pose3 p){_prior = p;}
    gtsam::Pose3 GetNextOdom(ParseFeatureTrackFile& PFT);
    gtsam::Pose3 PoseFromEssential(ParseFeatureTrackFile& last, ParseFeatureTrackFile& latest);
};


#endif /* SRC_VISUALODOMETRY_VISUALODOMETRY_HPP_ */
