//
//  VisualOdometry.hpp
//  boat_code_base
//
//  Created by Shane Griffith on 3/18/17.
//  Copyright © 2017 shane. All rights reserved.
//

#pragma once

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
    void AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, int landmark_key, std::vector<gtsam::Point2>& points, std::vector<gtsam::Symbol> camera_keys, bool used);
    void AddLandmarkTracks(std::vector<LandmarkTrack>& inactive);
    bool OptimizeThisIteration(int camera_key);
    void SaveResults(int iteration = -1, double percent_completed = -1);
    void RunGTSAM();
    ParseFeatureTrackFile LoadVisualFeatureTracks(int& index);
    void AddPoseConstraints(double delta_time, gtsam::Pose3 btwn_pos, gtsam::Pose3 vel_est, int camera_key, bool flipped);
    int AddCamera(gtsam::Pose3 cam);
    bool CheckCameraTransition(gtsam::Pose3 cam, gtsam::Pose3 last_cam);
    void ConstructGraph(gtsam::Pose3 cam, std::shared_ptr<ParseFeatureTrackFile> PFT);
    int FindSynchronizedAUXIndex(std::vector<double>& timings, double pft_time, int from_idx);
    void Reset();
    gtsam::Values RunBA();
    gtsam::Pose3 toPose3(cv::Mat R, cv::Mat t);
    gtsam::Pose3 toPose3(cv::Mat P);
    cv::Mat toMat(gtsam::Pose3 p);
    
    void printMat(cv::Mat R, cv::Mat t);
    void PrintVec(std::vector<double> p);
    void checkRes(cv::Mat R, cv::Mat t, std::vector<cv::Point2f> p0, std::vector<cv::Point2f> p1, std::vector<cv::Point2f> p2d1);

    void AddPose(gtsam::Symbol symb, gtsam::Pose3 pguess);
    void AddOdom(gtsam::Symbol symb0, gtsam::Pose3 pguess0, gtsam::Symbol symb1, gtsam::Pose3 pguess1);
    std::pair<double, int> getReprojectionError(cv::Mat& W, std::vector<cv::Point2f>& p2d1, cv::Mat P1);
    cv::Mat triangulatePointSet(std::vector<cv::Point2f>& p0, std::vector<cv::Point2f>& p1, cv::Mat P1);
    
    int triangulateAndCountInFrontOfCamera(gtsam::Pose3 guess, std::vector<cv::Point2f>& p0, std::vector<cv::Point2f>& p1);
    gtsam::Pose3 recoverPose(cv::Mat E, std::vector<cv::Point2f> p0, std::vector<cv::Point2f> p1);
    
    gtsam::Pose3 PnP(gtsam::Values& result, gtsam::Pose3 est, std::shared_ptr<ParseFeatureTrackFile> latest);
    std::pair<std::vector<cv::Point2f>, std::vector<int>> findOverlappingPointSet(std::vector<gtsam::Point2>& ic1, std::vector<int>& id1, std::vector<int>& ids, bool keep_unmatched = false);
    
    int num_cameras_in_traj = 0;
    
    std::shared_ptr<ParseFeatureTrackFile> lastPFT;
    void CopyLast(std::shared_ptr<ParseFeatureTrackFile> PFT);

    gtsam::NonlinearFactorGraph graph;
#ifdef GTSAM4
    std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> > landmark_factors; //GTSAM 4.0
#else
    std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> > landmark_factors; //GTSAM 3.2.1
#endif
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
    const Camera& _cam;
public:
    std::string _date;
    
    VisualOdometry(const Camera& cam):
    _cam(cam) {
        double deviation = 3.0;
        double dimensions = 2.0;
        pixelNoise = gtsam::noiseModel::Isotropic::Sigma(dimensions, deviation);
    }
    
    void SetDryRun(){dry_run = true;}
    void SetVerbose(){verbose = true;}
    
    //align the image tracking dataset with the interpolated data in the csv file.
    //this method assumes the image_auxiliary file starts before the csv sift file.
    void SetPrior(gtsam::Pose3 p){_prior = p;}
    gtsam::Pose3 GetNextOdom(std::shared_ptr<ParseFeatureTrackFile> PFT);
    std::pair<gtsam::Pose3, std::pair<double, int> > PoseFromEssential(std::shared_ptr<ParseFeatureTrackFile> last, std::shared_ptr<ParseFeatureTrackFile> latest);
    std::pair<double, int> KeypointChange(std::shared_ptr<ParseFeatureTrackFile> last, std::shared_ptr<ParseFeatureTrackFile> latest);
    int testAndVerifyVO(std::vector<std::shared_ptr<ParseFeatureTrackFile> > t);
};
