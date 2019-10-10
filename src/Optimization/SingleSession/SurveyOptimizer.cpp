//
//  SurveyOptimizer.cpp
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/11/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "SurveyOptimizer.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

#include "FileParsing/ParseSurvey.h"
#include "EvaluateSLAM.h"
#include "RFlowOptimization/LocalizePose.hpp"

using namespace std;

const vector<string> SurveyOptimizer::keys = {
    "OPT_OFFSET", "OPT_SKIP", "CAM_OFFSET", "CAM_SKIP"
};

SurveyOptimizer::SurveyOptimizer(const Camera& cam, FactorGraph * _fg, std::string date, std::string results_dir, bool print_data_increments):
_cam(cam), _results_dir(results_dir), FG(_fg), _date(date), _print_data_increments(print_data_increments) {}

SurveyOptimizer::SurveyOptimizer(const Camera& cam, std::string date, std::string results_dir, bool print_data_increments):
_cam(cam), _results_dir(results_dir), _date(date), _print_data_increments(print_data_increments)
{
    FG = new FactorGraph();
    clean_up = true;
}

void SurveyOptimizer::Initialize(){
    initialized = true;
    
    num_cameras_in_traj = 0;
    GTS = GTSAMInterface(FG);
    LoadParameters();
    
    _incremental = ((int) vals[Param::OPT_SKIP]) == 1;
    std::cout << "incremental? " << _incremental << std::endl;
    GTS.SetupSLAM(_incremental);
    if(debug)  GTS.SetPrintSymbols();
    
    std::cout << "Optimizing " << _date;
    if(_incremental) std::cout << " incrementally " << std::endl;
    else             std::cout << " in batch mode " << std::endl;
}

bool SurveyOptimizer::OptimizeThisIteration(int camera_key){
    //determine whether optimization should be run this iteration.
    return camera_key > vals[Param::OPT_OFFSET] && camera_key % (int) vals[Param::OPT_SKIP] == 0;
}

void SurveyOptimizer::RunGTSAM(bool update_everything){
    /*Running GTSAM consumes a lot of time in the update and in computing the MAP value for the saved variables*/
    if(!initialized){cout << "check initialization"<<endl; exit(1);}
    if(!dry_run) GTS.Update(update_everything);
    if(verbose) FG->PrintFactorGraph();
}

void SurveyOptimizer::SaveResults(SaveOptimizationResults& SOR, int iteration, double percent_completed, vector<double> drawscale){
    if(_print_data_increments || percent_completed == 100){
        vector<vector<double> > ls = GTS.GetOptimizedLandmarks();
        vector<vector<double> > ts = GTS.GetOptimizedTrajectory(FG->key[(int) FactorGraph::var::X], num_cameras_in_traj);
        vector<vector<double> > vs = GTS.GetOptimizedTrajectory(FG->key[(int) FactorGraph::var::V], num_cameras_in_traj);
        SOR.PlotAndSaveCurrentEstimate(ls, ts, vs, drawscale);
    }
    SOR.StatusMessage(iteration, percent_completed);
}

void SurveyOptimizer::LoadParameters(){
    /*Tries to load the parameters from the file. If they don't exist, 
     the defaults are used.*/
	if(!initialized){cout << "check initialization"<<endl; exit(1);}

    ParamsInterface PI;
    PI.LoadParams(_results_dir);
    SetParams(PI.LoadParams(SurveyOptimizer::Keys(), Params()));
    GTS.SetParams(PI.LoadParams(GTSAMInterface::Keys(), GTS.Params()));
    FG->SetParams(PI.LoadParams(FactorGraph::Keys(), FG->Params()));
}

void SurveyOptimizer::SaveParameters(){
    /*Save the params with the result.*/
    ParamsInterface PI;
    PI.AddParams(Keys(), Params());
    PI.AddParams(GTSAMInterface::Keys(), GTS.Params());
    PI.AddParams(FactorGraph::Keys(), FG->Params());
    PI.SaveParams(_results_dir + _date);
}

void SurveyOptimizer::AddPoseConstraints(double delta_time, gtsam::Pose3 btwn_pos, gtsam::Pose3 vel_est, int camera_key, bool transition){
    static bool transition_prevstep = true;
    
    //the island transition. if the pan/tilt camera moves, use a simple odom factor to get around the yaw constraint.
    if(!transition) {
        if(transition_prevstep) {
            GTS.InitializePose(FG->key[(int)FactorGraph::var::V], camera_key-1, vel_est);
            FG->AddVelocity(camera_key-1, vel_est);
            transition_prevstep = false;
        }
        
        GTS.InitializePose(FG->key[(int)FactorGraph::var::V], camera_key, vel_est);
        FG->AddVelocity(camera_key, vel_est);
        
        FG->AddKinematicConstraint(camera_key, delta_time);
        FG->AddSmoothVelocityConstraint(camera_key);
    } else {
        FG->AddOdomFactor(camera_key, btwn_pos);
        transition_prevstep=true;
        if(debug)
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>transition at<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pose " << camera_key << endl;
    }
}

//void SurveyOptimizer::AddCamera(int camera_key, gtsam::Pose3& measured, gtsam::Pose3& localized){
//    num_cameras_in_traj++;
//    GTS.InitializePose(FG->key[(int)FactorGraph::var::X], camera_key, localized);
//    FG->AddCamera(camera_key, measured);
//}

void SurveyOptimizer::AddLandmarkTracks(vector<LandmarkTrack>& landmarks){
    for(int i=0; i<landmarks.size(); i++)
    {
        FG->AddLandmarkTrack(_cam.GetGTSAMCam(), landmarks[i]);
    }
}

void SurveyOptimizer::AddActiveLandmarks(vector<LandmarkTrack>& landmarks)
{
    /*Use this for incremental optimization; the active landmarks are ultimately used to estimate the new location of a pose.*/
    for(int i=0; i<landmarks.size(); i++)
    {
        int len = landmarks[i].Length();
        if(len >= 3)
        {
            int smart_factor_idx = FG->GraphHasLandmark(landmarks[i].key);
            if(smart_factor_idx == -1)
            {
                std::cout << "SurveyOptimizer::AddActiveLandmarks() error. the active landmark should have already been added to the factor graph, but it wasn't found." << std::endl;
                exit(-1);
            }
#ifdef GTSAM4
            //remove the factor from isam2
            GTS.RemoveLandmarkFactor(smart_factor_idx);
#endif
            
            //add the new measurement to the existing factor and add it back to the factor graph, which will be added to isam2
            FG->AddToExistingLandmark(landmarks[i].points[len-1], (int) 'x', landmarks[i].camera_keys[len-1], smart_factor_idx);
        }
        else if(len > 1)
        {
            FG->AddLandmarkTrack(_cam.GetGTSAMCam(), landmarks[i]);
        }
    }
}

void SurveyOptimizer::CacheLandmarks(vector<LandmarkTrack>& inactive){
    for(int i=0; i<inactive.size(); i++)
        cached_landmarks[cache_set].push_back(inactive[i]);
}

boost::optional<gtsam::Pose3> SurveyOptimizer::LocalizeCurPose(gtsam::Pose3& cam)
{
    if(active.size() <= 3)
    {
        return {};
    }
    
    int latest_pose_idx = active[0].camera_keys[active[0].Length()-1].index();
    
    //find the earliest pose that observed the currently active feature tracks.
    int first_pose_for_cur_feature_tracks = latest_pose_idx;
    for(int i=0; i<active.size(); ++i)
    {
        int first_camera_index = active[i].camera_keys[0].index();
        first_pose_for_cur_feature_tracks = std::min(first_pose_for_cur_feature_tracks, first_camera_index);
    }
    
    std::shared_ptr<gtsam::Values> v = GTS.getPoseValuesFrom(FG->key[(int)FactorGraph::var::X], first_pose_for_cur_feature_tracks, latest_pose_idx);
    
    std::vector<gtsam::Point3> p3d;
    std::vector<gtsam::Point2> p2d1;
    for(int i=0; i<active.size(); ++i)
    {
        int smart_factor_idx = FG->GraphHasLandmark(active[i].key);
        if(smart_factor_idx < 0)
            continue;

        boost::optional<gtsam::Point3> triangulated = GTS.triangulatePointFromPoseValues(*v, smart_factor_idx);
        if(triangulated)
        {
            p3d.push_back(triangulated.get());
            p2d1.push_back(active[i].points[active[i].Length()-1]);
        }
    }
    
    //std::cout << "localizing the new pose using: " << p3d.size() << " points " << std::endl;
    
    LocalizePose loc(_cam);
    loc.setRANSACModel(1);
    loc.debug = true;
    std::vector<double> vec = GTSAMInterface::PoseToVector(cam);
    std::vector<double> inliers(p3d.size(), 1.0);
    std::vector<std::vector<double>> res = loc.combinedLocalizationMethod(vec, p3d, p2d1, inliers); //UseBAIterative
    if(res.size() > 0 and (res[1][1] > 0.5 * p3d.size() or res[1][1] > 15))
        return GTSAMInterface::VectorToPose(res[0]);
    return {};
}

int SurveyOptimizer::ConstructGraph(ParseSurvey& PS, ParseFeatureTrackFile& PFT, int cidx, int lcidx, bool gap){
    //get the poses
    gtsam::Pose3 cam = PS.CameraPose(cidx); //the camera pose estimated using sensors.
    static gtsam::Pose3 last_cam = PS.CameraPose(lcidx);
    gtsam::Pose3 curcam = cam;
    
    int camera_key = FG->GetNextCameraKey();
    std::cout << "at iteration with camera x" << camera_key << std::endl;
    
    //check for camera transitions
    bool flipped = PS.CheckCameraTransition(cidx, lcidx);
    bool transition = flipped || gap;
    if(transition){
        if(debug) std::cout << "flip? " << flipped << ", gap? " << gap << ", at " << cidx << std::endl;
    	if(cache_landmarks) CacheLandmarks(active);
    	else if(_incremental) AddActiveLandmarks(active);
        else AddLandmarkTracks(active);
    	active.clear();
    }
    
    //process the landmark measurement
    std::vector<LandmarkTrack> inactive = PFT.ProcessNewPoints((int) 'x', camera_key, active, percent_of_tracks);
    
    //initialize the camera (this is added after processing the landmarks in order to have the active set for localizing the current pose)
    boost::optional<gtsam::Pose3> curpose;
    if(_incremental and camera_key > 1)
    {
        curpose = LocalizeCurPose(cam);
        if(curpose) {
            GTS.InitializePose(FG->key[(int)FactorGraph::var::X], camera_key, curpose.get());
            std::cout << "localization succeeded" << std::endl;
            curcam = curpose.get();
            transition = true;
        }
        else { std::cout << "localization failed" << std::endl;} //transition = true;
    }
    if(not curpose) GTS.InitializePose(FG->key[(int)FactorGraph::var::X], camera_key, cam);
    
    //add the landmark measurement to the graph
    if(cache_landmarks) CacheLandmarks(inactive);
    else if(_incremental) AddActiveLandmarks(active);
    else AddLandmarkTracks(inactive);
    
    ++num_cameras_in_traj;
    FG->AddCamera(camera_key, curcam);
    
    //add the kinematic constraints.
    if(camera_key != 0) {
        gtsam::Pose3 btwn_pos = last_cam.between(curcam);
        
        if(PS.ConstantVelocity()){
            //note: keep constant velocity, but assume the between factor accounts for the time between poses.
            double av = PS.GetAvgAngularVelocity(lcidx, cidx); //Estimate the angular velocity of the boat.
            double delta_t = PS.timings[cidx]-PS.timings[lcidx]; //Get the difference in time.
            std::vector<double> pvec = {av*delta_t, 0.0, 0.0, btwn_pos.x(), btwn_pos.y(), btwn_pos.z()};
            gtsam::Pose3 vel_est = GTSAMInterface::VectorToPose(pvec);
            AddPoseConstraints(delta_t, btwn_pos, vel_est, camera_key, transition);
        } else {
            FG->AddOdomFactor(camera_key, btwn_pos);
        }
    }
    
    last_cam = cam;
    return camera_key;
}

void SurveyOptimizer::Optimize(ParseSurvey& PS) {
    //TODO: implement pose decimation, rather than the 1 by 10 (CAM_SKIP) rule currently used.
	if(!initialized){cout << "SurveyOptimizer::Optimize() check initialization"<<endl; exit(1);}
    
    std::cout << "Constructing the factor graph." << std::endl;
    
    EvaluateSLAM es(_cam, _date, _results_dir);
    es.debug=true;
    
    SaveOptimizationResults SOR(_results_dir + _date);
    SOR.SetSaveStatus();
    SOR.SetDrawMap();
    
    int cidx = 0, lcidx=0;
    for(int i=vals[Param::CAM_OFFSET]; ; i=i+vals[Param::CAM_SKIP]) {
        //Find the AUX file entry that is time-aligned with the visual feature track data. (for the camera pose)
        bool gap = PS.CheckGap(lcidx, lcidx + vals[Param::CAM_SKIP]);
        
        ParseFeatureTrackFile PFT = PS.LoadVisualFeatureTracks(_cam, i, gap);
        cidx = PS.FindSynchronizedAUXIndex(PFT.time, cidx);
        if(cidx==-1) break;
        if(cidx == lcidx) continue; //the feature track file didn't advance anything.
        if(!PS.Useable(cidx, lcidx)) continue; //the camera is being adjusted, don't use the data.
        if(PFT.CheckImageDuplication(active)) continue; //the image was duplicated, so the tracking data is unhelpful, don't use the data.
        
        //Construct the graph (camera, visual landmarks, velocity, and time)
        //int camera_key = ConstructGraph(PS.CameraPose(cidx), PFT, av, PS.timings[cidx]);
        int camera_key = ConstructGraph(PS, PFT, cidx, lcidx, gap);
        
        //Optimize the graph
        if(OptimizeThisIteration(camera_key)){
            SOR.TimeOptimization();
            RunGTSAM(false);
            if(_print_data_increments)
            {
                SaveResults(SOR, camera_key, 100.0*cidx/(PS.timings.size()-1000), PS.GetDrawScale());
                es.ErrorForSurvey(PS._pftbase, true);
            }
        }
        
        //Log the data alignment.
        int imageno = PS.GetImageNumber(cidx);
        SOR.SaveDataCorrespondence(camera_key, i, cidx, imageno, PS.timings[cidx]);
        if(debug)
            cout << "correspondence: " << camera_key <<", " << i << ", "<<cidx<<", "<<imageno<<", "<<(int) (PS.timings[cidx]/100000) << ((int)PS.timings[cidx])%100000 <<endl;
        
        lcidx = cidx;
    }
    
    //Add the remaining landmarks
    AddLandmarkTracks(active);
    active.clear();
    
    std::cout << "Optimizing..." << std::endl;
    
    //Final optimization
    SOR.TimeOptimization();
    RunGTSAM(true);
    SaveResults(SOR, 0, 100.0, PS.GetDrawScale());
    SaveParameters();
    es.ErrorForSurvey(PS._pftbase, true);
    es.PrintTots();
}









