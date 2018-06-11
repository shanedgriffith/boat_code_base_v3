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

using namespace std;

const vector<string> SurveyOptimizer::keys = {
    "OPT_OFFSET", "OPT_SKIP", "CAM_OFFSET", "CAM_SKIP"
};

void SurveyOptimizer::Initialize(){
    initialized = true;
    
    num_cameras_in_traj = 0;
    GTS = GTSamInterface(FG);
    LoadParameters();
    
    GTS.SetupIncrementalSLAM();
    if(debug)  GTS.SetPrintSymbols();
    
    std::cout << "Optimizing " << _date << std::endl;
}

bool SurveyOptimizer::OptimizeThisIteration(int camera_key){
    //determine whether optimization should be run this iteration.
    return camera_key > vals[Param::OPT_OFFSET] && camera_key % (int) vals[Param::OPT_SKIP] == 0;
}

void SurveyOptimizer::RunGTSAM(){
    /*Running GTSAM consumes a lot of time in the update and in computing the MAP value for the saved variables*/
    if(!initialized){cout << "check initialization"<<endl; exit(1);}
    if(!dry_run) GTS.RunBundleAdjustment(); //GTS.Update();
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
    GTS.SetParams(PI.LoadParams(GTSamInterface::Keys(), GTS.Params()));
    FG->SetParams(PI.LoadParams(FactorGraph::Keys(), FG->Params()));
}

void SurveyOptimizer::SaveParameters(){
    /*Save the params with the result.*/
    ParamsInterface PI;
    PI.AddParams(Keys(), Params());
    PI.AddParams(GTSamInterface::Keys(), GTS.Params());
    PI.AddParams(FactorGraph::Keys(), FG->Params());
    PI.SaveParams(_results_dir + _date);
}

void SurveyOptimizer::AddPoseConstraints(double delta_time, gtsam::Pose3 btwn_pos, gtsam::Pose3 vel_est, int camera_key, bool transition){
    static bool transition_prevstep = true;
    
    //the island transition. if the pan/tilt camera moves, use a simple odom factor to get around the yaw constraint.
    if(!transition) {
        if(transition_prevstep) {
            GTS.InitializeValue(FG->key[(int)FactorGraph::var::V], camera_key-1, (gtsam::Value *) &vel_est);
            FG->AddVelocity(camera_key-1, vel_est);
            transition_prevstep = false;
        }
        
        GTS.InitializeValue(FG->key[(int)FactorGraph::var::V], camera_key, (gtsam::Value *) &vel_est);
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

int SurveyOptimizer::AddCamera(gtsam::Pose3 cam){
    num_cameras_in_traj++;
    int camera_key = FG->GetNextCameraKey();
    GTS.InitializeValue(FG->key[(int)FactorGraph::var::X], camera_key, (gtsam::Value *) &cam);
    FG->AddCamera(camera_key, cam);
    return camera_key;
}

void SurveyOptimizer::AddLandmarkTracks(vector<LandmarkTrack>& landmarks){
    for(int i=0; i<landmarks.size(); i++)
        FG->AddLandmarkTrack(_cam.GetGTSAMCam(), landmarks[i]);
}

void SurveyOptimizer::CacheLandmarks(vector<LandmarkTrack>& inactive){
    for(int i=0; i<inactive.size(); i++)
        cached_landmarks[cache_set].push_back(inactive[i]);
}

int SurveyOptimizer::ConstructGraph(ParseSurvey& PS, ParseFeatureTrackFile& PFT, int cidx, int lcidx, bool gap){
    //get the poses
    gtsam::Pose3 cam = PS.CameraPose(cidx);
    gtsam::Pose3 last_cam = PS.CameraPose(lcidx);
    
    //add the camera
    int camera_key = AddCamera(cam);
    
    //check for camera transitions
    bool flipped = PS.CheckCameraTransition(cidx, lcidx);
    bool transition = flipped || gap;
    if(transition){
        if(debug) std::cout << "flip? " << flipped << ", gap? " << gap << ", at " << cidx << std::endl;
    	if(cache_landmarks) CacheLandmarks(active);
    	else AddLandmarkTracks(active);
    	active.clear();
    }
    
    //process the landmark measurement and add it to the graph
    vector<LandmarkTrack> inactive = PFT.ProcessNewPoints((int) 'x', camera_key, active, percent_of_tracks);
    if(cache_landmarks) CacheLandmarks(inactive);
    else AddLandmarkTracks(inactive);
    
    //add the kinematic constraints.
    if(camera_key != 0) {
        gtsam::Pose3 btwn_pos = last_cam.between(cam);
        
        if(PS.ConstantVelocity()){
            //note: keep constant velocity, but assume the between factor accounts for the time between poses.
            double av = PS.GetAvgAngularVelocity(lcidx, cidx); //Estimate the angular velocity of the boat.
            double delta_t = PS.timings[cidx]-PS.timings[lcidx]; //Get the difference in time.
            gtsam::Pose3 vel_est = gtsam::Pose3(gtsam::Rot3::ypr(av*delta_t, 0, 0), gtsam::Point3(btwn_pos.x(), btwn_pos.y(), btwn_pos.z()));
            AddPoseConstraints(delta_t, btwn_pos, vel_est, camera_key, transition);
        } else {
            FG->AddOdomFactor(camera_key, btwn_pos);
        }
    }
    
    return camera_key;
}

void SurveyOptimizer::Optimize(ParseSurvey& PS){
    //TODO: implement pose decimation, rather than the 1 by 10 (CAM_SKIP) rule currently used.
	if(!initialized){cout << "SurveyOptimizer::Optimize() check initialization"<<endl; exit(1);}
    
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
            RunGTSAM();
            SaveResults(SOR, camera_key, 100.0*cidx/(PS.timings.size()-1000), PS.GetDrawScale());
            es.ErrorForSurvey(PS._pftbase, true);
        }
        
        //Log the data alignment.
        int imageno = PS.GetImageNumber(cidx);
        SOR.SaveDataCorrespondence(camera_key, i, cidx, imageno, PS.timings[cidx]);
        if(debug)
            cout << "correspondence: " << camera_key <<", " << i << ", "<<cidx<<", "<<imageno<<", "<<PS.timings[cidx]<<endl;
        
        lcidx = cidx;
    }
    
    //Add the remaining landmarks
    AddLandmarkTracks(active);
    active.clear();
    
    //Final optimization
    SOR.TimeOptimization();
    RunGTSAM();
    SaveResults(SOR, 0, 100.0, PS.GetDrawScale());
    SaveParameters();
    es.ErrorForSurvey(PS._pftbase, true);
    es.PrintTots();
}









