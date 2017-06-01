//
//  SurveyOptimizer.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/11/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __BundleAdjustOneDataset__SurveyOptimizer__
#define __BundleAdjustOneDataset__SurveyOptimizer__

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/SimpleCamera.h>    //calibration and performs projections

#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/ParamsInterface.h>
#include <DataTypes/LandmarkTrack.h>
#include <DataTypes/Camera.hpp>
#include <FileParsing/SaveOptimizationResults.h>
#include <FileParsing/ParseSurvey.h>

#include "GTSamInterface.h"
#include "FactorGraph.hpp"

class SurveyOptimizer {
protected:
	bool initialized = false;
    bool debug = false;
    bool dry_run = false;
    bool verbose = false;
    bool _print_data_increments = false;
    bool cache_landmarks = false;
    int num_landmarks_skipped = 0;
    
    static const std::vector<std::string> keys;
    enum Param { OPT_OFFSET, OPT_SKIP, CAM_OFFSET, CAM_SKIP };
    std::vector<double> vals = {  0, 100, 0, 5 }; //defaults
    
    void RemoveLandmarkFromList(int list_idx);
    void AddLandmarkTracks(std::vector<LandmarkTrack> inactive, int survey=-1);
    std::vector<LandmarkTrack> ProcessNewPoints(int ckey, ParseFeatureTrackFile& pft);
    bool OptimizeThisIteration(int camera_key);
    void SaveResults(int iteration = -1, double percent_completed = -1, vector<double> drawscale);
    void RunGTSAM();
    ParseFeatureTrackFile LoadVisualFeatureTracks(int& index);
    void AddPoseConstraints(double delta_time, gtsam::Pose3 btwn_pos, gtsam::Pose3 vel_est, int camera_key, bool flipped);
    int AddCamera(gtsam::Pose3 cam);
    int ConstructGraph(ParseSurvey& PS, ParseFeatureTrackFile& PFT, int cidx, int lcidx);
    std::vector<LandmarkTrack> DEBUGMOD(std::vector<LandmarkTrack>& inactive);
    
    int cache_set=0;
    std::vector<std::vector<LandmarkTrack> > cached_landmarks;//landmarks are cached to retroactively add inter-survey constraints.
    void CacheLandmarks(std::vector<LandmarkTrack>& inactive);
    
    int num_cameras_in_traj=0;
    std::vector<LandmarkTrack> active;
    GTSamInterface GTS;
    Camera& _cam;
    SaveOptimizationResults SOR;
    FactorGraph * FG;
    bool clean_up = false;
public:
    double percent_of_tracks = 100.0;
    std::string _date;
    std::string _results_dir;
    
    SurveyOptimizer(Camera& cam, FactorGraph * _fg, std::string date, std::string results_dir, bool print_data_increments=false):
    _cam(cam), _results_dir(results_dir), FG(_fg), _date(date), SOR(results_dir + date), _print_data_increments(print_data_increments){}
    
    SurveyOptimizer(Camera& cam, std::string date, std::string results_dir, bool print_data_increments=false):
    	_cam(cam), _results_dir(results_dir), _date(date), SOR(results_dir + date), _print_data_increments(print_data_increments){
    	FG = new FactorGraph();
    	clean_up = true;
    }
    
    ~SurveyOptimizer() {
    	if(clean_up) delete(FG);
    }
    
    void Initialize();
    void SetDryRun(){dry_run = true;}
    void SetVerbose(){verbose = true;}
    
    //align the image tracking dataset with the interpolated data in the csv file.
    //this method assumes the image_auxiliary file starts before the csv sift file.
    void Optimize(ParseSurvey& PS);
    
    std::vector<double> Params(){return vals;}
    static std::vector<std::string> Keys(){return keys;}
    void SetParams(std::vector<double> params){vals = params;}
    void SaveParameters(std::string dir_of_param_file);
    void SetPercentOfTracks(double p){percent_of_tracks = p;}
};



#endif /* defined(__BundleAdjustOneDataset__SurveyOptimizer__) */
