#pragma once


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
#include <FileParsing/ParseOptimizationResults.h>

#include "GTSAMInterface.h"
#include "FactorGraph.hpp"

class SurveyOptimizer {
protected:
	bool initialized = false;
    bool debug = false;
    bool dry_run = false;
    bool verbose = false;
    bool _print_data_increments = false;
    bool cache_landmarks = false;
    bool _incremental = false;
    
    static const std::vector<std::string> keys;
    enum Param { OPT_OFFSET, OPT_SKIP, OPT_STOP, CAM_OFFSET, CAM_SKIP };
    std::vector<double> vals = {  0, 100, 10000, 0, 5 }; //defaults
    
    void AddLandmarkTracks(std::vector<LandmarkTrack>& inactive);
    void AddActiveLandmarks(std::vector<LandmarkTrack>& landmarks);
    bool OptimizeThisIteration(int camera_key);
    void SaveResults(SaveOptimizationResults& SOR, int iteration = -1, double percent_completed = -1, std::vector<double> drawscale={});
    void RunGTSAM(bool update_everything);
    ParseFeatureTrackFile LoadVisualFeatureTracks(int& index);
    void AddPoseConstraints(double delta_time, gtsam::Pose3 btwn_pos, gtsam::Pose3 vel_est, int camera_key, bool transition);
    std::vector<gtsam::Pose3> LocalizeCurPose2D(int cur_pose_idx);
    std::vector<gtsam::Pose3> LocalizeCurPose(int cur_pose_idx);
    void AddCamera(int camera_key, gtsam::Pose3& cam, gtsam::Pose3& localized);
    int ConstructGraph(std::shared_ptr<ParseSurvey> PS, ParseFeatureTrackFile& PFT, int cidx, int lcidx, bool gap);
    
    void testE(const gtsam::Pose3& localized_pose, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d0, std::vector<gtsam::Point2>& p2d1);
    
    int cache_set=0;
    std::vector<std::vector<LandmarkTrack> > cached_landmarks; //landmarks are cached to retroactively add inter-survey constraints.
    void CacheLandmarks(std::vector<LandmarkTrack>& inactive);
    
    int num_cameras_in_traj=0;
    std::vector<LandmarkTrack> active;
    GTSAMInterface GTS;
    const Camera& cam_;
    FactorGraph * FG;
    bool clean_up = false;
    
public:
    double percent_of_tracks = 100.0;
    std::string _date;
    std::string _results_dir;
    
    SurveyOptimizer(const Camera& cam, FactorGraph * _fg, std::string date, std::string results_dir, bool print_data_increments=false);
    
    SurveyOptimizer(const Camera& cam, std::string date, std::string results_dir, bool print_data_increments=false);
    
    ~SurveyOptimizer() {
    	if(clean_up) delete(FG);
    }
    
    void Initialize();
    void SetDryRun(){dry_run = true;}
    void SetVerbose(){verbose = true;}
    
    //align the image tracking dataset with the interpolated data in the csv file.
    //this method assumes the image_auxiliary file starts before the csv sift file.
    void Optimize(std::shared_ptr<ParseSurvey> PS);
    
    std::vector<double> Params(){return vals;}
    static std::vector<std::string> Keys(){return keys;}
    void SetParams(std::vector<double> params){vals = params;}
    void SaveParameters();
    void LoadParameters();
    void SetPercentOfTracks(double p){percent_of_tracks = p;}
};

