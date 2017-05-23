#ifndef __BundleAdjustOneDataset__FactorGraph__
#define __BundleAdjustOneDataset__FactorGraph__

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <DataTypes/Camera.hpp>

#define GPS_NOISE 10 //10 meters deviation from actual position.
#define COMPASS_NOISE 0.1745 //10 degrees deviation from actual yaw angle. (noise for the measurements of change in yaw)
#define UNBOUND_ANGLE 1.0
#define UNBOUND_POS 10

class FactorGraph{
protected:
    static const std::vector<std::string> keys;
    bool deferred_add_landmarks = false; //defers until the next update.
    
    enum Param {PRIOR_P_X, PRIOR_P_Y, PRIOR_P_Z, PRIOR_P_ROLL, PRIOR_P_PITCH, PRIOR_P_YAW,
        PRIOR_V_X, PRIOR_V_Y, PRIOR_V_Z, PRIOR_V_ROLL, PRIOR_V_PITCH, PRIOR_V_YAW,
        DELTA_P_X, DELTA_P_Y, DELTA_P_Z, DELTA_P_ROLL, DELTA_P_PITCH, DELTA_P_YAW,
        DELTA_V_X, DELTA_V_Y, DELTA_V_Z, DELTA_V_ROLL, DELTA_V_PITCH, DELTA_V_YAW,
        LANDMARK_DEVIATION, MAX_LANDMARK_DIST, MAX_ALLOWED_OUTLIER_NOISE
    };
    //CAM_PARAM_FX, CAM_PARAM_FY, CAM_PARAM_S, CAM_PARAM_U, CAM_PARAM_V
    
    //defaults
    std::vector<double> vals = {
        GPS_NOISE, GPS_NOISE, 0.03, 0.05, 0.05, COMPASS_NOISE,
        0.15, 0.15, 0.05, UNBOUND_ANGLE, UNBOUND_ANGLE, 0.015,
        0.05, 0.05, UNBOUND_POS, UNBOUND_ANGLE, UNBOUND_ANGLE, 0.25,
        0.4, 0.4, 0.04, 0.025, 0.025, 0.02,
        1, 100, 100
    };
    
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr velNoise;
    gtsam::noiseModel::Diagonal::shared_ptr kinNoise;
    gtsam::noiseModel::Diagonal::shared_ptr dVNoise;
    gtsam::noiseModel::Isotropic::shared_ptr pixelNoise;

    void CacheLandmark(int landmark_key, gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> sppf);

    //true if the survey landmark tracks haven't been filtered out yet. sppf is set to filter bad points.
    bool sppf_prune = true;
public:
    enum class var {L, V, X};
    char key[3]={'l', 'v', 'x'};
    
    gtsam::NonlinearFactorGraph graph;
    std::vector<std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> > > landmark_factors; //used by the GTSamInterface
    std::vector<std::vector<int> > landmark_keys; //used by the GTSamInterface
    
    FactorGraph(){
        ChangeLandmarkSet(0);
        InitializeNoiseModels();
    }
    
    virtual ~FactorGraph(){}

    bool hold_constant = false;
    void HoldConstant(bool h){hold_constant=h;}
    
    int active_landmark_set = 0;
    void ChangeLandmarkSet(int set);
    
    void InitializeNoiseModels();

    void AddCamera(int camera_key, gtsam::Pose3 cam_est);
    void AddVelocity(int camera_key, gtsam::Pose3 vel_est);
    void AddOdomFactor(int camera_key, gtsam::Pose3 delta_pose);
    void AddKinematicConstraint(int camera_key, double delta_time);
    void AddSmoothVelocityConstraint(int camera_key);
    void AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, int landmark_key, std::vector<gtsam::Point2> points,
                          std::vector<int> camera_keys, bool used=true, int survey=-1);
    
    void Clear();
    void PrintFactorGraph();
    virtual gtsam::Symbol GetSymbol(int survey, int pnum);
    
    std::vector<double> Params(){return vals;}
    static std::vector<std::string> Keys(){return keys;}
    void SetParams(std::vector<double> params){vals = params; InitializeNoiseModels();}
};


#endif /* defined(__BundleAdjustOneDataset__FactorGraph__) */
