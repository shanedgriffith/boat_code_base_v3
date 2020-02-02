#ifndef SRC_OPTIMIZATION_FACTORGRAPH_HPP_
#define SRC_OPTIMIZATION_FACTORGRAPH_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <DataTypes/Camera.hpp>
#include <DataTypes/LandmarkTrack.h>

#define GPS_NOISE 10 //10 meters deviation from actual position.
#define COMPASS_NOISE 0.1745 //10 degrees deviation from actual yaw angle. (noise for the measurements of change in yaw)
#define UNBOUND_ANGLE 1.0
#define UNBOUND_POS 10

class FactorGraph{
protected:
    static const std::vector<std::string> keys;
    bool deferred_add_landmarks = false; //defers until the next update.
    long landmarks;
    long landmark_constraints;
    long variables;
    long variable_constraints;
    
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
    gtsam::noiseModel::Diagonal::shared_ptr tightKinNoise;
    gtsam::noiseModel::Diagonal::shared_ptr orientNoise;
    gtsam::noiseModel::Diagonal::shared_ptr dVNoise;
    gtsam::noiseModel::Isotropic::shared_ptr pixelNoise;
//    gtsam::noiseModel::Robust::shared_ptr pixelNoise;
    
    int next_camera_key;
    int active_landmark_set;
    
    //true if the survey landmark tracks haven't been filtered out yet. sppf is set to filter bad points.
    bool sppf_prune = true;
public:
    enum class var {L, V, X};
    char key[3]={'l', 'v', 'x'};
    
    gtsam::NonlinearFactorGraph graph;
    std::vector<std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> > > landmark_factors; //used by the GTSAMInterface
    std::vector<std::vector<int> > landmark_keys; //used by the GTSAMInterface
    std::vector<std::vector<int> > landmark_to_graph_index;
    
    
    FactorGraph(): active_landmark_set(0), next_camera_key(0), landmarks(0), landmark_constraints(0), variables(0), variable_constraints(0) {
        ChangeLandmarkSet(0);
        InitializeNoiseModels();
    }
    
    virtual ~FactorGraph(){}

    bool hold_constant = false;
    void HoldConstant(bool h){hold_constant=h;}
    
    void ChangeLandmarkSet(int set);
    int GetNextCameraKey() {return next_camera_key++;}
    int GetActiveLandmarkSet(){return active_landmark_set;}
    
    void InitializeNoiseModels();
    
    void AddCamera(int camera_key, gtsam::Pose3 cam_est);
    void AddEssentialMatrix(int camera_key, gtsam::EssentialMatrix& E);
    void AddVelocity(int camera_key, gtsam::Pose3 vel_est);
    void AddOdomFactor(int camera_key, gtsam::Pose3 delta_pose, bool tight = false);
    void AddKinematicConstraint(int camera_key, double delta_time);
    void AddOrientationConstraint(int camera_key1, int camera_key2, const gtsam::Rot3& rot);
    void AddSmoothVelocityConstraint(int camera_key);
    virtual void AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, LandmarkTrack& landmark);
    void AddToExistingLandmark(gtsam::Point2& point, int survey, int camera_key, int smart_factor_idx);
    int GraphHasLandmark(int landmark_key);
    void SetLandmarkDeviation(double dev);
    
    void Clear();
    void PrintFactorGraph();
    void PrintStats();
    virtual gtsam::Symbol GetSymbol(int survey, int pnum);
    
    std::vector<double> Params(){return vals;}
    static std::vector<std::string> Keys(){return keys;}
    void SetParams(std::vector<double> params){vals = params; InitializeNoiseModels();}
};


#endif /* SRC_OPTIMIZATION_FACTORGRAPH_HPP_ */
