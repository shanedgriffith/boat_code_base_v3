//
//  GTSamOne.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 5/29/14.
//  Copyright (c) 2014 shane. All rights reserved.
//

#ifndef __BundleAdjustOneDataset__GTSamInterface__
#define __BundleAdjustOneDataset__GTSamInterface__

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>    //calibration and performs projections
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h> //for calculating the marginal covariance of the desired variables.
#include <gtsam/nonlinear/Values.h>//the initial guess of each thing (be it a pose3, point3, point3, whatever).
#include <gtsam/nonlinear/Symbol.h>//using symbols to identify factors
#include <gtsam/base/debug.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>  //used for visual slam. this is one nonlinear optimizer.
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>

#include "FactorGraph.hpp"

class GTSamInterface {
private:
    bool debug = false;
    static const std::vector<std::string> keys;
    
    //set to true in case of SLAM problems. This shows the variable where the problem occurred,
    //which is necessary for isolating the cause.
    bool print_symbol_number = false;
    
    enum Param {
        ANGLE_RELINEARIZE_THRESH, YAW_RELINEARIZE_THRESH, POS_RELINEARIZE_THRESH, VP_RELINEARIZE_THRESH, VA_RELINEARIZE_THRESH, VYAW_RELINEARIZE_THRESH,
        GTSAM_SKIP, UPDATE_ITERATIONS
    };
    
    //defaults
    std::vector<double> vals = {
        0.05, 0.05, 0.05, 0.02, 0.02, 0.01,
        1, 2
    };
    
    gtsam::Values initialEstimate;
    gtsam::Values results;
    gtsam::ISAM2 i2;
    
    bool IsMeasurementPossible(gtsam::Point2 measurement);
    
    int next_camera_key = 0;
    int next_landmark_key = 0;
    
    int num_landmarks_in_graph=0;
    int num_cameras_in_graph=0;
    
public:
    
    FactorGraph * _fg;
    
    GTSamInterface():_fg(NULL){}
    GTSamInterface(FactorGraph * fg): _fg(fg){}
    
    enum Constants {
        LEVENBERG_MARQUARDT = 0,
        DOGLEG = 1,
        NONLINEAR = 2,
        GAUSS_NEWTON = 3
    };
    
    void RunBundleAdjustment(int choix=0);
    void SetupIncrementalSLAM();
    void Update();
    void ClearGraph();
    bool HasResult(gtsam::Symbol s);
    
    int GetNextCameraKey() {return next_camera_key++;}
    int GetNumCameras() {return num_cameras_in_graph;}
    
    void InitializeValue(gtsam::Symbol s, gtsam::Value * p);
    void InitializeValue(char c, int num, gtsam::Value * p);

    std::vector<double> MAPPoseEstimate(gtsam::Symbol s);
    std::vector<double> MAPPoseEstimate(int num);
    gtsam::Pose3 PoseResult(gtsam::Symbol s);
    gtsam::Point3 MAPLandmarkEstimate(int idx);
    std::vector<std::vector<double> > GetOptimizedLandmarks();
    std::vector<std::vector<double> > GetOptimizedTrajectory(int var_id, int N);
    
    void PrintInitialEstimate();
    void PrintResults();
    
    std::vector<double> Params(){return vals;}
    static std::vector<std::string> Keys(){return keys;}
    void SetParams(std::vector<double> params){vals = params;}
    void SetPrintSymbols(){print_symbol_number = true; cout <<"set print symbols"<<endl;}
};











#endif /* defined(__BundleAdjustOneDataset__GTSamInterface__) */
