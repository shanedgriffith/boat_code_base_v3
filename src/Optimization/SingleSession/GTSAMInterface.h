//
//  GTSamOne.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 5/29/14.
//  Copyright (c) 2014 shane. All rights reserved.
//

#ifndef SRC_OPTIMIZATION_GTSAMInterface_H_
#define SRC_OPTIMIZATION_GTSAMInterface_H_

//#include <optional>

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

class GTSAMInterface {
private:
    static const int _batch_optimizer = 0;
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
    bool _incremental = false;
    
    bool IsMeasurementPossible(gtsam::Point2 measurement);
    void Printi2Graph(const std::string& name, const gtsam::NonlinearFactorGraph& nfg);
    
    void BatchUpdate();
    void IncrementalUpdate();
    void checkbad();
    
    int last_landmark_idx;
    gtsam::FactorIndices factors_to_remove;
    gtsam::FactorIndices last_factor_indices;
    
public:
    std::string _identifier;
    FactorGraph * _fg;
    
    GTSAMInterface():_fg(NULL){}
    GTSAMInterface(FactorGraph * fg): _fg(fg){}
    
    enum Constants {
        LEVENBERG_MARQUARDT = 0,
        DOGLEG = 1,
        NONLINEAR = 2,
        GAUSS_NEWTON = 3
    };
    
    void SetupSLAM(bool incremental);
    void Update(bool everything = true);
    bool HasResult(gtsam::Symbol s);
    
    void InitializePose(gtsam::Symbol s, gtsam::Pose3 p);
    void InitializePose(char c, int num, gtsam::Pose3 p);

    std::vector<double> MAPPoseEstimate(gtsam::Symbol s);
    gtsam::Pose3 PoseResult(gtsam::Symbol s);
    gtsam::Point3 MAPLandmarkEstimate(int idx);
    std::vector<std::vector<double> > GetOptimizedLandmarks(bool sorted=false);
    std::vector<std::vector<double> > GetOptimizedTrajectory(int var_id, int N);
    
    void PrintInitialEstimate();
    void PrintResults();
    void SetIdentifier(std::string identifier) {_identifier = identifier;}
    
    std::vector<double> Params(){return vals;}
    static std::vector<std::string> Keys(){return keys;}
    void SetParams(std::vector<double> params){vals = params;}
    void SetPrintSymbols(){print_symbol_number = true; std::cout <<"set print symbols"<<std::endl;}
    
    
    static std::vector<double> PoseToVector(const gtsam::Pose3& cam);
    static gtsam::Pose3 VectorToPose(const std::vector<double>& p);
    
    std::shared_ptr<gtsam::Values> getPoseValuesFrom(int var_id, int from_pose, int latest_pose);
    boost::optional<gtsam::Point3> triangulatePointFromPoseValues(gtsam::Values& v, int landmark_idx);
    void RemoveLandmarkFactor(int landmark_idx);
};


#endif /* SRC_OPTIMIZATION_GTSAMInterface_H_ */
