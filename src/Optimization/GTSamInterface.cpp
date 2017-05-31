//
//  GTSamInterface.cpp
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 5/29/14.
//  Copyright (c) 2014 shane. All rights reserved.
//

#include "GTSamInterface.h"

//#include <iostream>

//the algorithm used to solve the optimization problem. GTSam includes a bunch of choices.
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>  //used for visual slam. this is one nonlinear optimizer.
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>

using namespace gtsam;
using namespace std;


const vector<string> GTSamInterface::keys = {
    "ANGLE_RELINEARIZE_THRESH", "YAW_RELINEARIZE_THRESH", "POS_RELINEARIZE_THRESH", "VP_RELINEARIZE_THRESH", "VA_RELINEARIZE_THRESH", "VYAW_RELINEARIZE_THRESH",
    "GTSAM_SKIP", "UPDATE_ITERATIONS"
};

void GTSamInterface::SetupIncrementalSLAM() {
    /*Setup ISAM for the optimization.
     */
    
    FastMap<char,gtsam::Vector> thresholds;
    
    thresholds['x'] = (gtsam::Vector(6) << vals[Param::POS_RELINEARIZE_THRESH],
                       vals[Param::POS_RELINEARIZE_THRESH],
                       vals[Param::POS_RELINEARIZE_THRESH],
                       vals[Param::ANGLE_RELINEARIZE_THRESH],
                       vals[Param::ANGLE_RELINEARIZE_THRESH],
                       vals[Param::ANGLE_RELINEARIZE_THRESH]);
    
//    thresholds['l'] = (gtsam::Vector(3) << vals[Param::POS_RELINEARIZE_THRESH],
//                       vals[Param::POS_RELINEARIZE_THRESH],
//                       vals[Param::POS_RELINEARIZE_THRESH]);
    
    thresholds['v'] = (gtsam::Vector(6) << vals[Param::VP_RELINEARIZE_THRESH],
                       vals[Param::VP_RELINEARIZE_THRESH],
                       vals[Param::VP_RELINEARIZE_THRESH],
                       vals[Param::VA_RELINEARIZE_THRESH],
                       vals[Param::VA_RELINEARIZE_THRESH],
                       vals[Param::VA_RELINEARIZE_THRESH]);
    
    ISAM2Params parameters;
    parameters.relinearizeThreshold = thresholds;
    parameters.relinearizeSkip = vals[Param::GTSAM_SKIP];
    parameters.factorization = ISAM2Params::CHOLESKY; //QR is slower than CHOLESKY but more robust to wide differences in noise.
    //        parameters.findUnusedFactorSlots = true;
    //        parameters.evaluateNonlinearError = true;
    //        parameters.enablePartialRelinearizationCheck = false;
    ISAM2 isam(parameters);
    i2 = isam;
    ClearGraph();
}

void GTSamInterface::Update() {
    /* Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
     If accuracy is desired at the expense of time, update(*) can be called additional times
     to perform multiple optimizer iterations every step.
     */
    
    results.clear();
    
    try {
        int iterations = vals[Param::UPDATE_ITERATIONS];
        for(int i=0; i<iterations; i++) {
            if(i==0) {
                i2.update(_fg->graph, initialEstimate);
            } else {
                i2.update();
            }
        }
        
        results = i2.calculateEstimate();
        ClearGraph();
    } catch(const std::exception& ex) {
        printf("There was an exception while attempting to solve the factor graph.");
        printf("Known causes of the exception:\n");
        printf(" >The camera pose used to create landmark observations was not initialized\n");
        printf(" >A prior was not specified for the first camera pose.\n");
        printf(" >landmarks need to be observed at least twice.");
        
        printf("\nThe factor graph.\n");
        _fg->PrintFactorGraph();
        printf("\n\n\nThe initial estimates.\n");
        initialEstimate.print();
        printf("Terminated with an exception.");
        std::cout << ex.what()<<std::endl;
        exit(-1);
    }
}

void GTSamInterface::RunBundleAdjustment(int choix) {
    results.clear();
    //printf("Running the optimizer.\n=============================================================================\n");
    try {
        switch(choix) {
            case LEVENBERG_MARQUARDT:
                results = LevenbergMarquardtOptimizer(_fg->graph, initialEstimate).optimize();
                break;
            case DOGLEG:
                results = DoglegOptimizer(_fg->graph, initialEstimate).optimize();
                break;
            case NONLINEAR:
                results = NonlinearConjugateGradientOptimizer(_fg->graph, initialEstimate).optimize();
                break;
            case GAUSS_NEWTON:
                results = GaussNewtonOptimizer(_fg->graph, initialEstimate).optimize();
                break;
            default:
                break;
        }
    } catch(const std::exception& ex) {
        printf("GTSamInterface::RunBundleAdjustment. Exception.");
        printf(" There was an exception while attempting to solve the factor graph.");
        printf(" Known causes of the exception:\n");
        printf("  >The camera pose used to create landmark observations was not initialized\n");
        printf("  >A prior was not specified for the first camera pose.\n");
        printf("  >landmarks need to be observed at least twice.\n");

        if(debug) {
            printf("\n The factor graph.\n");
            _fg->PrintFactorGraph();
            printf("\n\n\n The initial estimates.\n");
            initialEstimate.print();
            printf("Terminated with an exception.");
            std::cout << ex.what()<<std::endl;
        }
        exit(-1);
    }
}

void GTSamInterface::ClearGraph() {
    _fg->Clear();
    initialEstimate.clear();
    next_camera_key = 0;
    next_landmark_key = 0;
    num_landmarks_in_graph=0;
    num_cameras_in_graph=0;
}

void GTSamInterface::InitializeValue(gtsam::Symbol s, Value * p) {
    if(print_symbol_number) cout << "initializing " << s.key() << ": (" << (int) s.chr() << ", " << s.index() << ") " << endl;
    initialEstimate.insert(s, *p);
}

void GTSamInterface::InitializeValue(char c, int num, Value * p) {
    Symbol s(c, num);
    InitializeValue(s, p);
}

Point3 GTSamInterface::MAPLandmarkEstimate(int idx) {
    /*In this function, the idx isn't the landmark_key*/
    Values* v;
    if(results.size()==0) v = &initialEstimate;
    else                  v = &results;
    return _fg->landmark_factors[_fg->active_landmark_set][idx].point(*v).get();
}

gtsam::Pose3 GTSamInterface::PoseResult(Symbol s) {
    if(results.exists<Pose3>(s))
        return results.at<Pose3>(s);
    else if(initialEstimate.exists<Pose3>(s))
        return initialEstimate.at<Pose3>(s);
    else {
        cout<<"GTSamInterface Error: Key "<< s << " is not in the initial estimates or the graph. Can't get an estimate.\n"<<endl;
        exit(-1);
    }
}

bool GTSamInterface::HasResult(Symbol s) {
    if(results.exists<Pose3>(s)) return true;
    if(initialEstimate.exists<Pose3>(s)) return true;
    return false;
}

vector<double> GTSamInterface::MAPPoseEstimate(Symbol s) {
    gtsam::Pose3 ev = PoseResult(s);
    gtsam::Rot3 r = ev.rotation();
    vector<double> pose = {ev.x(), ev.y(), ev.z(), r.roll(), r.pitch(), r.yaw(), (double) s.index()};
    return pose;
}

vector<vector<double> > GTSamInterface::GetOptimizedLandmarks() {
    vector<vector<double> > landmarks;
    
    for(int i=0; i<_fg->landmark_keys[_fg->active_landmark_set].size(); i++) {
        gtsam::Point3 ev = MAPLandmarkEstimate(i);
        vector<double> landmark = {ev.x(), ev.y(), ev.z(), (double)_fg->landmark_keys[_fg->active_landmark_set][i]};
        landmarks.push_back(landmark);
    }
    return landmarks;
}

vector<vector<double> > GTSamInterface::GetOptimizedTrajectory(int var_id, int N) {
    vector<vector<double> > vel(N, vector<double>(7, 0));
    
    for(int i=0; i<N; i++) {
    	vector<double> pose;
    	gtsam::Symbol s(var_id, i);
    	if(HasResult(s)){
    	    pose = MAPPoseEstimate(s);
    	} else {
    		cout<<"GTSamInterface WARNING: Symbol ("<<(char)s.chr()<<", "<<s.index() << ") isn't in the initial estimates or the graph. Can't get an estimate.\n"<<endl;
    		pose = {0,0,0,0,0,0,(double)i};
    	}
        vel[i] = pose;
    }
    return vel;
}

void GTSamInterface::PrintResults() {
    /*Print the estimate from bundle adjustment*/
    if(results.size() == 0) {
        printf("run the optimizer first...\n");
    } else {
        results.print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Current Estimate:<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
        printf("\n\n");
    }
}

void GTSamInterface::PrintInitialEstimate() {
    initialEstimate.print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Initial Estimate:<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
}








