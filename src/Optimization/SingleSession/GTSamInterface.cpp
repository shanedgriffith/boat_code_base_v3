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

std::vector<double> GTSamInterface::PoseToVector(const gtsam::Pose3& cam){
    return {cam.x(), cam.y(), cam.z(), cam.rotation().roll(), cam.rotation().pitch(), cam.rotation().yaw()};
}

gtsam::Pose3 GTSamInterface::VectorToPose(const std::vector<double>& p){
#ifdef GTSAM4
    return gtsam::Pose3(gtsam::Rot3::Ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2])); //for GTSAM 4.0
#else
    return gtsam::Pose3(gtsam::Rot3::ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2])); //for GTSAM 3.2.1
#endif
}

void GTSamInterface::SetupIncrementalSLAM() {
    /*Setup ISAM for the optimization.
     */
    /*
     //double-check the examples to use this in GTSAM4.
     
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
    i2 = isam;*/
    _fg->Clear();
    initialEstimate.clear();
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
        _fg->Clear();
        initialEstimate.clear();
    } catch(const std::exception& ex) {
        printf("There was an exception while attempting to solve the factor graph.");
        printf("Known causes of the exception:\n");
        printf(" >The camera pose used to create landmark observations was not initialized\n");
        printf(" >A prior was not specified for the first camera pose.\n");
        printf(" >landmarks need to be observed at least twice.");
        std::cout << ex.what()<<std::endl;
        
        if(debug) {
            printf("\nThe factor graph.\n");
            _fg->PrintFactorGraph();
            printf("\n\n\nThe initial estimates.\n");
            initialEstimate.print();
            printf("Terminated with an exception.");
            std::cout << ex.what()<<std::endl;
            exit(-1);
        }
    }
}

void GTSamInterface::RunBundleAdjustment(int choix) {
    results.clear();
    //printf("Running the optimizer.\n=============================================================================\n");
    
//    std::cout << "fg size: " << _fg->graph.size() << std::endl;
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
        initialEstimate.clear();
    } catch(std::exception& ex) {
        printf("GTSamInterface::RunBundleAdjustment() Exception.");
        printf(" There was an exception while attempting to solve the factor graph.");
        printf(" Known causes of the exception:\n");
        printf("  >The camera pose used to create landmark observations was not initialized\n");
        printf("  >A prior was not specified for the first camera pose.\n");
        printf("  >landmarks need to be observed at least twice.\n");
        std::cout << "Process for: " << _identifier << std::endl;

        std::cout << ex.what() << std::endl;
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

void GTSamInterface::InitializePose(gtsam::Symbol s, gtsam::Pose3 p) {
    if(print_symbol_number) cout << "initializing " << s.key() << ": (" << (int) s.chr() << ", " << s.index() << ") " << endl;
    initialEstimate.insert(s, p);
}

void GTSamInterface::InitializePose(char c, int num, gtsam::Pose3 p) {
    Symbol s(c, num);
    InitializePose(s, p);
}

//void GTSamInterface::ClearInitialEstimate(){
//    initialEstimate.clear();
//}

Point3 GTSamInterface::MAPLandmarkEstimate(int idx) {
    /*In this function, the idx isn't the landmark_key*/
    Values* v;
    if(results.size()==0) v = &initialEstimate;
    else                  v = &results;
    boost::optional<gtsam::Point3> worldpoint = _fg->landmark_factors[_fg->GetActiveLandmarkSet()][idx].point(*v);
    if(worldpoint) return worldpoint.get();
    else {
        if(debug) std::cout << "GTSamInterface::MAPLandmarkEstimate() encountered degeneracy with landmark " << _fg->GetActiveLandmarkSet() << "." << idx << ". " << std::endl;
        return gtsam::Point3(0,0,0);
    }
}

gtsam::Pose3 GTSamInterface::PoseResult(Symbol s) {
    if(results.exists<Pose3>(s))
        return results.at<Pose3>(s);
    else if(initialEstimate.exists<Pose3>(s))
        return initialEstimate.at<Pose3>(s);
    else {
        cout<<"GTSamInterface Error: Key ("<< s.chr() << "."<<s.index() << ") is not in the initial estimates or the graph. Can't get an estimate.\n"<<endl;
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
    vector<double> pose = {ev.x(), ev.y(), ev.z(), r.roll(), r.pitch(), r.yaw()};//, (double) s.index()
    return pose;
}

vector<vector<double> > GTSamInterface::GetOptimizedLandmarks(bool sorted) {
    vector<vector<double> > landmarks;
    
    for(int i=0; i<_fg->landmark_keys[_fg->GetActiveLandmarkSet()].size(); i++) {
        gtsam::Point3 ev = MAPLandmarkEstimate(i);
        vector<double> landmark = {ev.x(), ev.y(), ev.z(), (double)_fg->landmark_keys[_fg->GetActiveLandmarkSet()][i]};
        landmarks.push_back(landmark);
    }
    
    if(sorted){
        std::sort(landmarks.begin(), landmarks.end(), [](const std::vector<double>& a, const std::vector<double>& b) {
            return a[3] < b[3];
        });
    }
    
    return landmarks;
}

vector<vector<double> > GTSamInterface::GetOptimizedTrajectory(int var_id, int N) {
    vector<vector<double> > vel(N, vector<double>(6, 0));
    
    for(int i=0; i<N; i++) {
    	vector<double> pose;
    	gtsam::Symbol s(var_id, i);
    	if(HasResult(s)){
    	    pose = MAPPoseEstimate(s);
    	} else {
            if(debug)
                cout<<"GTSamInterface WARNING: Symbol ("<<(char)s.chr()<<", "<<s.index() << ") isn't in the initial estimates or the graph. Can't get an estimate.\n"<<endl;
    		pose = {0,0,0,0,0,0};
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








