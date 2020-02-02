//
//  GTSAMInterface.cpp
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 5/29/14.
//  Copyright (c) 2014 shane. All rights reserved.
//

#include "GTSAMInterface.h"

//#include <iostream>

//the algorithm used to solve the optimization problem. GTSam includes a bunch of choices.
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>  //used for visual slam. this is one nonlinear optimizer.
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>

using namespace gtsam;
using namespace std;


const vector<string> GTSAMInterface::keys = {
    "ANGLE_RELINEARIZE_THRESH", "YAW_RELINEARIZE_THRESH", "POS_RELINEARIZE_THRESH", "VP_RELINEARIZE_THRESH", "VA_RELINEARIZE_THRESH", "VYAW_RELINEARIZE_THRESH",
    "GTSAM_SKIP", "UPDATE_ITERATIONS"
};

std::vector<double> GTSAMInterface::PoseToVector(const gtsam::Pose3& cam)
{
    return {cam.x(), cam.y(), cam.z(), cam.rotation().roll(), cam.rotation().pitch(), cam.rotation().yaw()};
}

gtsam::Pose3 GTSAMInterface::VectorToPose(const std::vector<double>& p)
{
    return gtsam::Pose3(gtsam::Rot3::Ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

void GTSAMInterface::SetupSLAM(bool incremental) {
    /* Setup ISAM for the optimization. */
    if(incremental)
    {
        _incremental = true;
        FastMap<char, gtsam::Vector> thresholds;
        
        thresholds['x'] =  Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&vals[Param::POS_RELINEARIZE_THRESH]), 6, 1);
        thresholds['v'] =  Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&vals[Param::VP_RELINEARIZE_THRESH]), 6, 1);
        
        ISAM2Params parameters;
        parameters.relinearizeThreshold = thresholds;
        parameters.relinearizeSkip = vals[Param::GTSAM_SKIP];
        parameters.factorization = ISAM2Params::CHOLESKY; //QR is slower than CHOLESKY but more robust to wide differences in noise.
        //        parameters.findUnusedFactorSlots = true;
        //        parameters.evaluateNonlinearError = true;
        //        parameters.enablePartialRelinearizationCheck = false;
        parameters.findUnusedFactorSlots = true;
//        parameters.cacheLinearizedFactors = false;
        ISAM2 isam(parameters);
        i2 = isam;
    }
    
    _fg->Clear();
    initialEstimate.clear();
}

void GTSAMInterface::RemoveLandmarkFactor(int landmark_idx)
{
    if(landmark_idx > _fg->landmark_to_graph_index[_fg->GetActiveLandmarkSet()].size())
    {
        std::cout << "GTSAMInterface::RemoveLandmarkFactor() error. The landmark_idx hasn't been added to the factor indices." << std::endl;
        exit(-1);
    }
    int last_graph_idx = _fg->landmark_to_graph_index[_fg->GetActiveLandmarkSet()][landmark_idx];
    int factor_idx = last_factor_indices[last_graph_idx];
    
    factors_to_remove.push_back(factor_idx);
    
//    const NonlinearFactorGraph& nfg = i2.getFactorsUnsafe(); //reordered?
//    auto f = nfg[factor_idx];
//    std::cout << "set to remove landmark: " << landmark_idx << ", with factor idx: " << factor_idx << ", and factor: ";
//    f->printKeys();
}

void GTSAMInterface::Printi2Graph(const std::string& name, const gtsam::NonlinearFactorGraph& nfg)
{
    std::cout << name << std::endl;
    for(int i=0; i<nfg.size(); ++i)
    {
        auto f = nfg[i];
        
        std::cout << "factor: [" << i <<"]: ";
        if(f != nullptr and f->size() > 0)
            f->printKeys();
        else
            std::cout << " erased " << std::endl;
    }
}

void GTSAMInterface::IncrementalUpdate() {
    /* Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
     If accuracy is desired at the expense of time, update(*) can be called additional times
     to perform multiple optimizer iterations every step.
     */
    
//    std::cout << "incremental update. removing " << factors_to_remove.size() << std::endl;
//    initialEstimate.print();
//    _fg->graph.print();
    //    std::cout << "initial error. " << _fg->graph.error(initialEstimate) << std::endl;
//    Printi2Graph("Factor graph before update()", i2.getFactorsUnsafe());
//    Printi2Graph("Factors to add", _fg->graph);
    
//    _fg->graph.print();
//    initialEstimate.print();
//    try {
        int iterations = vals[Param::UPDATE_ITERATIONS];
        for(int i=0; i<iterations; i++) {
            if(i==0) {
                ISAM2Result res = i2.update(_fg->graph, initialEstimate, factors_to_remove);
                factors_to_remove.clear();
                last_factor_indices = res.newFactorsIndices;
            } else {
                i2.update();
            }
        }
        
        //results = i2.calculateEstimate(); //only calculate an estimate for a specific key, if the whole thing is needed, calculateBestEstimate()
        //isam2 holds its own copy of those values and the graph structure.
//        _fg->Clear();
        _fg->graph.resize(0);
        initialEstimate.clear();
//    } catch(const std::exception& ex) {
//        printf("There was an exception while attempting to solve the factor graph. ");
//        printf("Known causes of the exception:\n");
//        printf(" >The camera pose used to create landmark observations was not initialized\n");
//        printf(" >A prior was not specified for the first camera pose.\n");
//        printf(" >landmarks need to be observed at least twice.\n");
//        std::cout << ex.what()<<std::endl;
//        
//        if(debug) {
//            printf("\nThe factor graph.\n");
//            _fg->PrintFactorGraph();
//            printf("\n\n\nThe initial estimates.\n");
//            initialEstimate.print();
//            printf("Terminated with an exception.");
//            std::cout << ex.what()<<std::endl;
//            exit(-1);
//        }
//        exit(-1); //exit because the upkeep after update() wasn't done.
//    }
//    Printi2Graph("Factor graph after update()", i2.getFactorsUnsafe());
}


void GTSAMInterface::BatchUpdate() {
    
    std::cout << "Batch update. Factor graph size: " << _fg->graph.size() << std::endl;
    std::cout << "initial error. " << _fg->graph.error(initialEstimate) << std::endl;
    
    try {
        
        switch(_batch_optimizer) {
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
        printf("GTSAMInterface::RunBundleAdjustment() Exception.");
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
    
    std::cout << "final error. " << _fg->graph.error(results) << std::endl;
}

void GTSAMInterface::Update(bool everything)
{
    results.clear();
    if(_incremental)
    {
        IncrementalUpdate();
        if(everything)
        {
            results = i2.calculateBestEstimate();
        }
    }
    else
    {
        BatchUpdate();
    }
}

void GTSAMInterface::InitializePose(gtsam::Symbol s, gtsam::Pose3 p) {
    if(print_symbol_number) cout << "initializing " << s.key() << ": (" << (int) s.chr() << ", " << s.index() << ") " << endl;
    initialEstimate.insert(s, p);
}

void GTSAMInterface::InitializePose(char c, int num, gtsam::Pose3 p) {
    Symbol s(c, num);
    InitializePose(s, p);
}

boost::optional<gtsam::Point3> GTSAMInterface::triangulatePointFromPoseValues(gtsam::Values& v, int landmark_idx)
{
    boost::optional<gtsam::Point3> worldpoint = _fg->landmark_factors[_fg->GetActiveLandmarkSet()][landmark_idx].point(v);
    if(worldpoint) {return worldpoint.get();}
    else {
        if(debug) std::cout << "GTSAMInterface::MAPLandmarkEstimate() encountered degeneracy with landmark " << _fg->GetActiveLandmarkSet() << "." << landmark_idx << ". " << std::endl;
        return {};
    }
}

Point3 GTSAMInterface::MAPLandmarkEstimate(int landmark_idx) {
    /*In this function, the idx isn't the landmark_key*/
    Values* v;
    if(results.size()==0) v = &initialEstimate;
    else                  v = &results;
    boost::optional<gtsam::Point3> triangulated = triangulatePointFromPoseValues(*v, landmark_idx);
    if(triangulated) return triangulated.get();
    return gtsam::Point3(0,0,0);
}

std::shared_ptr<gtsam::Values> GTSAMInterface::getPoseValuesFrom(int var_id, int from_pose, int latest_pose)
{
    std::shared_ptr<gtsam::Values> v = std::make_shared<gtsam::Values>();
    for(int i=from_pose; i<latest_pose; ++i)
    {
        gtsam::Symbol s(var_id, i);
        gtsam::Pose3 cur = i2.calculateEstimate<gtsam::Pose3>(s);
        v->insert<gtsam::Pose3>(s, cur);
    }
    return v;
}

gtsam::Pose3
GTSAMInterface::
PoseResult(Symbol s)
{
    if(_incremental and not i2.valueExists(s))//exists() is O(1); uses a map.
        return i2.calculateEstimate<gtsam::Pose3>(s);
    else if(results.exists<Pose3>(s))
        return results.at<Pose3>(s);
    else if(initialEstimate.exists<Pose3>(s))
        return initialEstimate.at<Pose3>(s);
    else {
        cout<<"GTSAMInterface Error: Key ("<< s.chr() << "."<<s.index() << ") is not in the initial estimates or the graph. Can't get an estimate.\n"<<endl;
        exit(-1);
    }
}

bool GTSAMInterface::HasResult(Symbol s) {
    if(results.exists<Pose3>(s)) return true;
    if(initialEstimate.exists<Pose3>(s)) return true;
    return false;
}

vector<double> GTSAMInterface::MAPPoseEstimate(Symbol s) {
    gtsam::Pose3 ev = PoseResult(s);
    gtsam::Rot3 r = ev.rotation();
    vector<double> pose = {ev.x(), ev.y(), ev.z(), r.roll(), r.pitch(), r.yaw()};//, (double) s.index()
    return pose;
}

vector<vector<double> > GTSAMInterface::GetOptimizedLandmarks(bool sorted) {
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

vector<vector<double> > GTSAMInterface::GetOptimizedTrajectory(int var_id, int N) {
    vector<vector<double> > vel(N, vector<double>(6, 0));
    
    for(int i=0; i<N; i++) {
    	vector<double> pose;
    	gtsam::Symbol s(var_id, i);
    	if(HasResult(s)){
    	    pose = MAPPoseEstimate(s);
    	} else {
            if(debug)
                cout<<"GTSAMInterface WARNING: Symbol ("<<(char)s.chr()<<", "<<s.index() << ") isn't in the initial estimates or the graph. Can't get an estimate.\n"<<endl;
    		pose = {0,0,0,0,0,0};
    	}
        vel[i] = pose;
    }
    return vel;
}

void GTSAMInterface::PrintResults() {
    /*Print the estimate from bundle adjustment*/
    if(results.size() == 0) {
        printf("run the optimizer first...\n");
    } else {
        results.print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Current Estimate:<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
        printf("\n\n");
    }
}

void GTSAMInterface::PrintInitialEstimate() {
    initialEstimate.print("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Initial Estimate:<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
}








