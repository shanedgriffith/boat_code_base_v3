//
//  ViewpointMachine.hpp
//  boat_code_base
//
//  Created by Shane Griffith on 1/23/17.
//  Copyright © 2017 shane. All rights reserved.
//

#ifndef SRC_RFLOWOPTIMIZATION_OPTIMIZATIONMACHINE_HPP_
#define SRC_RFLOWOPTIMIZATION_OPTIMIZATIONMACHINE_HPP_

#include <stdio.h>
#include <ImageAlignment/FlowFrameworks/AlignmentMachine.h>
#include <Optimization/SingleSession/GTSamInterface.h>
#include <Optimization/SingleSession/FactorGraph.hpp>
#include <FileParsing/ParseOptimizationResults.h>
#include <FileParsing/ParamsInterface.h>
#include <DataTypes/Camera.hpp>
#include "RFlowFactorGraph.hpp"
#include "RFlowOptimization/LPDInterface.hpp"

class OptimizationMachine: public AlignmentMachine{
private:
    bool bFilter = true;
    
    std::vector<std::vector<std::vector<double> > > * posesTimeT2;
    std::vector<std::vector<std::vector<double> > > posesTimeT1;
    std::vector<std::vector<std::vector<double> > > * landmarks;
    std::vector<std::vector<double>> * lpd_rerror;
    ParseOptimizationResults * originPOR;
    std::vector<LandmarkTrack> * cached_landmarks;
    std::vector<std::vector<int> > * forwardLMap;
    std::vector<LPDInterface> * lpdi;
    double * avgposchange;
    double * avgorientchange;
    double _weight;
    int survey;
    
    void ConstructFactorGraph();
    void AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, gtsam::Pose3 offset, double noise);
    int AddDirectionalLocalization(int s, int j, int d);
    void AddLocalizations();
    int SessionToNum(std::string session);
    
    enum Direction {
        BACKWARD = 0,
        FORWARD = 1
    };
    
    std::vector<std::string> sessiondates;
    RFlowFactorGraph* rfFG;
    GTSamInterface GTS;
    Camera& _cam;
public:
    OptimizationMachine(Camera& cam, std::string _results_dir): _cam(cam){
        rfFG = new RFlowFactorGraph();
        GTS = GTSamInterface(rfFG);
        
        ParamsInterface PI;
        PI.LoadParams(_results_dir);
        GTS.SetParams(PI.LoadParams(GTSamInterface::Keys(), GTS.Params()));
        rfFG->SetParams(PI.LoadParams(FactorGraph::Keys(), rfFG->Params()));
        GTS.SetupIncrementalSLAM();
        Reset();
    }
    ~OptimizationMachine(){
        delete(rfFG);
    }
    void Setup(std::vector<std::vector<std::vector<double> > > * poses_,
               std::vector<std::vector<std::vector<double> > > * landmarks_,
               std::vector<std::vector<double>> * lpd_rerror_,
               ParseOptimizationResults * originPOR_,
               std::vector<LandmarkTrack> * cached_landmarks_,
               std::vector<std::vector<int> > * forwardLMap_,
               std::vector<LPDInterface> * lpdi_,
               int survey_);
    void SessionDates(std::vector<std::string>& dates);
    void toLogPoseChange(double * apc, double * aoc);
    void SetWeight(double weight);
    void Reset();
    void * Run();
    void LogResults();
    void FilterBad(bool filter=true);
};



#endif /* SRC_RFLOWOPTIMIZATION_OPTIMIZATIONMACHINE_HPP_ */
