/*
 * AnchoredMultiSessionOptimization.hpp
 *
 *  Created on: ~June 28, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_ANCHOREDMULTISESSIONOPTIMIZATION_HPP_
#define SRC_RFLOWOPTIMIZATION_ANCHOREDMULTISESSIONOPTIMIZATION_HPP_


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

#include <Optimization/SurveyOptimizer.h>
#include <Optimization/GTSamInterface.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <DataTypes/Camera.hpp>

#include "RFlowFactorGraph.hpp"
#include "LPDInterface.hpp"
#include "LocalizedPoseData.hpp"

class AnchoredMultiSessionOptimization: public SurveyOptimizer {
protected:
    //number of surveys optimized before a survey is ''locked-in''. Constrained by the memory.
    const int K = 500;
    const int MAX_ITERATIONS = 10;
    
    int optstart;
    std::vector<std::string> dates;
    std::vector<LPDInterface> lpdi;
    std::vector<std::vector<double>> lpd_rerror;
    std::vector<std::vector<double>> lpd_eval;
    std::vector<ParseOptimizationResults> POR;
    std::vector<double> inlier_ratio;
    std::vector<double> heights;
    std::vector<std::vector<double> > rerrs;
    std::vector<double> AverageRerror;
    
    gtsam::Pose3 ComputeP1frame0(std::vector<double>& a0, std::vector<double>& a1, std::vector<double>& p1);
    
    void IdentifyOptimizationDates();
    void Initialize();
    void BuildLandmarkSet();
    void ConstructFactorGraph();
    void AddLocalizations(bool firstiter);
    void AddAllTheLandmarkTracks();
    double UpdateErrorAdaptive(bool firstiter);
    void SaveResults();
    
    RFlowFactorGraph* rfFG;
public:
    std::string _map_dir;
    std::string _pftbase;
    
    AnchoredMultiSessionOptimization(Camera& cam, std::string results_dir, std::string pftbase, std::string date = "");
    
    ~AnchoredMultiSessionOptimization(){
        delete(rfFG);
    }
    
    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_ANCHOREDMULTISESSIONOPTIMIZATION_HPP_ */
