/*
 * MultiAnchorsOptimization.hpp
 *
 *  Created on: Aug 6, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_MULTIANCHORSOPTIMIZATION_HPP_
#define SRC_RFLOWOPTIMIZATION_MULTIANCHORSOPTIMIZATION_HPP_


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
#include "Anchors.hpp"
#include "FactorsToConstraint.hpp"

/*GOAL: scalable multi-survey optimization that's robust to outliers.
 Tries to replace all the survey data using anchors and then optimize using the information
 among the interconnections.
 */
class MultiAnchorsOptimization: public SurveyOptimizer {
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
    std::vector<Anchors> A;
    std::vector<FactorsToConstraint> constraints;
    
    void PrintStats(std::vector<std::unordered_map<int, double> >& intra, std::vector<std::unordered_map<int, double> >& inter);
    void IdentifyOptimizationDates();
    void Initialize();
    void BuildLandmarkSet();
    void ConstructFactorGraph(bool firstiter);
    double UpdateErrorAdaptive(bool firstiter);
    void SaveResults();
    
    RFlowFactorGraph* rfFG;
public:
    std::string _map_dir;
    std::string _pftbase;
    std::string _query_loc;
    
    MultiAnchorsOptimization(Camera& cam, std::string results_dir, std::string pftbase, std::string query_loc, std::string date = "");
    
    ~MultiAnchorsOptimization(){
        //delete(rfFG);
    }
    
    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_MULTIANCHORSOPTIMIZATION_HPP_ */
