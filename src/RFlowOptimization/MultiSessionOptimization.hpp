/*
 * RFlowSurveyOptimizer.hpp
 *
 *  Created on: Jul 28, 2016
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_MULTISESSIONOPTIMIZATION_HPP_
#define SRC_RFLOWOPTIMIZATION_MULTISESSIONOPTIMIZATION_HPP_


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

#include "RFlowFactorGraph.hpp"

#include <Optimization/SurveyOptimizer.h>
#include <Optimization/GTSamInterface.h>
#include <ImageAlignment/DREAMFlow/ImageRetrieval.hpp>
#include <ImageAlignment/GeometricFlow/MultiSurveyViewpointSelection.hpp>

#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/ParamsInterface.h>
#include <FileParsing/SaveOptimizationResults.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <FileParsing/ParseSurvey.h>

#include <DataTypes/AlignmentResult.h>
#include <DataTypes/Map.hpp>
#include <DataTypes/Camera.hpp>
#include <DataTypes/LandmarkTrack.h>
#include <DataTypes/ImagePose.h>

#include "LPDInterface.hpp"
#include "LocalizedPoseData.hpp"

class MultiSessionOptimization: public SurveyOptimizer {
protected:
    //number of surveys optimized before a survey is ''locked-in''. Constrained by the memory.
    const int K = 5;
    const int MAX_ITERATIONS = 10;
    
    int optstart;
    std::vector<std::string> dates;
    std::vector<LPDInterface> lpdi;
    std::vector<std::vector<double>> lpd_rerror;
    std::vector<ParseOptimizationResults> POR;
    std::vector<std::unordered_map<int, int>> lmap;
    std::vector<double> inlier_ratio;
    
    void IdentifyOptimizationDates();
    void Initialize();
    void UpdateLandmarkMap(std::vector<LandmarkTrack>& tracks);
    void StandAloneFactorGraph(int survey, bool firstiter);
    void ConstructFactorGraph(bool firstiter);
    void AddLocalizations(bool firstiter);
    void AddAllTheLandmarkTracks();
    double UpdateError(bool firstiter);
    void SaveResults();
    
    RFlowFactorGraph* rfFG;
public:
    std::string _map_dir;
    std::string _pftbase;
    
    MultiSessionOptimization(Camera& cam, std::string results_dir, std::string pftbase, std::string date = ""):
        _map_dir(results_dir + "maps/"), _pftbase(pftbase),
        SurveyOptimizer(cam, rfFG, date, results_dir, false) {
        
        std::cout << "Multi-Session Optimization up to " << date << std::endl;
        rfFG = new RFlowFactorGraph();
        FG = rfFG;
        FG->SetLandmarkDeviation(3.0);
        std::cout << "  Initializing.."<<std::endl;
        SurveyOptimizer::Initialize();
        IdentifyOptimizationDates();
        Initialize();
    }

    ~MultiSessionOptimization(){
        delete(rfFG);
    }

    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_MULTISESSIONOPTIMIZATION_HPP_ */
