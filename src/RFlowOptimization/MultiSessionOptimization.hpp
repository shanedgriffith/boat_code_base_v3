/*
 * RFlowSurveyOptimizer.hpp
 *
 *  Created on: Jul 28, 2016
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_RFLOWSURVEYOPTIMIZER_HPP_
#define SRC_RFLOWOPTIMIZATION_RFLOWSURVEYOPTIMIZER_HPP_


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

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

#include "LocalizedPoseData.hpp"

class MultiSessionOptimization: public SurveyOptimizer {
protected:
    //number of surveys optimized before a survey is ''locked-in''. Constrained by the memory.
    const int K = 5;
    
//    std::unordered_map<std::string, int> datetable;
    int optstart;
    std::vector<std::string> dates;
    std::vector<LPDInterface> lpdi;
    std::vector<std::vector<double>> lpd_rerror;
    std::vector<ParseOptimizationResults> POR;
    std::vector<unordered_map<int, int>> lmap;
    
    void IdentifyOptimizationDates();
    void UpdateLandmarkMap(std::vector<LandmarkTrack> tracks);
    void StandAloneFactorGraph(int survey, bool firstiter);
    void ConstructFactorGraph(bool firstiter);
    void AddAdjustableISC(int s0, int s1, int s1time, std::vector<int>& pids, std::vector<gtsam::Point2>& p2d1, bool on);
    void AddLocalizations();
    void AddAllTheLandmarkTracks();
    
    RFlowFactorGraph* rfFG;
public:
    std::string _map_dir;
    std::string _pftbase;
    
    MultiSessionOptimization(Camera& cam, std::string results_dir, std::string pftbase):
        _map_dir(results_dir + "maps/"), _pftbase(pftbase), _cam(cam),
        SurveyOptimizer(cam, rfFG, date, results_dir, false) {
        
        std::cout << "RFlow Optimization for : " << date << std::endl;
        rfFG = new RFlowFactorGraph();
        FG = rfFG;
        cout << "  Initializing.."<<endl;
        SurveyOptimizer::Initialize();
        IdentifyOptimizationDates();
        Initialize();
    }

    ~MultiSessionOptimization(){
        delete(rfFG);
    }

    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_RFLOWSURVEYOPTIMIZER_HPP_ */
