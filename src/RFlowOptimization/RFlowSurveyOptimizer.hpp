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

class RFlowSurveyOptimizer: public SurveyOptimizer {
protected:
    int MAX_ITERATIONS = 10;

    static const std::string _locoptname;

    double LoadHopDistance(std::string path, std::string date);
    std::vector<double> GetAvgHopCounts();
    void SaveLocalLog(int numverified);

    void Initialize();
    int GetLPDIdx(int por1time);
    ParseFeatureTrackFile LoadFTF(int time);
    void ModifyFTF(ParseFeatureTrackFile& pftf);

    void SaveResults();
    int UpdateError();
    void AddUnverified();
    
    void AddLocalizations();
    void StandAloneFactorGraph();
    
    ParseOptimizationResults POR;
    int latestsurvey;
    std::vector<double> lpd_rerror;
    std::vector<LocalizedPoseData> localizations;
    std::unordered_map<int, int> lpdtable;

	RFlowFactorGraph* rfFG;
    
    Camera& _cam;
public:
    std::string _date;
    std::string _results_dir;
    std::string _first_optimization_dir;
    std::string _pftbase;
    
	/* date is the survey that's not yet consistent with the others.
	 * the others are in folder _results_dir + "maps/"
	 * */
    RFlowSurveyOptimizer(Camera& cam, std::string date, std::string results_dir, std::string first_optimization_dir, std::string pftbase):
        POR(first_optimization_dir + date), _results_dir(results_dir), _first_optimization_dir(first_optimization_dir),
        _pftbase(pftbase), _cam(cam), SurveyOptimizer(cam, rfFG, date, results_dir, false), _date(date){
        
        std::cout << "RFlow Optimization for : " << date << std::endl;
        rfFG = new RFlowFactorGraph();
        FG = rfFG;
        SurveyOptimizer::Initialize();
        Initialize();
    }

    ~RFlowSurveyOptimizer(){
        delete(rfFG);
    }

    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_RFLOWSURVEYOPTIMIZER_HPP_ */
