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

#include "LPDInterface.hpp"
#include "LocalizedPoseData.hpp"

class RFlowSurveyOptimizer: public SurveyOptimizer {
protected:
    int MAX_ITERATIONS = 10;

    void Initialize();

    void SaveResults();
    int UpdateError();
    void LoadUnverified();
    
    void AddLocalizations();
    void StandAloneFactorGraph();
    
    ParseOptimizationResults POR;
    int latestsurvey;
    std::vector<double> lpd_rerror;
    LPDInterface lpdi;
    
	RFlowFactorGraph* rfFG;
    
    Camera& _cam;
public:
    std::string _date;
    std::string _map_dir;
    std::string _pftbase;
    
    RFlowSurveyOptimizer(Camera& cam, std::string date, std::string results_dir, std::string pftbase):
        POR(results_dir + "maps/" + date), _map_dir(results_dir + "maps/"),
        _pftbase(pftbase), _cam(cam), SurveyOptimizer(cam, rfFG, date, results_dir + "maps/", false), _date(date){
        
        std::cout << "RFlow Optimization for : " << date << std::endl;
        rfFG = new RFlowFactorGraph();
        FG = rfFG;
        Initialize();
        rfFG->SetLandmarkDeviation(3.0); //must be *after* initialize();
    }

    ~RFlowSurveyOptimizer(){
        delete(rfFG);
    }

    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_RFLOWSURVEYOPTIMIZER_HPP_ */
