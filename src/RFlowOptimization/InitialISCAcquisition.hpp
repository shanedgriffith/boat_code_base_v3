/*
 * InitialISCAcquisition.hpp
 *
 *  Created on: Feb 16, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_InitialISCAcquisition_HPP_
#define SRC_RFLOWOPTIMIZATION_InitialISCAcquisition_HPP_


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <ImageAlignment/DREAMFlow/ImageRetrieval.hpp>
#include <ImageAlignment/GeometricFlow/MultiSurveyViewpointSelection.hpp>

#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/ParamsInterface.h>
#include <FileParsing/SaveOptimizationResults.h>
#include <FileParsing/ParseOptimizationResults.h>

#include <DataTypes/AlignmentResult.h>
#include <DataTypes/Map.hpp>
#include <DataTypes/Camera.hpp>
#include <DataTypes/LandmarkTrack.h>
#include <DataTypes/ImagePose.h>

#include "LocalizedPoseData.hpp"
#include "LPDInterface.hpp"

class InitialISCAcquisition{
private:
    int MAX_NO_ALIGN = 2;//5;
    double PERCENT_DENSE_CORRESPONDENCES = 0.4;//this threshold might be lowerable
    double verification_threshold = 0.4;
    
    static const std::string _logname;
    
    void Initialize();
    void WriteLog(std::vector<double> data);
    
    int RFViewpointSelection(std::vector<double>& rfpose);
    int IdentifyClosestPose(std::vector<double> pose1_est, std::string image1);
    std::vector<double> FindLocalization(int por0time, int por1time, bool hasRF, std::vector<double> pose1_est);
    
    std::vector<double> EstimateNextPose(int survey, int time, int por1time, bool ref);
    bool GetConstraints(int por1time, bool hasRF);
    int InitialISCAcquisitionWithRF(int por1time, int dir);
    int FindRestart();
    
    typedef struct {
        std::string date;
        ParseOptimizationResults por;
        double avg_hop_distance;
    } SurveyData;
    
    bool back_two = false;
    LPDInterface lpdi;
    std::vector<SurveyData> survey_est;
    std::vector<ReprojectionFlow*> rf;
    std::vector<Map*> _maps;
    Camera& _cam;
public:
    std::string _refdate, _priordate;
    std::string _query_loc, _pftbase, _origin;
    std::string _save_dir;
    bool debug = false;
    int nthreads = 8;
    
    InitialISCAcquisition(Camera& cam, std::string refdate, std::string priordate, std::string query_loc, std::string pftbase, std::string results_dir, std::string origin_dir);
    
    void Run(int user_specified_start=-1);
};



#endif /* SRC_RFLOWOPTIMIZATION_InitialISCAcquisition_HPP_ */



