/*
 * AcquireISConstraints.hpp
 *
 *  Created on: Feb 16, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_ACQUIREISCONSTRAINTS_HPP_
#define SRC_RFLOWOPTIMIZATION_ACQUIREISCONSTRAINTS_HPP_


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
#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
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
#include "LPDInterface.hpp"

class AcquireISConstraints{
private:
    int MAX_NO_ALIGN = 2;//5;
    double PERCENT_DENSE_CORRESPONDENCES = 0.3;//this threshold might be lowerable

    static const std::string _logname;
    
    void Initialize();
    void WriteLog(std::vector<double> data, std::vector<std::string> paths = {});

    std::vector<std::vector<double> > IdentifyClosestPose(std::vector<double> pose1_est, std::string image1, bool run_initial_IR);
    std::vector<double> FindLocalization(std::vector<std::vector<double> > topk, int por1time, bool hasRF, std::vector<double> pose1_est);

    std::vector<double> EstimateNextPose(int survey, int time, int por1time, bool ref);
    bool GetConstraints(int por1time, bool hasRF);
    int AcquireISConstraintsWithRF(int por1time, int dir);
    int FindRestart();

    std::list<int> CreateList(MultiSurveyViewpointSelection& msvs);
    int GetLPDIdx(int por1time);
    LocalizedPoseData* NearestLPD(int s1time);
    bool StoreLPD(LocalizedPoseData lpd);
    
    typedef struct {
        std::string date;
        ParseOptimizationResults por;
        double avg_hop_distance;
    } SurveyData;
    
    int latestsurvey;
    bool back_two = false;
    LPDInterface lpdi;
    std::vector<SurveyData> survey_est;
    std::vector<ReprojectionFlow*> rf;
    std::vector<ReprojectionFlow*> rf_latest;
    std::vector<Map*> _maps;
    Camera& _cam;
public:
    std::string _date;
    std::string _query_loc, _pftbase, _first_optimization_dir;
    std::string _map_dir;
    bool debug = false;
    int nthreads = 8;
    
    AcquireISConstraints(Camera& cam, std::string date, std::string query_loc, std::string pftbase,
                         std::string first_optimization_dir, std::string results_dir):
    _cam(cam), _date(date), _query_loc(query_loc), _pftbase(pftbase),
    _first_optimization_dir(first_optimization_dir), _map_dir(results_dir + "maps/") {
        //i.e., the nonincremental approach. This approach is useful because progress between poses is otherwise unknown (unless we can measure odometry).
        std::cout << "Adding IS constraints." << std::endl;
        Initialize();
    }
    
    void Run(int start=-1);
};



#endif /* SRC_RFLOWOPTIMIZATION_ACQUIREISCONSTRAINTS_HPP_ */
