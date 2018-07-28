/*
 * MultiSessionOptimization.hpp
 *
 *  Created on: ~June 28, 2017
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

#include <Optimization/SingleSession/SurveyOptimizer.h>
#include <Optimization/SingleSession/GTSamInterface.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <DataTypes/Camera.hpp>

#include "RFlowFactorGraph.hpp"
#include "RFlowOptimization/LPDInterface.hpp"
#include "RFlowOptimization/LocalizedPoseData.hpp"
#include "RFlowOptimization/EvaluateRFlow.hpp"

/*GOAL: To create a consistent map from multiple surveys using the TF from ISC constraints.
 Does so in an iterative way to add robustness to ISC outliers. 
 Multiple surveys are optimized at once.
 */
class MultiSessionOptimization: public SurveyOptimizer {
protected:
    //number of surveys optimized before a survey is ''locked-in''. Constrained by the memory.
    const int K = 500;
    const int MAX_ITERATIONS = 10;
    const int update_mult_factor = 3;
    bool bothinter = false;
    
    int optstart;
    std::vector<std::string> dates;
    std::vector<LPDInterface> lpdi;
    std::vector<int> outliers;
    std::vector<std::vector<double> > permerr;
    std::vector<std::vector<double>> lpd_rerror;
    std::vector<std::vector<double>> lpd_eval;
    std::vector<std::vector<double>> lpd_sum;
    std::vector<ParseOptimizationResults> POR;
    std::vector<double> heights;
    std::vector<std::vector<double> > rerrs;
    std::vector<double> AverageRerror;
    
    void SetHeight(gtsam::Pose3& traj, double z);
    void GetHeight(std::vector<std::vector<std::vector<double> > >& poses);
    
    void IdentifyOptimizationDates();
    virtual void Initialize();
    virtual void BuildLandmarkSet();
    virtual void ConstructFactorGraph();
    virtual void AddLocalizations(bool firstiter);
    void AddAllTheLandmarkTracks();
    virtual double UpdateErrorAdaptive(bool firstiter);
    virtual void SaveResults();
    bool CheckSave();
    virtual std::vector<bool> LPDInlierTest(int s, int l, double LPD_RERROR_THRESHOLD, std::vector<double>& error);
    virtual int EvaluateLPD(std::vector<std::vector<std::vector<double> > >& poseresult, std::vector<std::vector<std::vector<double> > >& landmarks, int s, int j, std::vector<EvaluateRFlow*> perf, std::vector<std::unordered_map<int, double> >& errorcache);
    double GetError(EvaluateRFlow& erflow, std::vector<double>& pose, std::vector<std::vector<double> >& landmarks, std::vector<LandmarkTrack>& cached_landmarks, int surveyTIME, std::unordered_map<int, double>& resultcache);
    virtual void PrintConvergenceStats(int s, const std::vector<EvaluateRFlow*> perf, int coutliers=-1);
    std::vector<double> InlierOutlierStats(bool compact = false);
    void Reset();
    int DateToIndex(std::string date);
    
    RFlowFactorGraph* rfFG;
public:
    std::string _map_dir;
    std::string _pftbase;
    
    MultiSessionOptimization(Camera& cam, std::string map_dir, std::string pftbase, std::string date = "", double percent_of_tracks = 100.0);

    ~MultiSessionOptimization(){
        delete(rfFG);
    }

    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_MULTISESSIONOPTIMIZATION_HPP_ */
