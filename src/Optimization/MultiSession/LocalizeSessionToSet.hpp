#ifndef SRC_RFLOWOPTIMIZATION_LocalizeSessionToSet_HPP_
#define SRC_RFLOWOPTIMIZATION_LocalizeSessionToSet_HPP_


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

class LocalizeSessionToSet: public SurveyOptimizer {
protected:
    //number of surveys optimized before a survey is ''locked-in''. Constrained by the memory.
    const int update_mult_factor = 3;
    
//    static const std::string locmapdir;
    std::vector<std::string> dates;
    std::vector<ParseOptimizationResults> POR;
    std::vector<double> rerrs;
    std::vector<double> lpd_rerror;
    std::vector<double> permerr;
    std::vector<double> lpd_eval;
    std::vector<double> lpd_sum;
    int outliers;
    std::vector<double> inter_error;
    LPDInterface lpdi;
    
    std::vector<std::vector<double> > poses;
    std::vector<std::vector<double> > landmarks;
    ParseOptimizationResults originPOR;
    
    
    void LoadFTF(ParseOptimizationResults& datePOR);
    
    void ConstructFactorGraph();
    int SessionToNum(std::string session);
    void AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, gtsam::Pose3 offset, double noise);
    void AddLocalizations();
    void Run();
    std::vector<bool> LPDInlierTest(int l, double LPD_RERROR_THRESHOLD, std::vector<double>& error);
    int EvaluateLPD(int j);
    double UpdateError();
    void Initialize();
    bool CheckSave();
    void SaveResults();
    void InlierOutlierStats();
    
    RFlowFactorGraph* rfFG;
public:
    std::string _ref_map_dir;
    std::string _loc_map_dir;
    std::string _pftbase;
    
    LocalizeSessionToSet(Camera& cam, std::string ref_map_dir, std::string loc_map_dir, std::string date, std::string pftbase, double percent_of_tracks = 100.0);
    
    ~LocalizeSessionToSet(){
        delete(rfFG);
    }
    
    void LocalizeSession();
};


#endif /* SRC_RFLOWOPTIMIZATION_LocalizeSessionToSet_HPP_ */







































