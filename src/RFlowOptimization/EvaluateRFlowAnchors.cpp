/*
 * EvaluateRFlow.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: shane
 */


#include "EvaluateRFlow.hpp"

#include <Visualizations/SLAMDraw.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <FileParsing/ParseFeatureTrackFile.h>

using namespace std;


double EvaluateRFlowAnchors::ComputeReprojectionError(std::vector<std::vector<double>> p_subset){
    for(int j=0; j<p_subset.size(); j++){
        if(p_subset[4]>badthreshold) num_bad++;
        count++;
        total_error += error;
    }
    std::vector<double> stats = {total_error, count, num_bad};
    return UpdateTots(stats);
}

std::vector<std::vector<double>> EvaluateRFlowAnchors::GetSubsetOfPoints(const std::vector<std::vector<double> >& landmarks, const vector<int>& ids_subset) {
    std::vector<std::vector<double>> pset(ids_subset.size(), vector<double>(5, 0));
    if(ids_subset.size() == 0) return pset;
    
    int iter = -1;
    int countfound = 0;
    for(int i=0; i < ids_subset.size(); i++) {
        bool found = false;
        if(iter==-1) iter = GetIndexOfFirstPoint(landmarks, ids_subset[i]);
        
        if(iter>=0){
            for(int j=iter; j < landmarks.size(); j++) {
                if(((int)landmarks[j][3])==ids_subset[i]) {
                    std::copy(landmarks[j].begin(), landmarks[j].end(), pset[i].begin());
                    iter = j;
                    countfound++;
                    found = true;
                    break;
                } else if(((int) landmarks[j][3]) > ids_subset[i]){
                    break;
                }
            }
        }
    }
    return pset;
}

double EvaluateRFlowAnchors::OnlineRError(ParseOptimizationResults& POR, int idx, std::string _pftset, const std::vector<std::vector<double> >& landmarks) {
    ParseFeatureTrackFile PFT(_cam, _pftset, POR.ftfilenos[idx]);
    vector<vector<double>> p_subset = GetSubsetOfPoints(landmarks, PFT.ids);
    return ComputeReprojectionError(p_subset);
}

double EvaluateRFlow::InterSurveyErrorAtLocalization(const LocalizedPoseData& localization, const std::vector<std::vector<std::vector<double> > >& landmarks) {
    vector<vector<double>> p_subset = GetSubsetOfPoints(landmarks[localization.s0], localization.pids);
    return ComputeReprojectionError(p_subset);
}


















