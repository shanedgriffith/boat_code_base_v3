/*
 * EvaluateRFlow.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: shane
 */


#include "EvaluateRFlowAnchors.hpp"


using namespace std;

double EvaluateRFlowAnchors::ComputeReprojectionError(std::vector<std::vector<double>> p_subset){
    double total_error=0, count=0, num_bad=0;
    for(int j=0; j<p_subset.size(); j++){
        if(p_subset[j][0]==0.0 && p_subset[j][1]==0.0 && p_subset[j][2]==0.0) continue;
        if(p_subset[4]>badthreshold) num_bad++;
        count++;
        total_error += p_subset[4];
    }
    std::vector<double> stats = {total_error, count, num_bad};
    return UpdateTots(stats);
}

double EvaluateRFlowAnchors::ComputeNewReprojectionError(std::vector<double>& anchor, std::vector<double>& pose, const std::vector<std::vector<double> >& landmarks, ParseFeatureTrackFile& PFT){
    gtsam::Pose3 atf(gtsam::Rot3::ypr(anchor[5],anchor[4],anchor[3]), gtsam::Point3(anchor[0],anchor[1],anchor[2]));
    gtsam::Pose3 ptf(gtsam::Rot3::ypr(pose[5],pose[4],pose[3]), gtsam::Point3(pose[0],pose[1],pose[2]));
    gtsam::Pose3 shiftedp = atf.compose(ptf);
    
    double total_error=0, count=0, num_bad=0;
    int iter = -1;
    for(int i=0; i < PFT.ids.size(); i++) {
        if(iter==-1) iter = GetIndexOfFirstPoint(landmarks, PFT.ids[i]);
        
        if(iter>=0){
            for(int j=iter; j < landmarks.size(); j++) {
                if(((int)landmarks[j][3])==ids_subset[i]) {
                    //project the point
                    gtsam::Point3 res = shiftedp.transform_to(p_subset[j]);
                    gtsam::Point2 proj = _cam.ProjectToImage(res);
                    double dist = proj.distance(PFT.imagecoord[i]);
                    if(dist>badthreshold) num_bad++;
                    count++;
                    total_error += error;
                    
                    iter = j;
                    break;
                } else if(((int) landmarks[j][3]) > ids_subset[i]){
                    break;
                }
            }
        }
    }
    
    std::vector<double> stats = {total_error, count, num_bad};
    return UpdateTots(stats);
}

std::vector<std::vector<double>> EvaluateRFlowAnchors::GetSubsetOfPoints(const std::vector<std::vector<double> >& landmarks, const std::vector<int>& ids_subset) {
    std::vector<std::vector<double>> pset(ids_subset.size(), vector<double>(5, 0));
    if(ids_subset.size() == 0) return pset;
    
    int iter = -1;
    for(int i=0; i < ids_subset.size(); i++) {
        if(iter==-1) iter = GetIndexOfFirstPoint(landmarks, ids_subset[i]);
        
        if(iter>=0){
            for(int j=iter; j < landmarks.size(); j++) {
                if(((int)landmarks[j][3])==ids_subset[i]) {
                    std::copy(landmarks[j].begin(), landmarks[j].end(), pset[i].begin());
                    iter = j;
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

double EvaluateRFlowAnchors::InterSurveyErrorAtLocalization(const LocalizedPoseData& localization, const std::vector<std::vector<std::vector<double> > >& landmarks) {
    vector<vector<double>> p_subset = GetSubsetOfPoints(landmarks[localization.s0], localization.pids);
    return ComputeReprojectionError(p_subset);
}

double EvaluateRFlowAnchors::ComputeAnchorRError(vector<double>& anchor, ParseOptimizationResults& POR, int idx, std::string _pftset, const std::vector<std::vector<double> >& landmarks){
    ParseFeatureTrackFile PFT(_cam, _pftset, POR.ftfilenos[idx]);
    return ComputeNewReprojectionError(anchor, POR.boat[idx], landmarks, PFT);
}
















