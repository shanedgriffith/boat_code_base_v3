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


int EvaluateRFlow::GetIndexOfFirstPoint(std::vector<std::vector<double> >& landmarks, int id) {
    int bot = 0;
    int top = landmarks.size();
    while(bot < top) {
        int mid = bot + (top-bot)/2;
        if(((int)landmarks[mid][3]) > id) top = mid;
        else if(((int)landmarks[mid][3]) < id) bot = mid+1;
        else return mid;
    }
    return -1;
}

vector<gtsam::Point3> EvaluateRFlow::GetSubsetOf3DPoints(std::vector<std::vector<double> >& landmarks, vector<int>& ids_subset) {
    vector<gtsam::Point3> pset(ids_subset.size(), gtsam::Point3(0,0,0));
    if(ids_subset.size() == 0) return pset;
    
    int iter = -1;
    int countfound = 0;
    for(int i=0; i < ids_subset.size(); i++) {
        bool found = false;
        if(iter==-1) iter = GetIndexOfFirstPoint(landmarks, ids_subset[i]);
        
        if(iter>=0){
            for(int j=iter; j < landmarks.size(); j++) {
                if(((int)landmarks[j][3])==ids_subset[i]) {
                    pset[i] = gtsam::Point3(landmarks[j][0], landmarks[j][1], landmarks[j][2]); //check: does this deep copy?
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
    //if(debug) cout << "Found " << countfound << " of " << ids_subset.size() << " points." << endl;
    return pset;
}

double EvaluateRFlow::OnlineRError(ParseOptimizationResults& POR, int idx, std::string _pftbase, std::vector<double>& pose, std::vector<std::vector<double> >& landmarks) {
    ParseFeatureTrackFile PFT(_cam, _pftbase + _date, POR.ftfilenos[idx]);
    vector<gtsam::Point3> p_subset = GetSubsetOf3DPoints(landmarks, PFT.ids);
    return MeasureReprojectionError(pose, PFT.imagecoord, p_subset);
}

/*TODO: check this. It appears LPD either needs to be updated or I need to pass in the POR data. May need to use OnlineRError() */
double EvaluateRFlow::InterSurveyErrorAtLocalization(LocalizedPoseData& localization, std::vector<double>& boat, std::vector<std::vector<std::vector<double> > >& landmarks, int optstart){
    vector<gtsam::Point3> p3d0;
    int sidx = localization.s0 - optstart;
    if(sidx < 0) p3d0 = localization.p3d0;
    else p3d0 = GetSubsetOf3DPoints(landmarks[sidx], localization.pids);
    return MeasureReprojectionError(boat, localization.p2d1, p3d0, localization.rerrorp);
}

void EvaluateRFlow::Evaluate() {
    std::vector<LocalizedPoseData> localizations = LocalizedPoseData::LoadAll(_results_dir + _date);
    ParseOptimizationResults POR(_results_dir + _date);
    std::cout << "Evaluating " << _results_dir + _date << std::endl;

    vector<double> PostLocalizationRError = InterSurveyErrorAtLocalizations(localizations, POR.boat);
    SaveEvaluation(PostLocalizationRError, "/postlocalizationerror.csv");

//    vector<double> rerror = ErrorForSurvey();
//    SaveEvaluation(rerror);

    VisualizeDivergenceFromLocalizations(localizations, PostLocalizationRError);
}

/*plot each of the localized poses p1frame0; and every 10 or so of the optimized poses, each with their FOV.
 * -and different colors so they can be differentiated.
 * */
void EvaluateRFlow::VisualizeDivergenceFromLocalizations(std::vector<LocalizedPoseData>& localizations, vector<double>& rerror) {// vector<vector<double> >& traj,
    if(localizations.size() == 0){
        cout << "EvaluateRFlow::VisualizeDivergenceFromLocalizations() Error. Vector sizes are zero. Can't visualize the divergence."<<endl;
        exit(-1);
    }
    SLAMDraw draw;
    draw.SetScale(-300,300,-300,300);
    draw.ResetCanvas();

    int skip = 25;

    //draw outliers.
    for(int i=0; i<localizations.size(); i++){
        LocalizedPoseData& lpd = localizations[i];
        if(rerror[i] >= avgbadthreshold || rerror[i] < 0.0001)
            draw.AddPointPath(lpd.p1frame0[0], lpd.p1frame0[1], 255, 0, 0);
    }

    //draw inliers
    for(int i=0; i<localizations.size(); i++){
        LocalizedPoseData& lpd = localizations[i];
        if(rerror[i] < avgbadthreshold && rerror[i] > 0)
            draw.AddPointPath(lpd.p1frame0[0], lpd.p1frame0[1]);
    }

//    //draw the estimated landmark points
//    for(int i=0; i<localizations.size(); i++){
//        if(i>0 && localizations[i].s1time < localizations[i-1].s1time+skip-1) continue;
//        LocalizedPoseData& lpd = localizations[i];
//        draw.DrawSight(lpd.p1frame0[0], lpd.p1frame0[1], lpd.p1frame0[5]);
////        draw.DrawSight(traj[lpd.s1time][0], traj[lpd.s1time][1], traj[lpd.s1time][5], 0.86847586, 255, 0, 0);
//    }

    //save the visualization
    string visualization = _results_dir + _date + "/visual_LPD_check.jpg";
    draw.SaveDrawing(visualization);
}

void EvaluateRFlow::VisualizeFrameChange(std::vector<std::vector<double> >& traj, std::vector<LocalizedPoseData>& localizations) {
    if(localizations.size() == 0){
        cout << "EvaluateRFlow::VisualizeDivergenceFromLocalizations() Error. Vector sizes are zero. Can't visualize the divergence."<<endl;
        exit(-1);
    }
    SLAMDraw draw;
    draw.SetScale(-300,300,-300,300);
    draw.ResetCanvas();

    int skip = 25;

    //draw outliers.
    for(int i=0; i<localizations.size(); i++){
        LocalizedPoseData& lpd = localizations[i];
        draw.AddPointPath(lpd.p1frame0[0], lpd.p1frame0[1], 255, 0, 0);
        draw.AddPointPath(traj[lpd.s1time][0], traj[lpd.s1time][1], 0, 255, 0);
        draw.AddArrow(traj[lpd.s1time][0], traj[lpd.s1time][1], lpd.p1frame0[0], lpd.p1frame0[1]);
    }

    //save the visualization
    string visualization = _results_dir + _date + "/visual_offset_check.jpg";
    draw.SaveDrawing(visualization);
}


















