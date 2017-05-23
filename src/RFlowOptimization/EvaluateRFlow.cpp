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

vector<double> EvaluateRFlow::MeasureReprojectionError(gtsam::Pose3 tf, vector<gtsam::Point2>& orig_imagecoords, vector<gtsam::Point3>& p, vector<double>& rerror) {
    double total_error = 0;
    double count = 0;
    double num_bad=0;
    string string_data = "";
    for(int j=0; j<orig_imagecoords.size(); j++) {
        if(rerror[j]<0.0001 || rerror[j]>6.0) continue;//skip points that are likely mismatches.
        if(p[j].x()==0.0 && p[j].y()==0.0 && p[j].z()==0.0) continue;

        gtsam::Point3 res = tf.transform_to(p[j]);
        gtsam::Point2 twodim = _cam.ProjectToImage(res);
        gtsam::Point2 orig = orig_imagecoords[j];

        double error = pow(pow(twodim.x() - orig.x(), 2)+pow(twodim.y() - orig.y(), 2),0.5);
        if(error>badthreshold) num_bad++;
        count++;
        total_error += error;
    }
    vector<double> res = {total_error, count, num_bad};
    return res;
}

vector<double> EvaluateRFlow::MeasureReprojectionError(vector<double>& boat, vector<gtsam::Point2>& orig_imagecoords, vector<gtsam::Point3>& p, vector<double>& rerror) {
    gtsam::Pose3 tf(gtsam::Rot3::ypr(boat[5],boat[4],boat[3]), gtsam::Point3(boat[0],boat[1],boat[2]));
    return MeasureReprojectionError(tf, orig_imagecoords, p, rerror);
}

double EvaluateRFlow::UpdateTots(vector<double>& tots, vector<double>& stats){
    double ret = 0.0;
    if(stats[1] > 0) {
        ret = stats[0]/stats[1];
        tots[0] += ret;
        tots[1]++;
        if(ret > avgbadthreshold) tots[3]++;
        else if(tots.size()>4){
            tots[4] += ret;
            tots[5]++;
        }
    }
    else tots[3]++;
    tots[2] += stats[2];
    return ret;
}

void EvaluateRFlow::PrintTots(vector<double> tots, string name){
    if(debug) {
        cout << "Average " + name + " Rerror: " << tots[0]/tots[1] << std::endl;
        cout << "  " << tots[4]/tots[5] << " average " + name + " rerror for inliers" << endl;
        cout << "  " << tots[2] << " unacceptable landmarks (not so high)" << endl;
        cout << "  " << tots[3] << " average unacceptable (should be nearly zero)" << endl;
    }
}

vector<double> EvaluateRFlow::ErrorForLocalizations(std::vector<LocalizedPoseData>& localizations, vector<vector<double> >& traj){
    vector<double> result(localizations.size(), 0.0);
    vector<double> tots(6, 0.0);
    for(int i=0; i<localizations.size(); i++) {
        vector<double> boat = traj[localizations[i].s1time];
        vector<double> stats = MeasureReprojectionError(boat, localizations[i].p2d1, localizations[i].p3d0, localizations[i].rerrorp);
        result[i] = UpdateTots(tots, stats);
    }

    PrintTots(tots, "LPD");
    return result;
}

//vector<double> EvaluateRFlow::ErrorForLocalizations(std::vector<LocalizedPoseData>& localizations, vector<Anchors>& anch, vector<vector<double> >& traj) {
//    ParseOptimizationResults por0(anch[0]._base + anch[0]._date);
//    ParseOptimizationResults por1(anch[1]._base + anch[1]._date);
//    vector<double> result(localizations.size(), 0.0);
//
//    vector<double> tots(6, 0.0);
//    for(int i=0; i<localizations.size(); i++) {
//        int s0time = localizations[i].s0time;
//        int s1time = localizations[i].s1time;
//        gtsam::Pose3 a0 = anch[0].GetAnchorAsPose(s0time);
//        gtsam::Pose3 a1 = anch[1].GetAnchorAsPose(s1time);
//        gtsam::Pose3 p0 = por0.CameraPose(s0time);
////        gtsam::Pose3 p1 = por1.CameraPose(s1time);
//        gtsam::Pose3 p1(gtsam::Rot3::ypr(traj[s1time][5], traj[s1time][4], traj[s1time][3]), gtsam::Point3(traj[s1time][0], traj[s1time][1], traj[s1time][2]));
//        gtsam::Pose3 p1frame0 = p0.compose((a0.compose(p0)).between(a1.compose(p1)));
//
//        vector<double> stats = MeasureReprojectionError(p1frame0, localizations[i].p2d1, localizations[i].p3d0, localizations[i].rerrorp);
//        result[i] = UpdateTots(tots, stats);
//    }
//
//    PrintTots(tots, "LPD");
//
//    return result;
//}

vector<double> EvaluateRFlow::SurveyErrorAtLocalizations(std::vector<LocalizedPoseData>& localizations){
    ParseOptimizationResults POR(base + _date);

    vector<double> result(localizations.size(), 0.0);
    vector<double> tots(4, 0.0);
    for(int i=0; i<localizations.size(); i++) {
        int idx = localizations[i].s1time;
        ParseFeatureTrackFile PFT(_cam, siftloc + _date, POR.ftfilenos[idx]);
        vector<gtsam::Point3> p_subset = POR.GetSubsetOf3DPoints(PFT.ids);
        vector<double> stats = EvaluateSLAM::MeasureReprojectionError(POR.boat[idx], PFT.imagecoord, p_subset);
        result[i] = UpdateTots(tots, stats);
    }
    PrintTots(tots, "");
    return result;
}

void EvaluateRFlow::Evaluate() {
    std::vector<LocalizedPoseData> localizations = LocalizedPoseData::LoadAll(base + _date);
    ParseOptimizationResults POR(base + _date);
    std::cout << "Evaluating " << base + _date << std::endl;

    vector<double> PostLocalizationRError = ErrorForLocalizations(localizations, POR.boat);
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
    string visualization = base + _date + "/visual_LPD_check.jpg";
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
    string visualization = base + _date + "/visual_offset_check.jpg";
    draw.SaveDrawing(visualization);
}


















