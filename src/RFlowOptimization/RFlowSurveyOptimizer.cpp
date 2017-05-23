/*
 * RFlowSurveyOptimizer.cpp
 *
 *  Created on: Jul 28, 2016
 *      Author: shane
 */


#include "RFlowSurveyOptimizer.hpp"

#include <random>
#include <string>
#include <Optimization/EvaluateSLAM.h>

#include "SFlowDREAM2RF.hpp"
#include "LocalizePose.hpp"
#include "EvaluateRFlow.hpp"

using namespace std;

const string RFlowSurveyOptimizer::_locoptname = "/locoptlist.csv";

double RFlowSurveyOptimizer::LoadHopDistance(string path, string date) {
    //loads the average hop count of one map.
    string fname = base + _date + _locoptname;
    FILE * fp = fopen(fname.c_str(), "r");
    double avg_hop_distance = 0.0;
    if(fp) {
        if(fscanf(fp, "%lf\n", &avg_hop_distance)!=1){
            std::cout << "AcquireISConstraints::LoadHopDistance(). Error scanning the file: " << fname << std::endl;
            exit(-1);
        }
        fclose(fp);
    }

    return avg_hop_distance;
}

std::vector<double> RFlowSurveyOptimizer::GetAvgHopCounts() {
    //returns the average hop count of all previous maps.
    string dir = base + "maps/";
    std::vector<double> ret;
    if(FileParsing::DirectoryExists(dir)){
        vector<string> dates = FileParsing::ListFilesInDir(dir, "1");
        for(int i=0; i<dates.size(); i++){
            double avghop = LoadHopDistance(dir, dates[i]);
            ret.push_back(avghop);
        }
    }
    return ret;
}

void RFlowSurveyOptimizer::SaveLocalLog(int numverified) {
    std::vector<double> hopcounts = GetAvgHopCounts();
    double sum = 0.0;
    int count = 0;
    for(int i=0; i<localizations.size(); i++){
        if(lpd_rerror[i]<=0) continue;
        count++;
        sum += hopcounts[localizations[i].s0];
    }
    double avg_hop_distance = 1 + sum/count;

    string fname = base + _date + _locoptname;
    FILE * fp = fopen(fname.c_str(), "w");
    fprintf(fp, "%lf\n", avg_hop_distance);
    int countout = 0;
    for(int i=0; i<localizations.size(); i++){
        int verified = (i < numverified)?1:0;
        fprintf(fp, "%d, %d, %d, %lf\n", localizations[i].s1time, verified, (int)lpd_rerror[i], 1 + hopcounts[localizations[i].s0]);
    }

    fflush(fp);
    fclose(fp);
}

void RFlowSurveyOptimizer::Initialize() {
    vector<LocalizedPoseData> input = LocalizedPoseData::LoadAll(base + _date);
    for(int i=0; i<input.size(); i++) {
        lpdtable[input[i].s1time] = localizations.size();
        localizations.push_back(input[i]);
    }

    if(localizations.size() < POR.boat.size()*0.01) {
        std::cout<<"RFlowSurveyOptimizer Warning: There are too few localizations. Optimization will be the original set."<<std::endl;
        exit(-1);
    }

    lpd_rerror = vector<double>(localizations.size(), 0.0);
    latestsurvey = localizations[0].s1;
}

void RFlowSurveyOptimizer::AddUnverified(){
    //if we only had unverified, a ransac approach might work better.
    std::cout << "DEPRECATED. Don't use this. The LPD threshold is used here, and I didn't get better performance." << std::endl;
    exit(-1);
    /*vector<LocalizedPoseData> unverified = LocalizedPoseData::LoadAll(base + _date, "/unverified/");
    std::cout <<localizations.size() << " Verified LPD, "<<unverified.size() << " unverified LPD, ";

    EvaluateRFlow erf(_cam, _date);
    erf.debug = true;
    vector<vector<double> > poses = GTS.GetOptimizedTrajectory(latestsurvey, POR.boat.size());
    vector<double> unext = erf.ErrorForLocalizations(unverified, poses);
    int nadded = 0;
    for(int i=0; i<unverified.size(); i++) {
        if(GetLPDIdx(unverified[i].s1time)<0 && unext[i] < LPD_RERROR_THRESHOLD){
            lpdtable[unverified[i].s1time] = localizations.size();
            localizations.push_back(unverified[i]);
            nadded++;
        }
    }
    std::cout <<localizations.size() << " combined LPD (added "<<nadded << ")"<<std::endl;

    //keep the lpd_rerror of the old set of poses, and add the new poses with an lpd_rerror of -1.0.
    vector<double> new_error(localizations.size(), -1.0);
    new_error.assign(lpd_rerror.begin(), lpd_rerror.end());
    lpd_rerror = new_error;*/
}

int RFlowSurveyOptimizer::GetLPDIdx(int por1time){
    auto search = lpdtable.find(por1time);
    if(search != lpdtable.end()){
        return search->second;
    }
    return -1;
}

ParseFeatureTrackFile RFlowSurveyOptimizer::LoadFTF(int time) {
    ParseFeatureTrackFile pftf = ParseFeatureTrackFile(_cam, siftloc + _date, POR.ftfilenos[time]);
    if(pftf.time == -1) {
        cout << "Error. GetConstraints() couldn't open: " << pftf.siftfile << endl;
        exit(-1);
    }
    return pftf;
}

void RFlowSurveyOptimizer::ModifyFTF(ParseFeatureTrackFile& pftf){
    //Finds and keeps only the landmarks that were good in the stand-alone optimization.
    vector<gtsam::Point3> p3d = POR.GetSubsetOf3DPoints(pftf.ids);
    vector<int> ids;
    vector<gtsam::Point2> p2d;
    for(int i=0; i<p3d.size(); i++){
        if(p3d[i].x()==0 && p3d[i].y()==0 && p3d[i].z()==0) continue;
        ids.push_back(pftf.ids[i]);
        p2d.push_back(pftf.imagecoord[i]);
    }
    pftf.ids = ids;
    pftf.imagecoord = p2d;
}

void RFlowSurveyOptimizer::StandAloneFactorGraph() {
    LocalizePose lp(_cam);
    for(int i=0; i<POR.boat.size(); i++) {
        gtsam::Pose3 traj = POR.CameraPose(i);
        int lpdcur = GetLPDIdx(i);
        if(lpdcur >= 0 && lpd_rerror[lpdcur] >= 0) {
            traj = lp.VectorToPose(localizations[lpdcur].p1frame0); //TODO: change later.
        }
        rfFG->AddPose(latestsurvey, i, traj);
        GTS.InitializeValue(rfFG->GetSymbol(latestsurvey, i), &traj);

        if(i > 0) { //order matters; this has to be after the variables it depends on are initialized.
            gtsam::Pose3 cur1 = POR.CameraPose(i);
            gtsam::Pose3 last1 = POR.CameraPose(i-1);
            gtsam::Pose3 btwn = last1.between(cur1);
            rfFG->AddBTWNFactor(latestsurvey, i-1, latestsurvey, i, btwn);
        }

        ParseFeatureTrackFile pftf1 = LoadFTF(i);
        ModifyFTF(pftf1);
        vector<LandmarkTrack> tracks = ProcessNewPoints(i, pftf1);
        AddLandmarkTracks(tracks, latestsurvey);
    }

    //add the rest of the landmarks
    AddLandmarkTracks(active, latestsurvey);
    active.clear();
}

void RFlowSurveyOptimizer::AddLocalizations(){
    for(int i=0; i<localizations.size(); i++) {
        if(lpd_rerror[i] < 0) continue;
        LocalizedPoseData& lpd = localizations[i];
        rfFG->AddLocalizationFactors(_cam.GetGTSAMCam(), lpd.s1, lpd.s1time, lpd.p3d0, lpd.p2d1, lpd.rerrorp);
    }
}

void RFlowSurveyOptimizer::SaveResults() {
    vector<vector<double> > ls = GTS.GetOptimizedLandmarks();
    vector<vector<double> > ts = GTS.GetOptimizedTrajectory(latestsurvey, POR.boat.size());
    vector<vector<double> > vs;
    SOR.SetSaveStatus();
    SOR.SetDrawMap();
    SOR.PlotAndSaveCurrentEstimate(ls, ts, vs);
}

int RFlowSurveyOptimizer::UpdateError() {
    static vector<double> permerr(lpd_rerror.size(), 0);
    static vector<double> rerrs = EvaluateSLAM::LoadRerrorFile(optimized_datasets, _date);
    double mult = 3;
    EvaluateRFlow erf(_cam, _date);
    erf.debug = true;

    int nchanges = 0;
    vector<vector<double> > poses = GTS.GetOptimizedTrajectory(latestsurvey, POR.boat.size());
    vector<double> next = erf.ErrorForLocalizations(localizations, poses);
    vector<double> serror = erf.SurveyErrorAtLocalizations(localizations);
    vector<double> both(next.size());
    int coutliers = 0;
    for(int j=0; j<next.size(); j++) {
        //adaptive threshold.
        double LPD_RERROR_THRESHOLD = rerrs[localizations[j].s1time]*mult;
        bool inlier = true;
        if(std::isnan(next[j]) || LPD_RERROR_THRESHOLD <= next[j]) inlier = false;
        if(std::isnan(serror[j])) inlier = false;
        if(LPD_RERROR_THRESHOLD <= serror[j]) permerr[j] = 1;
        if(permerr[j]>0) inlier = false;

        if((inlier && lpd_rerror[j] <= 0) || (!inlier && lpd_rerror[j]> 0)) nchanges++;

        lpd_rerror[j] = 1;
        if(!inlier) {lpd_rerror[j] = -1; coutliers++;}
    }
    std::cout << "number of outliers: " << coutliers << std::endl;

    return nchanges;
}

void RFlowSurveyOptimizer::IterativeMerge() {
    LocalizePose lp(_cam);
//    AddUnverified();
    int numverified = localizations.size();
    for(int i=0, nchanges=100; nchanges>0 && i<MAX_ITERATIONS; i++) {
        std::cout << "Merging Surveys. Iteration " << i << " of " << MAX_ITERATIONS << ", nchanges in last iteration: " << nchanges << std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        StandAloneFactorGraph();
        AddLocalizations();

        std::cout << "  Optimizing..." << std::endl;
        RunGTSAM();

        std::cout << "  Saving.." << std::endl;
        SaveResults();

        std::cout << "  Finished." << std::endl;
        rfFG->Clear();
        GTS.ClearGraph();

        nchanges = UpdateError();
    }

    EvaluateRFlow erf(_cam, _date);
    erf.debug = true;
    vector<vector<double> > poses = GTS.GetOptimizedTrajectory(latestsurvey, POR.boat.size());
    vector<double> rerror_localizations = erf.ErrorForLocalizations(localizations, poses);
    erf.SaveEvaluation(rerror_localizations, "/postlocalizationerror.csv");
    erf.VisualizeDivergenceFromLocalizations(localizations, lpd_rerror);

    SaveLocalLog(numverified);
}




















