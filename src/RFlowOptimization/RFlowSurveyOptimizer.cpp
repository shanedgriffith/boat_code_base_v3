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
#include "HopcountLog.hpp"

using namespace std;

void RFlowSurveyOptimizer::Initialize() {
    int nloaded = lpdi.LoadLocalizations(_map_dir + _date);

    if(nloaded < POR.boat.size()*0.01) {
        std::cout<<"RFlowSurveyOptimizer Warning: There are too few localizations. Optimization will be the original set."<<std::endl;
        exit(-1);
    }

    lpd_rerror = vector<double>(nloaded, 0.0);
    latestsurvey = lpdi.localizations[0].s1;
}

void RFlowSurveyOptimizer::StandAloneFactorGraph() {
    LocalizePose lp(_cam);
    for(int i=0; i<POR.boat.size(); i++) {
        gtsam::Pose3 traj = POR.CameraPose(i);
        int lpdcur = lpdi.GetLPDIdx(i);
        if(lpdcur >= 0 && lpd_rerror[lpdcur] >= 0) {
            traj = lp.VectorToPose(lpdi.localizations[lpdcur].p1frame0); //TODO: change later.
        }
        rfFG->AddPose(latestsurvey, i, traj);
        GTS.InitializeValue(rfFG->GetSymbol(latestsurvey, i), &traj);

        if(i > 0) { //order matters; this has to be after the variables it depends on are initialized.
            gtsam::Pose3 cur1 = POR.CameraPose(i);
            gtsam::Pose3 last1 = POR.CameraPose(i-1);
            gtsam::Pose3 btwn = last1.between(cur1);
            rfFG->AddBTWNFactor(latestsurvey, i-1, latestsurvey, i, btwn);
        }

        ParseFeatureTrackFile pftf1 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + _date, POR.ftfilenos[i]);
        pftf1.ModifyFTFData(POR.GetSubsetOf3DPoints(pftf1.ids));
        vector<LandmarkTrack> tracks = pftf1.ProcessNewPoints(latestsurvey, i, active);
        AddLandmarkTracks(tracks);
    }

    //add the rest of the landmarks
    AddLandmarkTracks(active);
    active.clear();
}

void RFlowSurveyOptimizer::AddLocalizations(){
    for(int i=0; i<lpdi.localizations.size(); i++) {
        if(lpd_rerror[i] < 0) continue;
        LocalizedPoseData& lpd = lpdi.localizations[i];
        rfFG->AddLocalizationFactors(_cam.GetGTSAMCam(), lpd.s1, lpd.s1time, lpd.p3d0, lpd.p2d1, lpd.rerrorp);
    }
}

void RFlowSurveyOptimizer::SaveResults() {
    vector<vector<double> > ls = GTS.GetOptimizedLandmarks();
    vector<vector<double> > ts = GTS.GetOptimizedTrajectory(latestsurvey, POR.boat.size());
    vector<vector<double> > vs;
    SOR.SetSaveStatus();
    SOR.SetDrawMap();
    SOR.PlotAndSaveCurrentEstimate(ls, ts, vs, {});
}

int RFlowSurveyOptimizer::UpdateError() {
    static vector<double> permerr(lpd_rerror.size(), 0);
    EvaluateSLAM ESlam(_cam, _date, _map_dir);
    static vector<double> rerrs = ESlam.LoadRerrorFile();
    double mult = 3;
    EvaluateRFlow erfinter(_cam, _date, _map_dir);
    EvaluateRFlow erfintra(_cam, _date, _map_dir);
    erf.debug = true;
    
    int nchanges = 0;
    ParseOptimizationResults curPOR(_map_dir + _date);//yet, this result was saved over ..., it would make more sense to use the online error version.
    vector<vector<double> > poses = GTS.GetOptimizedTrajectory(latestsurvey, POR.boat.size());
    int coutliers = 0;
    for(int j=0; j<inter_error.size(); j++) {
        double intra_error = erfintra.OfflineRError(curPOR, lpdi.localizations[j].s1time, _pftbase);
        double inter_error = erfinter.InterSurveyErrorAtLocalization(lpdi.localizations[j], poses[lpdi.localizations[j].s1time]);
        
        //adaptive threshold.
        double LPD_RERROR_THRESHOLD = rerrs[lpdi.localizations[j].s1time]*mult;
        if(LPD_RERROR_THRESHOLD < 0) {
            std::cout << "RFlowSurveyOptimizer::UpdateError() Something went wrong with the Rerror file. Got negative rerror."<<std::endl;
            exit(1);
        } else if (LPD_RERROR_THRESHOLD == 0) {
            LPD_RERROR_THRESHOLD = ESlam.GetAverageRerror(rerrs)*mult;
        }
        
        bool inlier = true;
        if(std::isnan(inter_error) || LPD_RERROR_THRESHOLD <= inter_error) inlier = false;
        if(std::isnan(intra_error)) inlier = false;
        if(LPD_RERROR_THRESHOLD <= intra_error) permerr[j] = 1;
        if(permerr[j]>0) inlier = false;
        
        if((inlier && lpd_rerror[j] <= 0) || (!inlier && lpd_rerror[j]> 0)) nchanges++;
        
        lpd_rerror[j] = 1;
        if(!inlier) {lpd_rerror[j] = -1; coutliers++;}
    }
    erfintra.PrintTots("");
    erfinter.PrintTots("LPD");
    std::cout << "number of outliers: " << coutliers << std::endl;
    
    return nchanges;
}

void RFlowSurveyOptimizer::IterativeMerge() {
    LocalizePose lp(_cam);
    int numverified = lpdi.localizations.size();
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

        nchanges = UpdateError();
    }

    EvaluateRFlow erf(_cam, _date, _results_dir);
    erf.debug = true;
    vector<vector<double> > poses = GTS.GetOptimizedTrajectory(latestsurvey, POR.boat.size());
    vector<double> rerror_localizations = erf.InterSurveyErrorAtLocalizations(lpdi.localizations, poses);
    erf.SaveEvaluation(rerror_localizations, "/postlocalizationerror.csv");
    erf.VisualizeDivergenceFromLocalizations(lpdi.localizations, lpd_rerror);

    HopcountLog hlog(_map_dir);
    hlog.SaveLocalLog(_date, numverified, lpdi.localizations, lpd_rerror);
}




















