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
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/LandmarkTrack.hpp>

using namespace std;

void MultiSessionOptimization::IdentifyOptimizationRange(){
    /*The dates in datetable*/
    
    dates = FileParsing::ListDirsInDir(_map_dir);
    for(int i=0; i<dates.size(); i++){
        if(!HopcountLog::LogExists(_map_dir, alldates[i])) break;
        ParseOptimizationResults datePOR(_map_dir + date);
        POR.push_back(datePOR);
    }
    
    optstart = min(POR.size()-K-1, 0);
    dates = std::vector<string>(dates.begin(), dates.begin()+POR.size());
}

void MultiSessionOptimization::Initialize() {
     /*Load the lpd data for each one.*/

    cache_landmarks = true;
    for(int i=optstart; i<dates.size(); i++){
        LPDInterface lint;
        lpdi.push_back(lint);
        int nloaded = lint.LoadLocalizations(_map_dir + dates[i]);
        
        if(nloaded < POR[i].boat.size()*0.01) {
            std::cout<<"RFlowSurveyOptimizer Warning: There are too few localizations for " << dates[i] <<". Optimization will be the original set."<<std::endl;
            exit(-1);
        }
        
        lpd_rerror.push_back(vector<double>(nloaded, 0.0));
        std::vector<LandmarkTrack> set;
        cached_landmarks.push_back(set);
    }
}

void MultiSessionOptimization::UpdateLandmarkMap(std::vector<LandmarkTrack> tracks){
    int curlength = cached_landmarks[cache_set].size();
    for(int i=0; i<tracks.size(); i++) {
        lmap[lmap.size()-1].insert({tracks[i].key, curlength + i});
    }
}

void MultiSessionOptimization::StandAloneFactorGraph(int survey, bool firstiter) {
    bool latestsurvey = survey == POR.size()-1;

    LocalizePose lp(_cam);
    for(int i=0; i<POR[survey].boat.size(); i++) {
        gtsam::Pose3 traj = POR[survey].CameraPose(i);
        int lpdcur = lpdi[survey].GetLPDIdx(i);
        if(!latestsurvey && lpdcur >= 0 && lpd_rerror[sidx][lpdcur] >= 0) {
            traj = lp.VectorToPose(lpdi[survey].localizations[lpdcur].p1frame0);
        }
        rfFG->AddPose(survey, i, traj);
        GTS.InitializeValue(rfFG->GetSymbol(survey, i), &traj);

        if(i > 0) { //order matters; this has to be after the variables it depends on are initialized.
            gtsam::Pose3 cur1 = POR[survey].CameraPose(i);
            gtsam::Pose3 last1 = POR[survey].CameraPose(i-1);
            gtsam::Pose3 btwn = last1.between(cur1);
            rfFG->AddBTWNFactor(survey, i-1, survey, i, btwn);
        }

        if(firstiter){
            ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[survey], POR[survey].ftfilenos[i]);
            pftf.ModifyFTFData(POR[survey].GetSubsetOf3DPoints(pftf.ids));
            vector<LandmarkTrack> tracks = ProcessNewPoints(i, pftf);
            UpdateLandmarkMap(tracks);
            CacheLandmarks(tracks);
        }
    }

    if(firstiter){
        //add the rest of the landmarks
        UpdateLandmarkMap(tracks);
        CacheLandmarks(active);
        active.clear();
    }
}

void MultiSessionOptimization::ConstructFactorGraph(bool firstiter){
    cout <<"\nConstructing the Factor Graph:" << endl;
    cout << "   adding the surveys"<<endl;
    
    //add all the surveys to the graph
    for(int i=optstart; i<dates.size(); i++){
        cache_set = i-optstart;
        if(firstiter){
            unordered_map<int, int> landmark_id_to_cache_idx;
            lmap.push_back(landmark_id_to_cache_idx);
        }
        StandAloneFactorGraph(i, firstiter);
    }
}

void MultiSessionOptimization::AddAdjustableISC(int s0, int s1, int s1time, std::vector<int>& pids, std::vector<gtsam::Point2>& p2d1){
    int sidx = s0-optstart;
    for(int i=0; i<pids.size(); i++){
        auto search = lmap[sidx].find(pids[i]);
        if(search==lmap[sidx].end()){
            std::cout << "Warning. The pid wasn't found in the map." << std::endl;
            continue;
        }
        int lidx = search->second;
        cached_landmarks[sidx][lidx].AddToTrack(0.0, p2d1[i], s1time); //TODO: fix. s1time isn't the camera_key.
    }
}

void MultiSessionOptimization::AddLocalizations(){
    for(int i=0; i<lpdi.size(); i++) {
        for(int j=0; j<lpdi[i].localizations.size(); j++) {
            if(lpd_rerror[i][j] < 0) continue;
            LocalizedPoseData& lpd = lpdi[i].localizations[j];
            if(lpd.s0 < optstart) { //localization factors are only added for locked-in surveys
                std::vector<gtsam::Point3> p3d0 = POR[lpd.s0].GetSubsetOf3DPoints(lpd.pids);
                rfFG->AddLocalizationFactors(_cam.GetGTSAMCam(), lpd.s1, lpd.s1time, p3d0, lpd.p2d1, lpd.rerrorp);
            } else {
                AddAdjustableISC(lpd.s0, lpd.s1, lpd.s1time, lpd.pids, lpd.p2d1);
                AddAdjustableISC(lpd.s1, lpd.s0, lpd.s0time, lpd.bids, lpd.b2d0);
            }
        }
    }
}

void MultiSessionOptimization::AddAllTheLandmarkTracks(){
    cout << "Adding the landmark tracks."<<endl;
    //add all the landmark tracks to the factor graph.
    for(int i=0; i<cached_landmarks.size(); i++){
        rfFG->ChangeLandmarkSet(i);
        AddLandmarkTracks(cached_landmarks[i]);
    }
}

double MultiSessionOptimization::UpdateError(bool firstiter) {
    double mult = 3;
    static vector<vector<double> > permerr;
    static vector<vector<double> > rerrs;
    
    double totchanges = 0;
    for(int i=optstart; i<dates.size(); i++){
        int sidx = i-optstart;
        if(firstiter){
            permerr.push_back(vector<double>(lpd_rerror[sidx].size(), 0));
            EvaluateSLAM ESlam(_cam, dates[i], _map_dir);
            rerrs.push_back(ESlam.LoadRerrorFile());
        }

        EvaluateRFlow erf(_cam, dates[i], _map_dir);
        erf.debug = true;
        
        vector<vector<double> > poses = GTS.GetOptimizedTrajectory(i, POR[i].boat.size());
        rfFG->ChangeLandmarkSet(sidx);
        vector<vector<double> > landmarks = GTS.GetOptimizedLandmarks();
        vector<double> next = erf.ErrorForLocalizations(lpdi[sidx].localizations, poses);
        vector<double> serror = erf.SurveyErrorAtLocalizations(poses, landmarks, lpdi[sidx].localizations, POR[i], _pftbase);
        int nchanges = 0;
        int coutliers = 0;
        for(int j=0; j<next.size(); j++) {
            //adaptive threshold.
            double LPD_RERROR_THRESHOLD = rerrs[lpdi[sidx].localizations[j].s1time]*mult;
            bool inlier = true;
            if(std::isnan(next[j]) || LPD_RERROR_THRESHOLD <= next[j]) inlier = false;
            if(std::isnan(serror[j])) inlier = false;
            if(LPD_RERROR_THRESHOLD <= serror[j]) permerr[j] = 1;
            if(permerr[j]>0) inlier = false;
            
            if((inlier && lpd_rerror[sidx][j] <= 0) || (!inlier && lpd_rerror[sidx][j]> 0)) nchanges++;
            
            lpd_rerror[sidx][j] = 1;
            if(!inlier) {lpd_rerror[sidx][j] = -1; coutliers++;}
        }
        totchanges += nchanges;
        std::cout << "number of outliers: " << coutliers << std::endl;
        
    }
    
    return 1.0*totchanges/(dates.size()-optstart);
}

void MultiSessionOptimization::SaveResults() {
    for(int i=optstart; i<dates.size(); i++){
        SaveOptimizationResults curSOR(_map_dir + dates[i]);
        int sidx = i-optstart;
        rfFG->ChangeLandmarkSet(sidx);
        vector<vector<double> > ls = GTS.GetOptimizedLandmarks();
        vector<vector<double> > ts = GTS.GetOptimizedTrajectory(i, POR.boat.size());
        vector<vector<double> > vs;
        curSOR.SetSaveStatus();
        curSOR.SetDrawMap();
        curSOR.PlotAndSaveCurrentEstimate(ls, ts, vs, {});
    }
}

void MultiSessionOptimization::IterativeMerge() {
    LocalizePose lp(_cam);
    int numverified = lpdi.localizations.size();
    for(int i=0, nchanges=100; nchanges>1.0 && i<MAX_ITERATIONS; i++) {
        std::cout << "Merging Surveys. Iteration " << i << " of " << MAX_ITERATIONS << ", nchanges in last iteration: " << nchanges << std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        ConstructFactorGraph(i==0);
        AddLocalizations();
        AddAllTheLandmarkTracks();
        
        std::cout << "  Optimizing..." << std::endl;
        RunGTSAM();

        std::cout << "  Finished." << std::endl;
        rfFG->Clear();
        GTS.ClearGraph();

        nchanges = UpdateError(i==0);
    }
    
    EvaluateRFlow erf(_cam, _date, _map_dir);
    erf.debug = true;
    vector<vector<double> > poses = GTS.GetOptimizedTrajectory(latestsurvey, POR.boat.size());
    vector<double> rerror_localizations = erf.ErrorForLocalizations(lpdi.localizations, poses);
    erf.SaveEvaluation(rerror_localizations, "/postlocalizationerror.csv");
    erf.VisualizeDivergenceFromLocalizations(lpdi.localizations, lpd_rerror);
    
    //only overwrite the localizations if the error is low enough, otherwise write everything to a dump folder?
    std::cout << "  Saving.." << std::endl;
    SaveResults();
    
    HopcountLog hlog(_map_dir);
    hlog.SaveLocalLog(_date, numverified, lpdi.localizations);
}




















