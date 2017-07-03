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

void MultiSessionOptimization::IdentifyOptimizationDates(){
    /*The dates in datetable*/
    cout << "   parsing the optimization results."<<endl;
    
    dates = FileParsing::ListDirsInDir(_map_dir);
    for(int i=0; i<dates.size(); i++){
        ParseOptimizationResults datePOR(_map_dir + dates[i]);
        POR.push_back(datePOR);
        if(_date.compare(dates[i])==0) break;
        if(i>0 && !HopcountLog::LogExists(_map_dir, alldates[i])) break; //if I need to re-run, won't this method fail?
    }
    
    optstart = min(POR.size()-K-1, 0);
    dates = std::vector<string>(dates.begin(), dates.begin()+POR.size());
}

void MultiSessionOptimization::Initialize() {
     /*Load the lpd data for each one.*/
    cout << "   loading the localizations."<<endl;
    cache_landmarks = true;
    for(int i=optstart; i<dates.size(); i++){
        LPDInterface lint;
        lpdi.push_back(lint);
        int nloaded = lint.LoadLocalizations(_map_dir + dates[i]);
        
        if(nloaded < POR[i].boat.size()*0.01) {
            std::cout<<"RFlowSurveyOptimizer Warning: There are too few localizations for " << dates[i] <<". Optimization will be the original set."<<std::endl;
            exit(-1);
        }
        
        std::vector<LandmarkTrack> clset;
        cached_landmarks.push_back(clset);
        
        HopcountLog hlog(_map_dir);
        vector<double> rerror_set = hlog.LoadPriorRerror(dates[i], count);
        lpd_rerror.push_back(rerror_set);
        
        int ninliers = 0;
        for(int i=0; i<rerror_set.size(); i++)
            ninliers += (rerror_set[i]==1)?1:0;
        inlier_ratios.push_back(1.*ninliers/rerror_set.size());
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
        if(latestsurvey && lpdcur >= 0 && lpd_rerror[sidx][lpdcur] >= 0) {
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
            vector<LandmarkTrack> tracks = pftf.ProcessNewPoints(survey, i, active);
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
    cout << "   adding the surveys"<<endl;
    
    //add the latest K surveys to the graph
    for(int survey=optstart; survey<dates.size(); survey++){
        cache_set = survey-optstart; //update the parent class variable
        if(firstiter){
            unordered_map<int, int> landmark_id_to_cache_idx;
            lmap.push_back(landmark_id_to_cache_idx);
        }
        StandAloneFactorGraph(survey, firstiter);
    }
}

void MultiSessionOptimization::AddAdjustableISC(int s0, int s1, int s1time, std::vector<int>& pids, std::vector<gtsam::Point2>& p2d1, bool on){
    int sidx = s0-optstart;
    for(int i=0; i<pids.size(); i++){
        auto search = lmap[sidx].find(pids[i]);
        if(search==lmap[sidx].end()){
            std::cout << "MultiSessionOptimization::AddAdjustableISC() Warning. The pid wasn't found in the map. ISC: " << s0 <<", " << s1 << ", "<<s1time << ", " << pids[i] << std::endl;
            exit(-1);
        }
        int lidx = search->second;
        cached_landmarks[sidx][lidx].AddToTrack(p2d1[i], s1, s1time, on); //none are used by default.
    }
}

void MultiSessionOptimization::AddLocalizations(bool firstiter){
    cout << "   adding the localizations."<<endl;
    for(int i=0; i<lpdi.size(); i++) {
        for(int j=0; j<lpdi[i].localizations.size(); j++) {
            LocalizedPoseData& lpd = lpdi[i].localizations[j];
            if(lpd.s0 < optstart) { //add localization factors for locked-in surveys
                if(lpd_rerror[i][j] < 0) continue;
                std::vector<gtsam::Point3> p3d0 = POR[lpd.s0].GetSubsetOf3DPoints(lpd.pids);
                rfFG->AddLocalizationFactors(_cam.GetGTSAMCam(), lpd.s1, lpd.s1time, p3d0, lpd.p2d1, lpd.rerrorp);
            } else if(firstiter) { //add ISCs for the remaining surveys
                AddAdjustableISC(lpd.s0, lpd.s1, lpd.s1time, lpd.pids, lpd.p2d1, lpd_rerror[i][j] >= 0);
                AddAdjustableISC(lpd.s1, lpd.s0, lpd.s0time, lpd.bids, lpd.b2d0, lpd_rerror[i][j] >= 0);
            }
        }
    }
}

void MultiSessionOptimization::AddAllTheLandmarkTracks(){
    cout << "   adding the landmark tracks."<<endl;
    
    //add all the landmark tracks to the factor graph.
    for(int i=0; i<cached_landmarks.size(); i++){
        rfFG->ChangeLandmarkSet(i);
        AddLandmarkTracks(cached_landmarks[i]);
    }
}

void MultiSessionOptimization::ToggleLandmarkConstraints(int s0, int s1, int s1time, std::vector<int>& pids, std::vector<gtsam::Point2>& p2d1){
    int sidx = s0-optstart;
    for(int i=0; i<pids.size(); i++){
        auto search = lmap[sidx].find(pids[i]);
        if(search==lmap[sidx].end()){
            std::cout << "MultiSessionOptimization::AddAdjustableISC() Warning. The pid wasn't found in the map. ISC: " << s0 <<", " << s1 << ", "<<s1time << ", " << pids[i] << std::endl;
            exit(-1);
        }
        int lidx = search->second;
        cached_landmarks[sidx][lidx].Toggle(s1, s1time);
    }
}

double MultiSessionOptimization::UpdateError(bool firstiter) {
    double mult = 3;
    static vector<vector<double> > permerr;
    static vector<vector<double> > rerrs;
    
    vector<vector<vector<double> > > landmarks;
    for(int i=optstart; i<dates.size(); i++){
        rfFG->ChangeLandmarkSet(sidx);
        vector<vector<double> > landmarkset = GTS.GetOptimizedLandmarks();
        landmarks.push_back(landmarkset);
    }
    
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
        vector<double> intra_error = erf.IntraSurveyErrorAtLocalizations(poses, landmarks[sidx], lpdi[sidx].localizations, POR[i], _pftbase);
        vector<double> inter_error = erf.InterSurveyErrorAtLocalizations(lpdi[sidx].localizations, poses, landmarks[sidx], optstart);
        int nchanges = 0;
        int coutliers = 0;
        for(int j=0; j<inter_error.size(); j++) {
            //adaptive threshold.
            double LPD_RERROR_THRESHOLD = rerrs[sidx][lpdi[sidx].localizations[j].s1time]*mult;
            bool inlier = true;
            if(std::isnan(next[j]) || inter_error[j] > LPD_RERROR_THRESHOLD) inlier = false;
            if(std::isnan(intra_error[j])) inlier = false;
            if(intra_error[j] > LPD_RERROR_THRESHOLD) permerr[sidx][j] = 1;
            if(permerr[sidx][j] > 0) inlier = false;
            
            if((inlier && lpd_rerror[sidx][j] < 0) || (!inlier && lpd_rerror[sidx][j] > 0)) {
                nchanges++;
                if(lpd.s0 >= optstart) {
                    ToggleLandmarkConstraints(lpd.s0, lpd.s1, lpd.s1time, lpd.pids, lpd.p2d1);
                    ToggleLandmarkConstraints(lpd.s1, lpd.s0, lpd.s0time, lpd.bids, lpd.b2d0);
                }
            }
            
            lpd_rerror[sidx][j] = 1;
            if(!inlier) {lpd_rerror[sidx][j] = -1; coutliers++;}
        }
        totchanges += nchanges;
        std::cout << "number of outliers: " << coutliers << std::endl;
        inlier_ratio[sidx] = 1.0-(1.*coutliers/inter_error.size());
    }
    
    return 1.0*totchanges/(dates.size()-optstart);
}

void MultiSessionOptimization::SaveResults() {
    
    for(int i=0; i<inlier_ratio.size(); i++)
        if(inlier_ratio[i] < 0.9){
            std::cout << "  Save disabled due to the inlier/outlier ratio for " << dates[i+optstart] << inlier_ratio[sidx] << std::endl;
            return;
        }
    
    vector<vector<vector<double> > > landmarks;
    for(int i=optstart; i<dates.size(); i++){
        rfFG->ChangeLandmarkSet(sidx);
        vector<vector<double> > landmarkset = GTS.GetOptimizedLandmarks();
        landmarks.push_back(landmarkset);
    }
    
    for(int i=optstart; i<dates.size(); i++){
        SaveOptimizationResults curSOR(_map_dir + dates[i]);
        int sidx = i-optstart;
        vector<vector<double> > ts = GTS.GetOptimizedTrajectory(i, POR.boat.size());
        vector<vector<double> > vs;
        curSOR.SetSaveStatus();
        curSOR.SetDrawMap();
        curSOR.PlotAndSaveCurrentEstimate(landmarks[sidx], ts, vs, {});
        
        EvaluateRFlow erf(_cam, dates[i], _map_dir);
        vector<vector<double> > poses = GTS.GetOptimizedTrajectory(i, POR.boat.size());
        vector<double> inter_error = erf.InterSurveyErrorAtLocalizations(lpdi[sidx].localizations, poses, landmarks[sidx], optstart);
        erf.SaveEvaluation(inter_error, "/postlocalizationerror.csv");
        erf.VisualizeDivergenceFromLocalizations(lpdi[sidx].localizations, lpd_rerror[sidx]);
        
        HopcountLog hlog(_map_dir);
        hlog.SaveLocalLog(dates[i], numverified, lpdi[sidx].localizations, lpd_rerror[sidx]);
    }
}

void MultiSessionOptimization::IterativeMerge() {
    LocalizePose lp(_cam);
    int numverified = lpdi.localizations.size();
    for(int i=0, nchanges=100; nchanges>1.0 && i<MAX_ITERATIONS; i++) {
        std::cout << "Merging Surveys. Iteration " << i << " of " << MAX_ITERATIONS << ", nchanges in last iteration: " << nchanges << std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        ConstructFactorGraph(i==0);
        AddLocalizations(i==0);
        AddAllTheLandmarkTracks();
        
        std::cout << "  Optimizing..." << std::endl;
        RunGTSAM();

        std::cout << "  Finished." << std::endl;
        rfFG->Clear();
        GTS.ClearGraph();

        nchanges = UpdateError(i==0);
    }
    
    if(!dry_run){
        std::cout << "  Saving.." << std::endl;
        SaveResults();
    }
}




















