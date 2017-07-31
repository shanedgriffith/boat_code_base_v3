/*
 * RFlowSurveyOptimizer.cpp
 *
 *  Created on: Jul 28, 2016
 *      Author: shane
 */


#include "MultiSessionOptimization.hpp"

#include <random>
#include <string>
#include <Optimization/EvaluateSLAM.h>

#include "SFlowDREAM2RF.hpp"
#include "LocalizePose.hpp"
#include "EvaluateRFlow.hpp"
#include "HopcountLog.hpp"
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/LandmarkTrack.h>

using namespace std;

void MultiSessionOptimization::IdentifyOptimizationDates(){
    /*The dates in datetable*/
    cout << "   parsing the optimization results."<<endl;
    
    dates = FileParsing::ListDirsInDir(_map_dir);
    for(int i=0; i<dates.size(); i++){
        ParseOptimizationResults datePOR(_map_dir + dates[i]);
        POR.push_back(datePOR);
        if(_date.compare(dates[i])==0) break;
        if(i>0 && !HopcountLog::LogExists(_map_dir, dates[i])) break; //if I need to re-run, won't this method fail?
    }
    
    optstart = std::max(((int) POR.size()-K-1), 0);
    dates = std::vector<string>(dates.begin(), dates.begin()+POR.size());
}

void MultiSessionOptimization::Initialize() {
    SurveyOptimizer::Initialize();
     /*Load the lpd data for each one.*/
    cout << "   loading the localizations."<<endl;
    cache_landmarks = true;
    for(int i=optstart; i<dates.size(); i++){
        LPDInterface lint;
        int nloaded = lint.LoadLocalizations(_map_dir + dates[i]);
        lpdi.push_back(lint);
        
        if(i > 0 && nloaded < POR[i].boat.size()*0.01) {
            std::cout<<"RFlowSurveyOptimizer Warning: There are too few localizations for " << dates[i] <<". Optimizing a set that was previously optimized."<<std::endl;
            POR.erase(POR.end()-1);
            optstart = std::max(((int) POR.size()-K-1), 0);
            dates.erase(dates.end()-1);
            i--;
            continue;
        }
        
        std::vector<LandmarkTrack> clset;
        cached_landmarks.push_back(clset);
        
        HopcountLog hlog(_map_dir);
        vector<double> rerror_set = hlog.LoadPriorRerror(dates[i], nloaded);
        lpd_rerror.push_back(rerror_set);
        
        int ninliers = 0;
        for(int i=0; i<rerror_set.size(); i++)
            ninliers += (rerror_set[i]==1)?1:0;
        inlier_ratio.push_back(1.*ninliers/rerror_set.size());
    }
    heights = std::vector<double>(dates.size()-optstart, 0);
}

void MultiSessionOptimization::SetHeight(gtsam::Pose3& traj, double z){
    gtsam::Point3 t = traj.translation();
    gtsam::Rotation r = traj.rotation();
    t.z() = z;
    traj = gtsam::Pose3(r, t);
}

void MultiSessionOptimization::StandAloneFactorGraph(int survey, bool firstiter) {
    bool latestsurvey = survey == POR.size()-1;
    
    LocalizePose lp(_cam);
    for(int i=0; i<POR[survey].boat.size(); i++) {
        int sidx = survey - optstart;
        gtsam::Pose3 traj = POR[survey].CameraPose(i);
        int lpdcur = lpdi[survey].GetLPDIdx(i);
        if(latestsurvey && lpdcur >= 0 && lpd_rerror[sidx][lpdcur] >= 0) {
            traj = lp.VectorToPose(lpdi[survey].localizations[lpdcur].p1frame0);
        }
        if(!firstiter) SetHeight(traj, heights[sidx]); //set all the poses to the same height.
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
            CacheLandmarks(tracks);
        }
    }
    
    if(firstiter) {
        //add the rest of the landmarks
        CacheLandmarks(active);
        active.clear();
    }
}

void MultiSessionOptimization::ConstructFactorGraph(bool firstiter){
    cout << "   adding the surveys"<<endl;
    
    //add the latest K surveys to the graph
    for(int survey=optstart; survey<dates.size(); survey++){
        cache_set = survey-optstart; //update the parent class variable
        rfFG->ChangeLandmarkSet(cache_set);
        StandAloneFactorGraph(survey, firstiter);
    }
}

void MultiSessionOptimization::AddLocalizations(bool firstiter){
    cout << "   adding the localizations."<<endl;
    for(int i=0; i<lpdi.size(); i++) {
        for(int j=0; j<lpdi[i].localizations.size(); j++) {
            if(lpd_rerror[i][j] < 0) continue;
            LocalizedPoseData& lpd = lpdi[i].localizations[j];
            if(lpd.s0 < optstart) { //add localization factors for locked-in surveys
                std::vector<gtsam::Point3> p3d0 = POR[lpd.s0].GetSubsetOf3DPoints(lpd.pids);
                rfFG->AddLocalizationFactors(_cam.GetGTSAMCam(), lpd.s1, lpd.s1time, p3d0, lpd.p2d1, lpd.rerrorp);
            } else {
                rfFG->AddBTWNFactor(lpd.s0, lpd.s0time, lpd.s1, lpd.s1time, lpd.GetTFP0ToP1F0(), true);
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

void MultiSessionOptimization::GetHeight(vector<vector<vector<double> > >& poses){
    for(int i=0; i<poses.size(); i++){
        double sumz = 0
        for(int j=0; j<poses[i].size(); j++){
            sumz += poses[i][j][5];
        }
        heights[i] = sumz/poses[i].size();
    }
}

double MultiSessionOptimization::UpdateError(bool firstiter) {
    double mult = 3;
    static vector<vector<double> > permerr;
    static vector<vector<double> > rerrs;
    static vector<double> AverageRerror;
    vector<unordered_map<int, double> > intra;
    vector<vector<vector<double> > > poses;
    vector<vector<vector<double> > > landmarks;
    
    for(int i=optstart; i<dates.size(); i++){
        int sidx = i-optstart;
        rfFG->ChangeLandmarkSet(sidx);
        poses.push_back(GTS.GetOptimizedTrajectory(i, POR[i].boat.size()));
        landmarks.push_back(GTS.GetOptimizedLandmarks(true));
        
        if(firstiter){
            EvaluateSLAM ESlam(_cam, dates[i], _map_dir);
            rerrs.push_back(ESlam.LoadRerrorFile());
            double avg = ESlam.GetAverageRerror(rerrs[sidx]);
            AverageRerror.push_back(avg);
            
            permerr.push_back(vector<double>(lpd_rerror[sidx].size(), 0));
        }
        
        intra.push_back(unordered_map<int, double>());
    }
    GetHeight(poses); //update heights.
    
    double totchanges = 0;
    for(int i=optstart; i<dates.size(); i++){
        int sidx = i-optstart;
        
        EvaluateRFlow erfinter(_cam, dates[i], _map_dir);
        EvaluateRFlow erfintraS0(_cam, "-", _map_dir);
        EvaluateRFlow erfintraS1(_cam, dates[i], _map_dir);
        
        int nchanges = 0;
        int coutliers = 0;
        for(int j=0; j<lpdi[sidx].localizations.size(); j++) {
            LocalizedPoseData& lpd = lpdi[sidx].localizations[j];
            
            double inter_error = erfinter.InterSurveyErrorAtLocalization(lpd, poses[sidx][lpd.s1time], landmarks, optstart);
            double intra_errorS1 = erfintraS1.OnlineRError(POR[i], lpd.s1time, _pftbase+dates[i], poses[sidx][lpd.s1time], landmarks[sidx]);
            intra[sidx][lpd.s1time] = intra_errorS1;
            double intra_errorS0 = 0;
            if(lpd.s0 >= optstart) {
                int s0idx = lpd.s0-optstart;
                auto search = intra[s0idx].find(lpd.s0time);
                if(search != intra[s0idx].end())
                    intra_errorS0 = search->second;
                else {
                    intra_errorS0 = erfintraS0.OnlineRError(POR[lpd.s0], lpd.s0time, _pftbase+dates[lpd.s0], poses[s0idx][lpd.s0time], landmarks[s0idx]);
                    intra[s0idx][lpd.s0time] = intra_errorS0;
                }
            }
            
            //adaptive threshold.
            double LPD_RERROR_THRESHOLD = rerrs[sidx][lpd.s1time]*mult;
            if(LPD_RERROR_THRESHOLD < 0) {
                std::cout << "RFlowSurveyOptimizer::UpdateError() Something went wrong with the Rerror file. Got negative rerror."<<std::endl;
                exit(1);
            } else if (LPD_RERROR_THRESHOLD == 0) { //this occurs at places in the rerr vector that are zero.
                LPD_RERROR_THRESHOLD = AverageRerror[sidx]*mult;
            }
            
            bool inlier = true;
            if(std::isnan(inter_error) ||
               inter_error > LPD_RERROR_THRESHOLD) inlier = false;
            if(std::isnan(intra_errorS0) ||
               std::isnan(intra_errorS1)) inlier = false;
            if(intra_errorS0 > LPD_RERROR_THRESHOLD || //this threshold corresponds to s1, but it's inessential to change it.
               intra_errorS1 > LPD_RERROR_THRESHOLD) permerr[sidx][j] = 1;
            if(permerr[sidx][j] > 0) inlier = false;
            
            if(lpd_rerror[sidx][j] == 0 ||
               (inlier && lpd_rerror[sidx][j] < 0) ||
               (!inlier && lpd_rerror[sidx][j] > 0)) {
                nchanges++;
                if(inlier) std::cout << "activated   ("<<lpd.s1<<"."<<lpd.s1time << ", "<<lpd.s0<<"."<<lpd.s0time<<")" << std::endl;
                else       std::cout << "deactivated ("<<lpd.s1<<"."<<lpd.s1time << ", "<<lpd.s0<<"."<<lpd.s0time<<")" << std::endl;
            }
            
            lpd_rerror[sidx][j] = 1;
            if(!inlier) {lpd_rerror[sidx][j] = -1; coutliers++;}
        }
        totchanges += nchanges;
        
        if(i > 0) {
            erfintraS0.PrintTots("intra s0");
            erfintraS1.PrintTots("intra s1");
            erfinter.PrintTots("inter");
            std::cout << "number of outliers: " << coutliers << std::endl;
        }
        inlier_ratio[sidx] = 1.0-(1.*coutliers/lpdi[sidx].localizations.size());
    }
    
    //returns avg num_changes.
    if(optstart==0) return (int) (totchanges/(dates.size()-1)); //convert to int to avoid unnecessary iterations due to very small changes
    return (int) (totchanges/(dates.size()-optstart));
}

void MultiSessionOptimization::SaveResults() {
    
    for(int i=0; i<inlier_ratio.size(); i++)
        if(inlier_ratio[i] < 0.6){
            std::cout << "  Save disabled due to the inlier/outlier ratio for " << dates[i+optstart] << " with ratio " << inlier_ratio[i] << std::endl;
            return;
        }
    
    vector<vector<vector<double> > > landmarks;
    for(int i=optstart; i<dates.size(); i++){
        rfFG->ChangeLandmarkSet(i-optstart);
        landmarks.push_back(GTS.GetOptimizedLandmarks(true));
    }
    
    for(int i=optstart; i<dates.size(); i++){
        SaveOptimizationResults curSOR(_map_dir + dates[i]);
        int sidx = i-optstart;
        vector<vector<double> > ts = GTS.GetOptimizedTrajectory(i, POR[i].boat.size());
        vector<vector<double> > vs;
        curSOR.SetSaveStatus();
        curSOR.SetDrawMap();
        curSOR.PlotAndSaveCurrentEstimate(landmarks[sidx], ts, vs, {});
        
        EvaluateRFlow erfinter(_cam, dates[i], _map_dir);
        vector<vector<double> > poses = GTS.GetOptimizedTrajectory(i, POR[i].boat.size());
        if(i==0) continue;
        vector<double> inter_error(lpdi[sidx].localizations.size(), 0);
        for(int j=0; j<lpdi[sidx].localizations.size(); j++) {
            LocalizedPoseData& lpd = lpdi[sidx].localizations[j];
            inter_error[j] = erfinter.InterSurveyErrorAtLocalization(lpd, poses[lpd.s1time], landmarks, optstart);
        }
        erfinter.SaveEvaluation(inter_error, "/postlocalizationerror.csv");
        erfinter.VisualizeDivergenceFromLocalizations(lpdi[sidx].localizations, lpd_rerror[sidx]);
        
        HopcountLog hlog(_map_dir);
        hlog.SaveLocalLog(dates[i], lpd_rerror[sidx].size(), lpdi[sidx].localizations, lpd_rerror[sidx]);
    }
}

void MultiSessionOptimization::IterativeMerge() {
    time_t beginning,optstart,end;
    time (&beginning);
    LocalizePose lp(_cam);
    double last_nchanges = 10000000000;
    double avg_nchanges= last_nchanges - 1;
    for(int i=0; last_nchanges>avg_nchanges && i<MAX_ITERATIONS; i++) {
        rfFG->Clear();
        
        if(i>0) std::cout << "Merging Surveys. Iteration " << i << " of " << MAX_ITERATIONS << ", nchanges in last iteration: " << avg_nchanges << std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        ConstructFactorGraph(i==0);
        AddLocalizations(i==0);
        AddAllTheLandmarkTracks();
        
        std::cout << "  Optimizing..." << std::endl;
        time (&optstart);
        RunGTSAM();
        
        std::cout << "  Updating Error." << std::endl;
        last_nchanges = avg_nchanges;
        avg_nchanges = UpdateError(i==0);
        
        std::cout << "  Finished." << std::endl;
        time (&end);
        double optruntime = difftime (end, optstart);
        double totruntime = difftime (end, beginning);
        printf("ITERATION %d. AVG Nchanges %lf. Run time (HH:MM:SS) optimization %s, total %s\n", i, avg_nchanges, FileParsing::formattime(optruntime).c_str(), FileParsing::formattime(totruntime).c_str());
        std::cout << "changes, history: " <<last_nchanges << " -> " << avg_nchanges << std::endl;
    }
    
//    if(!dry_run){
//        std::cout << "  Saving.." << std::endl;
//        SaveResults();
//    }
}




















