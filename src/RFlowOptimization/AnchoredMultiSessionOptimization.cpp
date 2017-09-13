/*
 * AnchoredMultiSessionOptimization.cpp
 *
 *  Created on: ~June 28, 2017
 *      Author: shane
 */


#include "AnchoredMultiSessionOptimization.hpp"

#include <random>
#include <string>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/SaveOptimizationResults.h>
#include <Optimization/EvaluateSLAM.h>

#include "SFlowDREAM2RF.hpp"
#include "LocalizePose.hpp"
#include "EvaluateRFlow.hpp"
#include "HopcountLog.hpp"
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/LandmarkTrack.h>

using namespace std;

AnchoredMultiSessionOptimization::AnchoredMultiSessionOptimization(Camera& cam, std::string results_dir, std::string pftbase, std::string date):
_map_dir(results_dir + "maps/"), _pftbase(pftbase),
SurveyOptimizer(cam, rfFG, date, results_dir, false) {
    rfFG = new RFlowFactorGraph();
    FG = rfFG;
    
    std::cout << "Anchored Multi-Session Optimization up to " << date << std::endl;
    
    std::cout << "  Initializing.."<<std::endl;
    IdentifyOptimizationDates();
    Initialize();
    rfFG->SetLandmarkDeviation(3.0); //must be *after* initialize();
}

void AnchoredMultiSessionOptimization::IdentifyOptimizationDates(){
    /*The dates in datetable*/
    cout << "   parsing the optimization results."<<endl;
    
    dates = FileParsing::ListDirsInDir(_map_dir);
    for(int i=0; i<dates.size(); i++){
        ParseOptimizationResults datePOR(_map_dir + dates[i]);
        POR.push_back(datePOR);
        if(_date.compare(dates[i])==0) break;
        if(i>0 && !HopcountLog::LogExists(_map_dir, dates[i])) break; //if I need to re-run, won't this method fail?
    }
    
    optstart = std::max(((int) POR.size()-K), 0);
    dates = std::vector<string>(dates.begin(), dates.begin()+POR.size());
}

void AnchoredMultiSessionOptimization::Initialize() {
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
        
        EvaluateSLAM ESlam(_cam, dates[i], _map_dir);
        rerrs.push_back(ESlam.LoadRerrorFile());
        double avg = ESlam.GetAverageRerror(rerrs[i-optstart]);
        AverageRerror.push_back(avg);
        
        std::vector<LandmarkTrack> clset;
        cached_landmarks.push_back(clset);
        
        HopcountLog hlog(_map_dir);
        vector<double> rerror_set = hlog.LoadPriorRerror(dates[i], nloaded);
        lpd_rerror.push_back(rerror_set);
        lpd_eval.push_back(vector<double>(rerror_set.size(), 3.0));
        
        int ninliers = 0;
        for(int i=0; i<rerror_set.size(); i++)
            ninliers += (rerror_set[i]==1)?1:0;
        inlier_ratio.push_back(1.*ninliers/rerror_set.size());
    }
    heights = std::vector<double>(dates.size()-optstart, 0);
    BuildLandmarkSet();
}

void AnchoredMultiSessionOptimization::BuildLandmarkSet() {
    for(int survey=optstart; survey<dates.size(); survey++) {
        cache_set = survey-optstart; //update the parent class variable
        rfFG->ChangeLandmarkSet(cache_set);
        
        for(int i=0; i<500; i++){ //POR[survey].boat.size(); i++) {
            ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[survey], POR[survey].ftfilenos[i]);
            pftf.ModifyFTFData(POR[survey].GetSubsetOf3DPoints(pftf.ids));
            vector<LandmarkTrack> tracks = pftf.ProcessNewPoints(survey, i, active);
            CacheLandmarks(tracks);
        }
        
        //add the rest of the landmarks
        CacheLandmarks(active);
        active.clear();
    }
}

void AnchoredMultiSessionOptimization::ConstructFactorGraph() {
    cout << "   adding the surveys"<<endl;
    
    //add the latest K surveys to the graph
    for(int survey=optstart; survey<dates.size(); survey++){
        
        bool latestsurvey = survey == POR.size()-1;
        int sidx = survey - optstart;
        
        gtsam::Pose3 anc = gtsam::Pose3(gtsam::Rot3::ypr(0., 0., 0.), gtsam::Point3(0., 0., 0.));
        if(survey==0) rfFG->AddPose(survey, POR[survey].boat.size(), anc, true);
        else rfFG->AddPose(survey, POR[survey].boat.size(), anc, false);
        GTS.InitializeValue(rfFG->GetSymbol(survey, POR[survey].boat.size()), &anc);
        
        LocalizePose lp(_cam);
        for(int i=0; i<500; i++){ //POR[survey].boat.size(); i++) {
            gtsam::Pose3 traj = POR[survey].CameraPose(i);
            int lpdcur = lpdi[sidx].GetLPDIdx(i);
            if(latestsurvey && lpdcur >= 0 && lpd_rerror[sidx][lpdcur] >= 0) { //lpd_eval[sidx][lpdcur] < 6){ //
                traj = lp.VectorToPose(lpdi[sidx].localizations[lpdcur].p1frame0);
            }
            rfFG->AddPose(survey, i, traj);
            GTS.InitializeValue(rfFG->GetSymbol(survey, i), &traj);
            
            if(i > 0) { //order matters; this has to be after the variables it depends on are initialized.
                gtsam::Pose3 cur1 = POR[survey].CameraPose(i);
                gtsam::Pose3 last1 = POR[survey].CameraPose(i-1);
                gtsam::Pose3 btwn = last1.between(cur1);
                //            rfFG->AddBTWNFactor(survey, i-1, survey, i, btwn);
                rfFG->AddCustomBTWNFactor(survey, i-1, survey, i, btwn, 0.01);
            }
        }
    }
}

void AnchoredMultiSessionOptimization::AddLocalizations(bool firstiter){
    cout << "   adding the localizations."<<endl;
    for(int i=0; i<lpdi.size(); i++) {
        for(int j=0; j<lpdi[i].localizations.size(); j++) {
            if(lpd_rerror[i][j] < 0) continue;
            LocalizedPoseData& lpd = lpdi[i].localizations[j];
            if(lpd.s0time >= 500 || lpd.s1time >= 500) continue;
            double noise = pow(2, lpd_eval[i][j]/3.0) * 0.0001;
            int anum0 = POR[lpd.s0].boat.size();
            int anum1 = POR[lpd.s1].boat.size();
            rfFG->AddAnchorFactor(lpd.s0, anum0, lpd.s0time, lpd.s1, anum1, lpd.s1time, lpd.GetTFP0ToP1F0(), noise);
//            rfFG->AddCustomBTWNFactor(lpd.s0, lpd.s0time, lpd.s1, lpd.s1time, lpd.GetTFP0ToP1F0(), noise);
        }
    }
}

void AnchoredMultiSessionOptimization::AddAllTheLandmarkTracks(){
    cout << "   adding the landmark tracks."<<endl;
    
    //add all the landmark tracks to the factor graph.
    for(int i=0; i<cached_landmarks.size(); i++){
        rfFG->ChangeLandmarkSet(i);
        AddLandmarkTracks(cached_landmarks[i]);
    }
}

std::vector<double> AnchoredMultiSessionOptimization::ComputeP1frame0(std::vector<double>& a0, std::vector<double>& a1, std::vector<double>& p1){
    gtsam::Pose3 ga0(gtsam::Rot3::ypr(a0[5],a0[4],a0[3]), gtsam::Point3(a0[0],a0[1],a0[2]));
    gtsam::Pose3 ga1(gtsam::Rot3::ypr(a1[5],a1[4],a1[3]), gtsam::Point3(a1[0],a1[1],a1[2]));
    gtsam::Pose3 gp1(gtsam::Rot3::ypr(p1[5],p1[4],p1[3]), gtsam::Point3(p1[0],p1[1],p1[2]));
    gtsam::Pose3 p1frame0 = ga0.inverse() * ga1 * gp1;
    return {p1frame0.x(), p1frame0.y(), p1frame0.z(), p1frame0.rotation().roll(), p1frame0.rotation().pitch(), p1frame0.rotation().yaw()};
}

double AnchoredMultiSessionOptimization::UpdateErrorAdaptive(bool firstiter) {
    double mult = 3;
    static vector<vector<double> > permerr;
    vector<unordered_map<int, double> > intra;
    vector<vector<vector<double> > > poses;
    vector<vector<vector<double> > > landmarks;
    
    for(int i=optstart; i<dates.size(); i++){
        int sidx = i-optstart;
        rfFG->ChangeLandmarkSet(sidx);
        poses.push_back(GTS.GetOptimizedTrajectory(i, 500));//POR[i].boat.size()));
        landmarks.push_back(GTS.GetOptimizedLandmarks(true));
        
        intra.push_back(unordered_map<int, double>());
        permerr.push_back(vector<double>(lpdi[sidx].localizations.size(),0));
    }
    
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
            if(lpd.s0time >= 500 || lpd.s1time >= 500) continue;
            
            vector<double> p1frame0 = ComputeP1frame0(poses[lpd.s0][POR[lpd.s0].boat.size()], poses[lpd.s1][POR[sidx].boat.size()], poses[sidx][lpd.s1time]);
            double inter_error = erfinter.InterSurveyErrorAtLocalization(lpd, p1frame0, landmarks, 0);
//            double inter_error = erfinter.InterSurveyErrorAtLocalization(lpd, poses[sidx][lpd.s1time], landmarks, optstart);
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
            
            double newval = std::max(3.0, std::max(std::max(inter_error, intra_errorS1), intra_errorS0));
            lpd_eval[sidx][j] = newval;
            
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
                if(!firstiter){
                    if(inlier) std::cout << "activated   ("<<lpd.s1<<"."<<lpd.s1time << ", "<<lpd.s0<<"."<<lpd.s0time<<")" << std::endl;
                    else       std::cout << "deactivated ("<<lpd.s1<<"."<<lpd.s1time << ", "<<lpd.s0<<"."<<lpd.s0time<<")" << std::endl;
                }
            }
            
            lpd_rerror[sidx][j] = 1;
            if(!inlier) {lpd_rerror[sidx][j] = -1; coutliers++;}
        }
        totchanges += nchanges;
        
        if(i > 0) {
            erfintraS0.PrintTots("intra ISCs connected to " + dates[i]);
            erfintraS1.PrintTots("intra " + dates[i]);
            erfinter.PrintTots("inter");
            std::cout << "number of (virtual) outliers: " << coutliers << std::endl;
        }
        inlier_ratio[sidx] = 1.0-(1.*coutliers/lpdi[sidx].localizations.size());
    }
    
    //returns avg num_changes.
    if(optstart==0) return (int) (totchanges/(dates.size()-1)); //convert to int to avoid unnecessary iterations due to very small changes
    return (int) (totchanges/(dates.size()-optstart));
}

void AnchoredMultiSessionOptimization::SaveResults() {
    
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
        
        /* The reprojectionerror.csv is used for the adaptive pruning.
         EvaluateSLAM es(_cam, _date, _results_dir);
         es.debug=true;
         es.ErrorForSurvey(_pftbase, true);
         es.PrintTots(); */
        
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

void AnchoredMultiSessionOptimization::IterativeMerge() {
    time_t beginning,optstart,end;
    time (&beginning);
    LocalizePose lp(_cam);
    double last_nchanges = 10000000000;
    double avg_nchanges= last_nchanges - 1;
    for(int i=0; avg_nchanges>0 && last_nchanges>avg_nchanges && i<MAX_ITERATIONS; i++) {
        rfFG->Clear();
        
        if(i>0) std::cout << "Merging Surveys. Iteration " << i << " of " << MAX_ITERATIONS << ", nchanges in last iteration: " << avg_nchanges << std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        ConstructFactorGraph();
        AddLocalizations(i==0);
        AddAllTheLandmarkTracks();
        
        std::cout << "  Optimizing..." << std::endl;
        time (&optstart);
        RunGTSAM();
        
        std::cout << "  Updating Error." << std::endl;
        last_nchanges = avg_nchanges;
        avg_nchanges = UpdateErrorAdaptive(i==0);
        
        std::cout << "  Finished." << std::endl;
        time (&end);
        double optruntime = difftime (end, optstart);
        double totruntime = difftime (end, beginning);
        printf("ITERATION %d. AVG Nchanges %lf. Run time (HH:MM:SS) optimization %s, total %s\n", i, avg_nchanges, FileParsing::formattime(optruntime).c_str(), FileParsing::formattime(totruntime).c_str());
        std::cout << "changes, history: " <<last_nchanges << " -> " << avg_nchanges << std::endl;
    }
    
    if(!dry_run){
        std::cout << "  Saving.." << std::endl;
        //std::cout << "  Saving.. IS DISABLED FOR THE ADAPTIVE CONSTRAINT TEST" << std::endl;
        SaveResults();
    }
}



















