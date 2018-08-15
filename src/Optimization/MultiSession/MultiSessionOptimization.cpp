/*
 * MultiSessionOptimization.cpp
 *
 *  Created on: ~June 28, 2017
 *      Author: shane
 */


#include "MultiSessionOptimization.hpp"

#include <random>
#include <string>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/SaveOptimizationResults.h>
#include <Optimization/SingleSession/EvaluateSLAM.h>
#include "Optimization/SingleSession/GTSamInterface.h"

#include "RFlowOptimization/LocalizePose.hpp"
#include "RFlowOptimization/EvaluateRFlow.hpp"
#include "RFlowOptimization/HopcountLog.hpp"
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/LandmarkTrack.h>

using namespace std;

MultiSessionOptimization::MultiSessionOptimization(Camera& cam, std::string map_dir, std::string pftbase, std::string date, double plt):
_map_dir(map_dir), _pftbase(pftbase),
SurveyOptimizer(cam, rfFG, date, map_dir, false) {
    rfFG = new RFlowFactorGraph();
    FG = rfFG;
    
    std::cout << "Multi-Session Optimization up to " << date << std::endl;
    
    std::cout << "  Initializing.."<<std::endl;
    IdentifyOptimizationDates();
    percent_of_tracks = plt;
    Initialize();
    rfFG->SetLandmarkDeviation(3.0); //must be *after* initialize();
}

void MultiSessionOptimization::IdentifyOptimizationDates(){
    /*The dates in datetable*/
    cout << "   parsing the optimization results."<<endl;
    
    dates = FileParsing::ListDirsInDir(_map_dir);
    for(int i=0; i<dates.size(); i++){
        ParseOptimizationResults datePOR(_map_dir, dates[i]);
        POR.push_back(datePOR);
        if(_date.compare(dates[i])==0) break;
//        if(i>0 && !HopcountLog::LogExists(_map_dir, dates[i])) break; //if I need to re-run, won't this method fail?
    }
    
    optstart = std::max(((int) POR.size()-K), 0);
    dates = std::vector<string>(dates.begin(), dates.begin()+POR.size());
}

void MultiSessionOptimization::Initialize() {
    SurveyOptimizer::Initialize();
     /*Load the lpd data for each one.*/
    cout << "   loading the localizations."<<endl;
    cache_landmarks = true;
    for(int i=optstart; i<dates.size(); i++){
        LPDInterface lint;
        std::cout <<  dates[i] << ": ";
        int nloaded = lint.LoadLocalizations(_map_dir + dates[i], dates);
        lpdi.push_back(lint);
        permerr.push_back(vector<double>(lint.localizations.size(),0));
        
        /*
        if(i > 0 && nloaded < POR[i].boat.size()*0.01) {
            std::cout<<"RFlowSurveyOptimizer Warning: There are too few localizations for " << dates[i] <<". Optimizing a set that was previously optimized."<<std::endl;
            POR.erase(POR.end()-1);
            optstart = std::max(((int) POR.size()-K-1), 0);
            dates.erase(dates.end()-1);
            i--;
            continue;
        }*/
        
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
        lpd_sum.push_back(vector<double>(rerror_set.size(), 0.0));
        
        int ninliers = 0;
        for(int i=0; i<rerror_set.size(); i++)
            ninliers += (rerror_set[i]==1)?1:0;
        outliers.push_back(0);
    }
    
    //an adaptive threshold for the update step.
    for(int survey=optstart; survey<dates.size(); survey++){
        for(int j=0; j<POR[survey].boat.size(); j++){
            rerrs[survey-optstart][j] = rerrs[survey-optstart][j]*update_mult_factor;
            if(rerrs[survey-optstart][j] < 0) {
                std::cout << "MultiSessionOptimization::UpdateError() Something went wrong with the Rerror file. Got negative rerror."<<std::endl;
                exit(1);
            } else if (rerrs[survey-optstart][j] < 6) { //this occurs at places in the rerr vector that are zero.
                rerrs[survey-optstart][j] = AverageRerror[survey-optstart]*update_mult_factor;
            }
        }
    }
    
    heights = std::vector<double>(dates.size()-optstart, 0);
    BuildLandmarkSet();
}

void MultiSessionOptimization::SetHeight(gtsam::Pose3& traj, double z){
    gtsam::Point3 t = traj.translation();
    gtsam::Rot3 r = traj.rotation();
    gtsam::Point3 newt(t.x(), t.y(), z);
    traj = gtsam::Pose3(r, newt);
}

void MultiSessionOptimization::BuildLandmarkSet() {
    for(int survey=optstart; survey<dates.size(); survey++) {
        cache_set = survey-optstart; //update the parent class variable
        rfFG->ChangeLandmarkSet(cache_set);
        
        for(int i=0; i<POR[survey].boat.size(); i++) {
            ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[survey], POR[survey].ftfilenos[i]);
            std::vector<gtsam::Point3> p3d = POR[survey].GetSubsetOf3DPoints(pftf.ids);
            pftf.ModifyFTFData(p3d);
            vector<LandmarkTrack> tracks = pftf.ProcessNewPoints(survey, i, active, percent_of_tracks);
            CacheLandmarks(tracks);
        }
        
        //add the rest of the landmarks
        CacheLandmarks(active);
        active.clear();
        
        //sort the cached landmarks by the pose order
        std::qsort(&cached_landmarks[survey][0], cached_landmarks[survey].size(), sizeof(LandmarkTrack), [](const void* a, const void* b) {
            const LandmarkTrack* arg1 = static_cast<const LandmarkTrack*>(a);
            const LandmarkTrack* arg2 = static_cast<const LandmarkTrack*>(b);
            
            if(arg1->camera_keys[0] < arg2->camera_keys[0]) return -1;
            else if(arg1->camera_keys[0] > arg2->camera_keys[0]) return 1;
            else if(arg1->camera_keys.size() < arg2->camera_keys.size()) return -1;
            else if(arg1->camera_keys.size() > arg2->camera_keys.size()) return 1;
            return 0;
        });
    }
}

void MultiSessionOptimization::ConstructFactorGraph() {
    cout << "   adding the surveys"<<endl;
    
    //add the latest K surveys to the graph
    for(int survey=optstart; survey<dates.size(); survey++){
        
        bool latestsurvey = survey == POR.size()-1;
        int sidx = survey - optstart;
        
        for(int i=0; i<POR[survey].boat.size(); i++) {
            gtsam::Pose3 traj = POR[survey].CameraPose(i);
            int lpdcur = lpdi[sidx].GetLPDIdx(i);
            if(latestsurvey && lpdcur >= 0 && lpd_rerror[sidx][lpdcur] >= 0) {
                traj = GTSamInterface::VectorToPose(lpdi[sidx].localizations[lpdcur].p1frame0);
            }
            
            rfFG->AddPose(survey, i, traj);
            GTS.InitializePose(rfFG->GetSymbol(survey, i), traj);
            
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

int MultiSessionOptimization::DateToIndex(std::string date){
    for(int i=0; i<dates.size(); i++){
        if(date.compare(dates[i])==0) return i;
    }
    std::cout << "MultiSessionOptimization::DateToIndex() Error. Session not found. " << std::endl;
    exit(-1);
    return -1;
}

void MultiSessionOptimization::AddLocalizations(bool firstiter){
    cout << "   adding the localizations."<<endl;
    for(int i=0; i<lpdi.size(); i++) {
        for(int j=0; j<lpdi[i].localizations.size(); j++) {
            if(lpd_rerror[i][j] < 0) continue;
            LocalizedPoseData& lpd = lpdi[i].localizations[j];
            if(DateToIndex(lpd.date0) < optstart) { //add localization factors for locked-in surveys
                std::vector<gtsam::Point3> p3d0 = POR[DateToIndex(lpd.date0)].GetSubsetOf3DPoints(lpd.pids);
                rfFG->AddLocalizationFactors(_cam.GetGTSAMCam(), DateToIndex(lpd.date1), lpd.s1time, p3d0, lpd.p2d1, lpd.rerrorp);
            } else {
                //rfFG->AddBTWNFactor(DateToIndex(lpd.date0), lpd.s0time, DateToIndex(lpd.date1), lpd.s1time, lpd.GetTFP0ToP1F0(), true);
                double noise = pow(2, lpd_eval[i][j]/3.0) * 0.0001;
                rfFG->AddCustomBTWNFactor(DateToIndex(lpd.date0), lpd.s0time, DateToIndex(lpd.date1), lpd.s1time, lpd.GetTFP0ToP1F0(), noise);
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
        double sumz = 0;
        for(int j=0; j<poses[i].size(); j++){
            sumz += poses[i][j][5];
        }
        heights[i] = sumz/poses[i].size();
        std::cout << i << ": avg height: " << heights[i]<<std::endl;
    }
}

double MultiSessionOptimization::GetError(EvaluateRFlow& erflow, std::vector<double>& pose, std::vector<std::vector<double> >& landmarks, std::vector<LandmarkTrack>& cached_landmarks, int surveyTIME, std::unordered_map<int, double>& resultcache){
    
    return erflow.OnlineRError(cached_landmarks, surveyTIME, pose, landmarks);
//    auto search = resultcache.find(surveyTIME);
//    if(search != resultcache.end()) {
//        std::cout << "result: " << (double) search->second << " " << std::endl;
//        return search->second;
//    } else {
//        resultcache[surveyTIME] = erflow.OnlineRError(cached_landmarks, surveyTIME, pose, landmarks);
//    }
//    return resultcache[surveyTIME];
}

std::vector<bool> MultiSessionOptimization::LPDInlierTest(int s, int l, double LPD_RERROR_THRESHOLD, vector<double>& error){
    //double errS0, double errS1, double errISC0, double errISC1
    bool changed = false;
    bool inlier = true;
    if(std::isnan(error[2]) ||
       error[2] > LPD_RERROR_THRESHOLD) inlier = false;
    if(std::isnan(error[3]) ||
       error[3] > LPD_RERROR_THRESHOLD) inlier = false;
    if(std::isnan(error[0]) ||
       std::isnan(error[1])) inlier = false;
    if(error[0] > LPD_RERROR_THRESHOLD || //this threshold corresponds to s1, but it's inessential to change it.
       error[1] > LPD_RERROR_THRESHOLD) permerr[s][l] = 1;
    if(permerr[s][l] > 0) inlier = false;
    
    lpd_eval[s][l] = std::max(3.0, (std::max(error[0], error[1]), std::max(error[2], error[3])));
    for(int i=0; i<error.size(); i++) {
        if(i==0) lpd_sum[s][l] = error[i];
        else lpd_sum[s][l] += error[i];
    }
    lpd_sum[s][l] = error[2];
    
    if(lpd_rerror[s][l] == 0 ||
       (inlier && lpd_rerror[s][l] < 0) ||
       (!inlier && lpd_rerror[s][l] > 0)) {
        changed = true;
    }
    
    if(inlier) {lpd_rerror[s][l] = 1;}
    else lpd_rerror[s][l] = -1;
    
    return {inlier, changed};
}

int MultiSessionOptimization::EvaluateLPD(vector<vector<vector<double> > >& poseresult, std::vector<std::vector<std::vector<double> > >& landmarks, int s, int j, vector<EvaluateRFlow*> perf, vector<unordered_map<int, double> >& errorcache){
    LocalizedPoseData& l = lpdi[s].localizations[j];
    
    std::vector<double> p0 = GTS.MAPPoseEstimate(rfFG->GetSymbol(DateToIndex(l.date0), l.s0time));
    std::vector<double> p1 = GTS.MAPPoseEstimate(rfFG->GetSymbol(DateToIndex(l.date1), l.s1time));
    
    vector<double> error(4, 0);
    error[0] = GetError(*perf[0], p0, landmarks[DateToIndex(l.date0)], cached_landmarks[DateToIndex(l.date0)], l.s0time, errorcache[DateToIndex(l.date0)]);
    error[1] = GetError(*perf[1], p1, landmarks[DateToIndex(l.date1)], cached_landmarks[DateToIndex(l.date1)], l.s1time, errorcache[DateToIndex(l.date1)]);
    error[2] = perf[2]->InterSurveyErrorAtLocalization(p1, landmarks[DateToIndex(l.date0)], l.p2d1, l.pids, l.rerrorp);
    if(bothinter) error[3] = perf[2]->InterSurveyErrorAtLocalization(p0, landmarks[DateToIndex(l.date1)], l.b2d0, l.bids, l.rerrorb);
    
    std::vector<bool> result = LPDInlierTest(s, j, rerrs[DateToIndex(l.date1)][l.s1time], error);
    //these aren't necessary, given whatever new evaluation metric is used.
    //    coutliers += result[0]?0:1;
    //    nchanges += result[1]?1:0;
    if(result[0]) { //store the values
        poseresult[DateToIndex(l.date0)][l.s0time] = p0; //backward
        poseresult[DateToIndex(l.date1)][l.s1time] = p1; //forward
    } else outliers[s]++; //TODO: although, for the directional one this isn't counting correctly, I think.
    if(result[1]) return 1;
    return 0;
}

void MultiSessionOptimization::PrintConvergenceStats(int s, const vector<EvaluateRFlow*> perf, int coutliers){
    std::cout << " " << dates[s] << std::endl;
    perf[0]->PrintTots("  connected intra", true);
    perf[1]->PrintTots("  intra          ", true);
    perf[2]->PrintTots("  inter          ", true);
    if(coutliers>=0) std::cout << "number of (virtual) outliers: " << coutliers << std::endl;
}

double MultiSessionOptimization::UpdateErrorAdaptive(bool firstiter) {
    vector<unordered_map<int, double> > errorcache(dates.size());
    vector<vector<vector<double> > > poses;
    vector<vector<vector<double> > > landmarks;
    
    for(int i=0; i<dates.size(); i++){
        rfFG->ChangeLandmarkSet(i);
        poses.push_back(GTS.GetOptimizedTrajectory(i, POR[i].boat.size()));
        landmarks.push_back(GTS.GetOptimizedLandmarks(true));
    }
    
    double totchanges = 0;
    for(int i=0; i<dates.size(); i++){
        vector<EvaluateRFlow> erf = { EvaluateRFlow(_cam, "-", _map_dir),
            EvaluateRFlow(_cam, dates[i], _map_dir),
            EvaluateRFlow(_cam, dates[i], _map_dir)};
        vector<EvaluateRFlow*> perf = {&erf[0], &erf[1], &erf[2]};
        
        for(int j=0; j<lpdi[i].localizations.size(); j++)
            totchanges += EvaluateLPD(poses, landmarks, i, j, perf, errorcache);
        
//        if(i > 0)
//            PrintConvergenceStats(i, perf, outliers[i]);
    }
    
    //returns avg num_changes.
    if(optstart==0) return (int) (totchanges/(dates.size()-1)); //convert to int to avoid unnecessary iterations due to very small changes
    return (int) (totchanges/(dates.size()-optstart));
}

bool MultiSessionOptimization::CheckSave(){
    for(int survey=1; survey<outliers.size(); survey++) {
        double inlier_ratio = 1.0-(1.*outliers[survey]/lpdi[survey].localizations.size());
        if(inlier_ratio < 0.6){
            std::cout << "  Save disabled due to the inlier/outlier ratio for " << dates[survey] << " with ratio " << inlier_ratio << std::endl;
            return false;
        }
    }
    return true;
}

void MultiSessionOptimization::SaveResults() {
    if(!CheckSave()) return;
    
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
        EvaluateSLAM es(_cam, _date, _map_dir);
        es.debug=true;
        es.ErrorForSurvey(_pftbase, true);
        es.PrintTots(); */
        
        EvaluateRFlow erfinter(_cam, dates[i], _map_dir);
        vector<vector<double> > poses = GTS.GetOptimizedTrajectory(i, POR[i].boat.size());
        if(i==0) continue;
        vector<double> inter_error(lpdi[sidx].localizations.size(), 0);
        for(int j=0; j<lpdi[sidx].localizations.size(); j++) {
            LocalizedPoseData& l = lpdi[sidx].localizations[j];
            inter_error[j] = erfinter.InterSurveyErrorAtLocalization(poses[l.s1time], landmarks[DateToIndex(l.date0)], l.p2d1, l.pids, l.rerrorp);
//            inter_error[j] = erfinter.InterSurveyErrorAtLocalization(lpd, poses[lpd.s1time], landmarks, optstart);
        }
        erfinter.SaveEvaluation(inter_error, "/postlocalizationerror.csv");
        erfinter.VisualizeDivergenceFromLocalizations(lpdi[sidx].localizations, lpd_rerror[sidx]);
        
        HopcountLog hlog(_map_dir);
        hlog.SaveLocalLog(dates[i], lpd_rerror[sidx].size(), lpdi[sidx].localizations, lpd_rerror[sidx]);
    }
}

void MultiSessionOptimization::Reset(){
    rfFG->Clear();
    outliers = vector<int>(dates.size(), 0);
}

std::vector<double> MultiSessionOptimization::InlierOutlierStats(bool compact) {
    vector<double> mean(dates.size(), 0);
    vector<double> dev(dates.size(), 0);
    vector<double> meani(dates.size(), 0);
    vector<double> devi(dates.size(), 0);
    vector<int> numo(dates.size(), 0);
    int allo=0;
    int sizes = 0;
    double allmean=0;
    double allmeani=0;
    double alldev=0;
    double alldevi=0;
    int ninliers = 0;
    for(int i=0; i<dates.size(); i++) {
        int count = 0;
        for(int j=0; j<lpd_sum[i].size(); j++){
            mean[i] += lpd_sum[i][j];
            if(lpd_rerror[i][j] >= 0){
                meani[i] += lpd_sum[i][j];
                count++;
            } else numo[i]++;
        }
        allmeani += meani[i];
        sizes += lpd_sum[i].size();
        allmean += mean[i];
        allo += numo[i];
        mean[i] = mean[i]/lpd_sum[i].size();
        meani[i] = meani[i]/count;
        ninliers += count;
        
        for(int j=0; j<lpd_sum[i].size(); j++) {
            dev[i] += pow(lpd_sum[i][j] - mean[i], 2);
            if(lpd_rerror[i][j] == 1) {
                devi[i] += pow(lpd_sum[i][j] - meani[i], 2);
            }
        }
        dev[i] = pow(dev[i]/(lpd_sum[i].size()-1), 0.5);
        devi[i] = pow(devi[i]/(count-1), 0.5);
    }
    allmean /= sizes;
    allmeani /= ninliers;
    
    int c=0, ci=0;
    for(int i=0; i<dates.size(); i++) {
        for(int j=0; j<lpd_sum[i].size(); j++) {
            alldev += pow(lpd_sum[i][j] - allmean, 2);
            c++;
            if(lpd_rerror[i][j] == 1) {
                alldevi += pow(lpd_sum[i][j] - allmeani, 2);
                ci++;
            }
        }
    }
    alldev = pow(alldev/(c-1), 0.5);
    alldevi = pow(alldevi/(ci-1), 0.5);
    
    std::cout << " " << allo << "/"<<sizes<<" outliers. All: " << allmean << " ("<<alldev<<") rerror (avg). " << allmeani << "("<<alldevi<<") rerror (inliers)."<< std::endl;
    if(compact) return {allmean, allmeani};
    for(int i=0; i<dates.size(); i++)
        std::cout << "  " << dates[i] << ": " << numo[i] << " outliers. All: (" << mean[i] << ", " << dev[i] << ")," << lpd_rerror[i].size()-numo[i] << " inliers: (" <<meani[i] << ", " << devi[i] <<")" << std::endl;
    return {allmean, allmeani};
}

void MultiSessionOptimization::IterativeMerge() {
    time_t beginning,optstart,optend,end;
    time (&beginning);
    
    double last_nchanges = 10000000000;
    double avg_nchanges= last_nchanges - 1;
    for(int i=0; avg_nchanges>0 && last_nchanges>avg_nchanges && i<MAX_ITERATIONS; i++) {
        Reset();
        
        std::cout << "ITERATION " << i << " of " << MAX_ITERATIONS << std::endl;
        std::cout << "(all dates)" <<std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        ConstructFactorGraph();
        AddLocalizations(i==0);
        AddAllTheLandmarkTracks();
        rfFG->PrintStats();
        
        std::cout << "  Optimizing..." << std::endl;
        time (&optstart);
        RunGTSAM();
        time (&optend);
        
        std::cout << "  Updating Error." << std::endl;
        last_nchanges = avg_nchanges;
        avg_nchanges = UpdateErrorAdaptive(i==0);
        InlierOutlierStats();
        
        std::cout << "  Finished." << std::endl;
        time (&end);
        double optruntime = difftime (optend, optstart);
        double updateruntime = difftime (end, optend);
        double totruntime = difftime (end, beginning);
        std::cout << " " <<last_nchanges << " -> " << avg_nchanges << " changes " << std::endl;
        printf("  %s total runtime. %s optimization, %s update\n", FileParsing::formattime(totruntime).c_str(), FileParsing::formattime(optruntime).c_str(), FileParsing::formattime(updateruntime).c_str());
    }
    
    InlierOutlierStats();
    
    if(!dry_run){
        std::cout << "  Saving.." << std::endl;
        //std::cout << "  Saving.. IS DISABLED FOR THE ADAPTIVE CONSTRAINT TEST" << std::endl;
        SaveResults();
    }
}




















