#include <ctime>
#include <time.h>

#include "MultiCascade.hpp"
#include "HopcountLog.hpp"

using namespace std;

MultiCascade::MultiCascade(Camera& cam, std::string results_dir, std::string pftbase, std::string date, int nthreads):
_origin_dir(results_dir + "origin/"),
MultiSessionOptimization(cam, results_dir, pftbase, date, 10){
    
    std::cout << "Multi-Cascade" << std::endl;
    
    bothinter = true;
    
    for(int survey=0; survey<dates.size(); survey++)
        forwardLMap.push_back(std::vector<std::vector<int> >());
    
    poses.clear();
    landmarks.clear();
    isc_rerror.clear();
    cached_landmarks.clear();
    
    for(int survey=0; survey<dates.size(); survey++) {
        for(int i=0; i<lpdi[survey].localizations.size(); i++)
            forwardLMap[lpdi[survey].localizations[i].s0].push_back({survey,i});
        ParseOptimizationResults oPOR(_origin_dir + dates[survey]);
        originPOR.push_back(oPOR);
        poses.push_back(POR[survey].boat);
        std::vector<std::vector<double> > lset = POR[survey].GetLandmarkSet();
        landmarks.push_back(lset);
        isc_rerror.push_back(vector<double>(lpdi[survey].localizations.size(), 0));
        std::vector<LandmarkTrack> clset;
        cached_landmarks.push_back(clset);
    }
    rfFG->SetLandmarkDeviation(3.0);
    BuildLandmarkSet();
    
    
    if(nthreads > 1) {
        for(int i=0; i<nthreads; i++){
            ws.push_back(new OptimizationMachine(_cam, results_dir));
            man.AddMachine(ws[i]);
        }
    }
    runset = vector<int>(dates.size(), 0);
}

void MultiCascade::BuildLandmarkSet() {
    for(int survey=0; survey<dates.size(); survey++) {
        cache_set = survey; //update the parent class variable
        
        for(int i=0; i<originPOR[survey].boat.size(); i++) {
            ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[survey], originPOR[survey].ftfilenos[i]);
            std::vector<gtsam::Point3> p3d = originPOR[survey].GetSubsetOf3DPoints(pftf.ids);
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
            
            if(arg1->key < arg2->key) return -1;
            else if(arg1->key > arg2->key) return 1;
            else if(arg1->camera_keys.size() < arg2->camera_keys.size()) return -1;
            else if(arg1->camera_keys.size() > arg2->camera_keys.size()) return 1;
            return 0;
        });
    }
}

std::vector<bool> MultiCascade::LPDInlierTest(int s, int l, double LPD_RERROR_THRESHOLD, vector<double>& error) {
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

int MultiCascade::EvaluateLPD(vector<vector<vector<double> > >& poseresult, std::vector<std::vector<std::vector<double> > >& landmarks, int s, int j, vector<EvaluateRFlow*> perf, vector<unordered_map<int, double> >& errorcache){
    LocalizedPoseData& l = lpdi[s].localizations[j];
    
    std::vector<double> p0 = poseresult[l.s0][l.s0time];
    std::vector<double> p1 = poseresult[l.s1][l.s1time];
    
    vector<double> error(4, 0);
    error[0] = GetError(*perf[0], p0, landmarks[l.s0], cached_landmarks[l.s0], l.s0time, errorcache[l.s0]);
    error[1] = GetError(*perf[1], p1, landmarks[l.s1], cached_landmarks[l.s1], l.s1time, errorcache[l.s1]);
    error[2] = perf[2]->InterSurveyErrorAtLocalization(p1, landmarks[l.s0], l.p2d1, l.pids, l.rerrorp);
    if(bothinter) error[3] = perf[2]->InterSurveyErrorAtLocalization(p0, landmarks[l.s1], l.b2d0, l.bids, l.rerrorb);
    
    std::vector<bool> result = LPDInlierTest(s, j, rerrs[l.s1][l.s1time], error);
    
    if(!result[0]) outliers[s]++;
    if(result[1]) return 1;
    return 0;
}

double MultiCascade::UpdateErrorIterative() {
    vector<unordered_map<int, double> > errorcache(dates.size());
    vector<int> nchanges(dates.size());
    
    double totchanges = 0;
    bool stop = false;
    for(int survey=1; survey<dates.size(); survey++){
        int nchangesurvey = 0;
        vector<EvaluateRFlow> erf = { EvaluateRFlow(_cam, "-", _map_dir),
            EvaluateRFlow(_cam, dates[survey], _map_dir),
            EvaluateRFlow(_cam, dates[survey], _map_dir)};
        vector<EvaluateRFlow*> perf = {&erf[0], &erf[1], &erf[2]};
        
        for(int j=0; j<lpdi[survey].localizations.size(); j++)
            nchanges[survey] += (int) EvaluateLPD(poses, landmarks, survey, j, perf, errorcache);
        
        totchanges += nchangesurvey;
    }
    
    double minchanges = 10000000;
    for(int i=0; i<runset.size(); i++){
        if(runset[i]==-1)break;
        if(runset[i]==0) continue;
        minchanges = std::min((double) minchanges, (double) nchanges[runset[i]]);
    }
    
    std::cout << " " <<totchanges << " total changes. " << minchanges << " fewest." << std::endl;
    
    return minchanges;
}

void MultiCascade::SaveResults() {
    if(!CheckSave()) return;
    for(int survey=0; survey<dates.size(); survey++){
        SaveOptimizationResults curSOR(_map_dir + dates[survey]);
        vector<vector<double>> vs;
        curSOR.SetSaveStatus();
        curSOR.SetDrawMap();
        curSOR.PlotAndSaveCurrentEstimate(landmarks[survey], poses[survey], vs, {});
        
        EvaluateRFlow erfinter(_cam, dates[survey], _map_dir);
        if(survey==0) continue;
        vector<double> inter_error(lpdi[survey].localizations.size(), 0);
        for(int j=0; j<lpdi[survey].localizations.size(); j++) {
            LocalizedPoseData& l = lpdi[survey].localizations[j];
            inter_error[j] = erfinter.InterSurveyErrorAtLocalization(poses[l.s1][l.s1time], landmarks[l.s0], l.p2d1, l.pids, l.rerrorp);
        }
        erfinter.SaveEvaluation(inter_error, "/postlocalizationerror.csv");
        erfinter.VisualizeDivergenceFromLocalizations(lpdi[survey].localizations, lpd_rerror[survey]);
        
        HopcountLog hlog(_map_dir);
        hlog.SaveLocalLog(dates[survey], lpd_rerror[survey].size(), lpdi[survey].localizations, lpd_rerror[survey]);
    }
}

void MultiCascade::RunIteration(bool firstiter){
    vector<int> snum(dates.size(), -1);
    vector<int> surveys(dates.size(), -1);
    for(int i=0; i<surveys.size(); i++)
        surveys[i] = i;
    
    int count = 0;
    srand(std::time(0));
    for(int i=0; i<man.NumberOfMachines() && i<dates.size(); i++){
        if(i==0 && man.NumberOfMachines()>1) {
            snum[i] = dates.size()-1;
        } else {
            if(firstiter) break;
            int randix = rand()%(dates.size()-i);
            snum[i] = surveys[randix];
            swap(surveys[randix], surveys[dates.size()-1-i]);
        }
        count++;
        std::cout << "    " << dates[snum[i]] << " at " << i << std::endl;
        
        int tidx = man.GetOpenMachine();
        ws[tidx]->Setup(&poses, &landmarks, &lpd_rerror, &(originPOR[snum[i]]), &(cached_landmarks[snum[i]]), &(forwardLMap[snum[i]]), &lpdi, snum[i]);
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    for(int i=0; i<snum.size(); i++)
        runset[i] = snum[i];
}

void MultiCascade::IterativeMerge() {
    time_t beginning,optstart,optend,end;
    time (&beginning);
    
    vector<vector<double>> rerror;
    //save the state so this update can be used as a data acquisition step.
    for(int survey=0; survey<dates.size(); survey++) {
        rerror.push_back(vector<double>(lpd_rerror[survey].size(), 0));
        for(int j=0; j<lpdi[survey].localizations.size(); j++)
            rerror[survey][j] = lpd_rerror[survey][j];
    }
    
    UpdateErrorIterative();
    InlierOutlierStats();
    
    for(int survey=0; survey<dates.size(); survey++) {
        for(int j=0; j<lpdi[survey].localizations.size(); j++)
            lpd_rerror[survey][j] = rerror[survey][j];
    }
    
    double changed = 1;
    std::cout << "(" << dates[dates.size()-1] << ")" <<std::endl;
    for(int i=0; changed>0 && i<MAX_CASC_ITERS; i++) {
        outliers = vector<int>(dates.size(), 0);
        
        std::cout << "ITERATION " << i << " of " << MAX_CASC_ITERS << std::endl;
        time (&optstart);
        std::cout << "  Optimizing..." << std::endl;
        RunIteration(i==0);
        time (&optend);
        
        std::cout << "  Updating Error." << std::endl;
        changed = UpdateErrorIterative();
        InlierOutlierStats();
        
        time (&end);
        double optruntime = difftime (optend, optstart);
        double updateruntime = difftime (end, optend);
        double totruntime = difftime (end, beginning);
        printf("  %s total runtime. %s optimization, %s update\n", FileParsing::formattime(totruntime).    c_str(), FileParsing::formattime(optruntime).c_str(), FileParsing::formattime(updateruntime).c_str());
    }
    
    if(!dry_run){
        std::cout << "  Saving.." << std::endl;
        SaveResults();
    }
}














































