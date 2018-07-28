#include "MultiSessionIterativeSmoothingAndRefinement.hpp"
#include "RFlowOptimization/LocalizePose.hpp"
#include "RFlowOptimization/EvaluateRFlow.hpp"
#include "RFlowOptimization/HopcountLog.hpp"
#include "Optimization/SingleSession/GTSamInterface.h"

using namespace std;

MultiSessionIterativeSmoothingAndRefinement::MultiSessionIterativeSmoothingAndRefinement(Camera& cam, std::string results_dir, std::string pftbase, std::string date):
_origin_dir(results_dir + "origin/"),
MultiSessionOptimization(cam, results_dir, pftbase, date, 10){

    std::cout << "Multi-Session Iterative Smoothing And Refinement" << std::endl;
    
    bothinter = true;
    
    for(int survey=0; survey<dates.size(); survey++)
        forwardLMap.push_back(std::vector<std::vector<int> >());
    
    poses.clear();
    landmarks.clear();
    isc_rerror.clear();
    cached_landmarks.clear();
    
    for(int survey=0; survey<dates.size(); survey++) {
        for(int i=0; i<lpdi[survey].localizations.size(); i++)
            forwardLMap[DateToIndex(lpdi[survey].localizations[i].date0)].push_back({survey,i});
        ParseOptimizationResults oPOR(_origin_dir, dates[survey]);
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
}

void MultiSessionIterativeSmoothingAndRefinement::BuildLandmarkSet() {
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

bool MultiSessionIterativeSmoothingAndRefinement::DifferentPoses(std::vector<double>& a, std::vector<double>& b){
    for(int i=0; i<6; i++)
        if(abs(a[i]-b[i])>0.02) return true;
    return false;
}

void MultiSessionIterativeSmoothingAndRefinement::ConstructFactorGraph(int survey) {
    //changes:
    // variables connected to an isc are eliminated.
    cout << "   adding the surveys"<<endl;
    
    //here, odom constraints are only added between activated pose variables.
    LocalizePose lp(_cam);
    for(int i=0; i<poses[survey].size(); i++) {
        gtsam::Pose3 traj = originPOR[survey].CameraPose(i);
        rfFG->AddPose(survey, i, traj);
        GTS.InitializePose(rfFG->GetSymbol(survey, i), traj);
        
        if(i>0) {
            gtsam::Pose3 last = originPOR[survey].CameraPose(i-1);
            gtsam::Pose3 btwn = last.between(traj);
            //order matters; this has to be after the variables it depends on are initialized.
            rfFG->AddCustomBTWNFactor(survey, i-1, survey, i, btwn, 0.01);
        }
    }
    
    AddLandmarkTracks(cached_landmarks[survey]);
}

int MultiSessionIterativeSmoothingAndRefinement::BinarySearchLandmark(std::vector<LandmarkTrack>& landmarks, int lkey){
    //binary search with repeats.
    if(lkey<0) return 0;
    if(lkey>=landmarks[landmarks.size()-1].key) return landmarks.size()-1;
    int top = landmarks.size();
    int bot = 0;
    int med;
    while(top-bot>1) {
        med = bot + (top - bot)/2;
        if(landmarks[med].key < lkey) bot = med;
        else if(landmarks[med].key > lkey) top = med;
        else break;
    }
    return med;
}

std::vector<gtsam::Point2> MultiSessionIterativeSmoothingAndRefinement::CoordsFromCachedSetAndIds(std::vector<LandmarkTrack>& landmarks, std::vector<int>& ids, int ckey){
    int cur = BinarySearchLandmark(landmarks, ids[0]);
    
    std::vector<gtsam::Point2> coords;
    for(int i=0; i<ids.size(); i++){
        while(landmarks[cur].key < ids[i]) cur++;
        if(ids[i] > landmarks[cur].key) coords.push_back(gtsam::Point2(-1,-1));
        else coords.push_back(landmarks[cur].GetCoord(ckey));
    }
    
    return coords;
}

int MultiSessionIterativeSmoothingAndRefinement::GetIndexOfFirstPoint(const std::vector<std::vector<double> >& landmarks, int id) {
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

vector<gtsam::Point3> MultiSessionIterativeSmoothingAndRefinement::GetSubsetOf3DPoints(const std::vector<std::vector<double> >& landmarks, const vector<int>& ids_subset) {
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

void MultiSessionIterativeSmoothingAndRefinement::AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, gtsam::Pose3 offset, double noise){
    gtsam::Pose3 base = GTSamInterface::VectorToPose(poses[sISC][sTIME]);
    gtsam::Pose3 ptraj = base.compose(offset);
    rfFG->AddPosePrior(rfFG->GetSymbol(survey, surveyTIME), ptraj, noise);
}

int MultiSessionIterativeSmoothingAndRefinement::AddDirectionalLocalization(int s, int j, int d){
    if(lpd_rerror[s][j] < 0) return 0;
    LocalizedPoseData& l = lpdi[s].localizations[j];
    double noise = 0.0001;// pow(2, lpd_eval[s][j]/3.0) * 0.0001;
    if(d==Direction::BACKWARD) AddLocalization(DateToIndex(l.date0), l.s0time, DateToIndex(l.date1), l.s1time, l.GetTFP0ToP1F0(), noise);
    else AddLocalization(DateToIndex(l.date1), l.s1time, DateToIndex(l.date0), l.s0time, l.GetTFP0ToP1F0().inverse(), noise);
    return 1;
}

void MultiSessionIterativeSmoothingAndRefinement::AddLocalizations(int survey){
    cout << "   adding the localizations."<<endl;
    
    int cf=0, cb=0;
    for(int j=0; j<lpdi[survey].localizations.size(); j++)
        cb += AddDirectionalLocalization(survey, j, Direction::BACKWARD);
    for(int i=0; i<forwardLMap[survey].size(); i++)
        cf += AddDirectionalLocalization(forwardLMap[survey][i][0], forwardLMap[survey][i][1], Direction::FORWARD);
    std::cout << "    Backward  " << cb << " of " << lpdi[survey].localizations.size() << std::endl;
    std::cout << "    Forward: " << cf << " of " << forwardLMap[survey].size()<< "."<< std::endl;
}

std::vector<bool> MultiSessionIterativeSmoothingAndRefinement::LPDInlierTest(int s, int l, double LPD_RERROR_THRESHOLD, vector<double>& error) {
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
    
    lpd_eval[s][l] = std::max(3.0, std::max(std::max(error[0], error[1]), std::max(error[2], error[3])));
    for(int i=0; i<error.size(); i++) {
        if(i==0) lpd_sum[s][l] = error[i];
        else lpd_sum[s][l] += error[i];
    }
    lpd_sum[s][l] = error[2];
    isc_rerror[s][l] = (error[2] + error[3])/2.0;
    
    if(lpd_rerror[s][l] == 0 ||
       (inlier && lpd_rerror[s][l] < 0) ||
       (!inlier && lpd_rerror[s][l] > 0)) {
        changed = true;
    }
    
    if(inlier) {lpd_rerror[s][l] = 1;}
    else lpd_rerror[s][l] = -1;
    
    return {inlier, changed};
}

int MultiSessionIterativeSmoothingAndRefinement::EvaluateLPD(vector<vector<vector<double> > >& poseresult, std::vector<std::vector<std::vector<double> > >& landmarks, int s, int j, vector<EvaluateRFlow*> perf, vector<unordered_map<int, double> >& errorcache){
    LocalizedPoseData& l = lpdi[s].localizations[j];
    
    std::vector<double> p0 = poseresult[DateToIndex(l.date0)][l.s0time];
    std::vector<double> p1 = poseresult[DateToIndex(l.date1)][l.s1time];
    
    vector<double> error(4, 0);
    error[0] = GetError(*perf[0], p0, landmarks[DateToIndex(l.date0)], cached_landmarks[DateToIndex(l.date0)], l.s0time, errorcache[DateToIndex(l.date0)]);
    error[1] = GetError(*perf[1], p1, landmarks[DateToIndex(l.date1)], cached_landmarks[DateToIndex(l.date1)], l.s1time, errorcache[DateToIndex(l.date1)]);
    error[2] = perf[2]->InterSurveyErrorAtLocalization(p1, landmarks[DateToIndex(l.date0)], l.p2d1, l.pids, l.rerrorp);
    if(bothinter) error[3] = perf[2]->InterSurveyErrorAtLocalization(p0, landmarks[DateToIndex(l.date1)], l.b2d0, l.bids, l.rerrorb);
    
    std::vector<bool> result = LPDInlierTest(s, j, rerrs[DateToIndex(l.date1)][l.s1time], error);
    //these aren't necessary, given whatever new evaluation metric is used.
    
    if(!result[0]) outliers[s]++; //TODO: although, for the directional one this isn't counting correctly, I think.
    if(result[1]) return 1;
    return 0;
}

double MultiSessionIterativeSmoothingAndRefinement::UpdateErrorIterative() {
    vector<unordered_map<int, double> > errorcache(dates.size());
    
    for(int survey=0; survey<dates.size(); survey++){
        if(GTS.HasResult(gtsam::Symbol(survey, 0))){
            poses[survey] = GTS.GetOptimizedTrajectory(survey, poses[survey].size());
            landmarks[survey] = GTS.GetOptimizedLandmarks(true);
        }
    }
    
    double totchanges = 0;
    for(int survey=0; survey<dates.size(); survey++){
        vector<EvaluateRFlow> erf = { EvaluateRFlow(_cam, "-", _map_dir),
            EvaluateRFlow(_cam, dates[survey], _map_dir),
            EvaluateRFlow(_cam, dates[survey], _map_dir)};
        vector<EvaluateRFlow*> perf = {&erf[0], &erf[1], &erf[2]};
        
        for(int j=0; j<lpdi[survey].localizations.size(); j++)
            totchanges += (int) EvaluateLPD(poses, landmarks, survey, j, perf, errorcache);
        
        //this necessary, since this iteration is over all the surveys?
//        for(int j=0; j<forwardLMap[survey].size(); j++)
//            EvaluateLPD(poses, landmarks, forwardLMap[survey][j][0], forwardLMap[survey][j][1], {perf[1], perf[0], perf[2]}, errorcache);
        
//        if(survey==0) continue;
//        PrintConvergenceStats(survey, perf);
    }
    
    return totchanges/dates.size();
}

int MultiSessionIterativeSmoothingAndRefinement::GetSurveyWithHighestISCError() {
    double maxe=-1;
    int index=0;
    for(int i=0; i<dates.size(); i++){
        double sum=0;
        for(int j=0; j<lpdi[i].localizations.size(); j++)
            sum += isc_rerror[i][j];
        for(int j=0; j<forwardLMap[i].size(); j++)
            sum += isc_rerror[forwardLMap[i][j][0]][forwardLMap[i][j][1]];
        sum /= (lpdi[i].localizations.size()+forwardLMap[i].size());
        std::cout << dates[i] << " Average ISC Error: " << sum << std::endl;
        if(sum >= maxe){
            maxe = sum;
            index = i;
        }
    }
    return index;
}

void MultiSessionIterativeSmoothingAndRefinement::SaveResults() {
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
            inter_error[j] = erfinter.InterSurveyErrorAtLocalization(poses[DateToIndex(l.date1)][l.s1time], landmarks[DateToIndex(l.date0)], l.p2d1, l.pids, l.rerrorp);
        }
        erfinter.SaveEvaluation(inter_error, "/postlocalizationerror.csv");
        erfinter.VisualizeDivergenceFromLocalizations(lpdi[survey].localizations, lpd_rerror[survey]);
        
        HopcountLog hlog(_map_dir);
        hlog.SaveLocalLog(dates[survey], lpd_rerror[survey].size(), lpdi[survey].localizations, lpd_rerror[survey]);
    }
}

void MultiSessionIterativeSmoothingAndRefinement::IterativeMerge() {
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
    vector<double> currerror = InlierOutlierStats(false);
    double sumbest = currerror[0] + currerror[1];
    
    for(int survey=0; survey<dates.size(); survey++) {
        for(int j=0; j<lpdi[survey].localizations.size(); j++) {
            lpd_rerror[survey][j] = rerror[survey][j];
            lpd_eval[survey][j] = 3.0;
        }
    }
    
    double last_nchanges = 10000000000;
    double avg_nchanges= last_nchanges - 1;
    int survey = dates.size()-1;
    int STOP_ITER = 5*(dates.size()-1);
    for(int i=0; avg_nchanges>0 && i<STOP_ITER; i++) {
        Reset();
        
        std::cout << "ITERATION " << i << " of " << STOP_ITER << std::endl;
        std::cout << "(" << dates[survey] << ")" <<std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        ConstructFactorGraph(survey);
        AddLocalizations(survey);
        
        std::cout << "  Optimizing..." << std::endl;
        time (&optstart);
        RunGTSAM();
        time (&optend);
        
        std::cout << "  Updating Error." << std::endl;
        avg_nchanges = UpdateErrorIterative();
        InlierOutlierStats();
        
        time (&end);
        double optruntime = difftime (optend, optstart);
        double updateruntime = difftime (end, optend);
        double totruntime = difftime (end, beginning);
        std::cout << " " <<last_nchanges << " -> " << avg_nchanges << " changes " << std::endl;
        printf("  %s total runtime. %s optimization, %s update\n", FileParsing::formattime(totruntime).    c_str(), FileParsing::formattime(optruntime).c_str(), FileParsing::formattime(updateruntime).c_str());
        survey = (survey+1) % dates.size();
    }
    
    if(!dry_run){
        std::cout << "  Saving.." << std::endl;
        SaveResults();
    }
    
    /*
    int survey = dates.size()-1;
    vector<double> bestrerror = {100000000, 100000000};
    double sumbest =
    int nnotimproved = 0;
    int i=0;
    for(; nnotimproved < ITER_STALE && i<MAX_ITERATIONS; i++) {
        Reset();
        
        std::cout << "ITERATION " << i << " of " << MAX_ITERATIONS << std::endl;
        std::cout << "(" << dates[survey] << ")" <<std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        //int survey = GetSurveyWithHighestISCError();
        
        ConstructFactorGraph(survey);
        AddLocalizations(survey);
        rfFG->PrintStats();
        
        std::cout << "  Optimizing..." << std::endl;
        time (&optstart);
        RunGTSAM();
        time (&optend);
        
        std::cout << "  Updating Error." << std::endl;
        last_nchanges = avg_nchanges;
        avg_nchanges = UpdateErrorIterative();
        currerror = InlierOutlierStats();
        double cs = currerror[0]+ currerror[1];
        if(cs < sumbest){
            bestrerror = currerror;
            bestlandmarks = landmarks;
            bestposes = poses;
            bestlpd_rerror = lpd_rerror;
            nnotimproved = 0;
        } else nnotimproved++;
        
        time (&end);
        double optruntime = difftime (optend, optstart);
        double updateruntime = difftime (end, optend);
        double totruntime = difftime (end, beginning);
        std::cout << " " <<last_nchanges << " -> " << avg_nchanges << " changes. Stale iterations: " << nnotimproved << " of " << ITER_STALE << std::endl;
        printf("  %s total runtime. %s optimization, %s update\n", FileParsing::formattime(totruntime).c_str(), FileParsing::formattime(optruntime).c_str(), FileParsing::formattime(updateruntime).c_str());
        survey = (survey+1) % dates.size();
    }
    
    if(!dry_run && nnotimproved < i){
        std::cout << "  Saving.." << std::endl;
        //std::cout << "  Saving.. IS DISABLED FOR THE ADAPTIVE CONSTRAINT TEST" << std::endl;
        SaveResults();
    }*/
}














































