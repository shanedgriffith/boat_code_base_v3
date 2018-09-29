#include <FileParsing/FileParsing.hpp>


//const std::string LocalizeSessionToSet::locmapdir = "../localized_maps/";

LocalizeSessionToSet(Camera& cam, std::string ref_map_dir, std::string loc_map_dir, std::string date, std::string pftbase, double percent_of_tracks = 100.0):
SurveyOptimizer(cam, rfFG, date, loc_map_dir, false),
    _cam(cam), _ref_map_dir(ref_map_dir), _loc_map_dir(loc_map_dir), _date(date), _pftbase(pftbase)
{
    rfFG = new RFlowFactorGraph();
    FG = rfFG;
}


void LocalizeSessionToSet::LoadFTF(ParseOptimizationResults& datePOR) {
    std::vector<LandmarkTrack> clset;
    cached_landmarks.push_back(clset);
    
    for(int i=0; i<datePOR.boat.size(); ++i) {
        ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + date, datePOR.ftfilenos[i]);
        std::vector<gtsam::Point3> p3d = datePOR.GetSubsetOf3DPoints(pftf.ids);
        pftf.ModifyFTFData(p3d);
        vector<LandmarkTrack> tracks = pftf.ProcessNewPoints(0, i, active, percent_of_tracks);
        CacheLandmarks(tracks);
    }
    
    //add the rest of the landmarks
    CacheLandmarks(active);
    active.clear();
    
    //sort the cached landmarks by the pose order
    std::qsort(&cached_landmarks[0][0], cached_landmarks[0].size(), sizeof(LandmarkTrack), [](const void* a, const void* b) {
        const LandmarkTrack* arg1 = static_cast<const LandmarkTrack*>(a);
        const LandmarkTrack* arg2 = static_cast<const LandmarkTrack*>(b);
        
        if(arg1->camera_keys[0] < arg2->camera_keys[0]) return -1;
        else if(arg1->camera_keys[0] > arg2->camera_keys[0]) return 1;
        else if(arg1->camera_keys.size() < arg2->camera_keys.size()) return -1;
        else if(arg1->camera_keys.size() > arg2->camera_keys.size()) return 1;
        return 0;
    });
    
    cache_set = 0; //update the parent class variable
    rfFG->ChangeLandmarkSet(0);
}

void LocalizeSessionToSet::ConstructFactorGraph() {
    //changes:
    // variables connected to an isc are eliminated.
    //    cout << "   adding the survey"<<endl;
    
    LocalizePose lp(_cam);
    for(int i=0; i<originPOR->boat.size(); i++) {
        gtsam::Pose3 traj = originPOR->CameraPose(i);
        rfFG->AddPose(0, i, traj);
        GTS.InitializePose(rfFG->GetSymbol(0, i), traj);
        
        if(i>0) {
            gtsam::Pose3 last = originPOR->CameraPose(i-1);
            gtsam::Pose3 btwn = last.between(traj);
            //order matters; this has to be after the variables it depends on are initialized.
            rfFG->AddCustomBTWNFactor(0, i-1, 0, i, btwn, 0.01);
        }
    }
    
    for(int i=0; i<cached_landmarks.size(); i++)
        rfFG->AddLandmarkTrack(_cam.GetGTSAMCam(), cached_landmarks[i]);
}

int LocalizeSessionToSet::SessionToNum(std::string session){
    for(int i=0; i<dates.size(); i++){
        if(session.compare(dates[i])==0) return i;
    }
    return -1;
}

void LocalizeSessionToSet::AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, gtsam::Pose3 offset, double noise){
    gtsam::Pose3 base = GTSamInterface::VectorToPose(posesTimeT1[sISC][sTIME]);
    gtsam::Pose3 ptraj = base.compose(offset);
    rfFG->AddPosePrior(rfFG->GetSymbol(survey, surveyTIME), ptraj, noise);
}

void LocalizeSessionToSet::AddLocalizations(){
    double noise = 0.0001;
    for(int j=0; j<lpdi.localizations.size(); j++) {
        if(lpd_rerror[j] < 0) return 0;
        LocalizedPoseData& l = lpdi.localizations[j];
        AddLocalization(SessionToNum(l.date0), l.s0time, SessionToNum(l.date1), l.s1time, l.GetTFP0ToP1F0(), noise);
    }
}

void LocalizeSessionToSet::Run() {
    rfFG->Clear();
    rfFG->ChangeLandmarkSet(0);
    ConstructFactorGraph();
    
    AddLocalizations();
    
    GTS.SetIdentifier(date);
    GTS.RunBundleAdjustment();
    
    poses = GTS.GetOptimizedTrajectory(0, originPOR.boat.size());
    landmarks = GTS.GetOptimizedLandmarks(true);
}

std::vector<bool> LocalizeSessionToSet::LPDInlierTest(int l, double LPD_RERROR_THRESHOLD, vector<double>& error){
    //double errS0, double errS1, double errISC0, double errISC1
    bool changed = false;
    bool inlier = true;
    if(std::isnan(error[1]) ||
       error[1] > LPD_RERROR_THRESHOLD) inlier = false;
    if(std::isnan(error[0])) inlier = false;
    if(error[0] > LPD_RERROR_THRESHOLD) permerr[l] = 1;
    if(permerr[l] > 0) inlier = false;
    
    lpd_eval[l] = std::max(3.0, std::max(error[0], error[1]));
    lpd_sum[l] = error[1];
    
    if(lpd_rerror[l] == 0 ||
       (inlier && lpd_rerror[l] < 0) ||
       (!inlier && lpd_rerror[l] > 0)) {
        changed = true;
    }
    
    if(inlier) {lpd_rerror[l] = 1;}
    else lpd_rerror[l] = -1;
    
    return {inlier, changed};
}

int LocalizeSessionToSet::EvaluateLPD(int j){
    
    LocalizedPoseData& l = lpdi.localizations[j];
    std::vector<double> p1 = poseresult[l.s1time];
    
    if(l.date0 == date) {
        std::cout << "change the order" << std::endl;
        exit(-1);
    }
    
    EvaluateRFlow erf(_cam, date, _loc_map_dir);
    
    vector<double> error(2, 0);
    error[0] = erf.OnlineRError(cached_landmarks[0], l.s1time, p1, landmarks);
    error[1] = erf.MeasureReprojectionError(p1, l.p2d1, l.p3d0);
    std::vector<bool> result = LPDInlierTest(j, rerrs[l.s1time], error);
    inter_error[j] = error[1];

    if(!result[0]) outliers++;
    if(result[1]) return 1;
    return 0;
}

double LocalizeSessionToSet::UpdateErrorAdaptive() {
    outliers = 0;
    double totchanges = 0;
    for(int j=0; j<lpdi.localizations.size(); j++)
        totchanges += EvaluateLPD(j);
    return totchanges;
}

void LocalizeSessionToSet::Initialize() {
    
    dates = FileParsing::ListDirsInDir(_ref_map_dir);
    for(int i=0; i<dates.size(); i++){
        ParseOptimizationResults datePOR(_ref_map_dir, dates[i]);
        POR.push_back(datePOR);
    }
    
    originPOR = ParseOptimizationResults(loc_map_dir, date);
    LoadFTF(originPOR);
    
    LPDInterface lint;
    std::cout <<  date << ": ";
    int nloaded = lint.LoadLocalizations(_loc_map_dir, date);
    lpdi.push_back(lint);
    
    HopcountLog hlog(loc_map_dir);
    lpd_rerror = hlog.LoadPriorRerror(date, nloaded);
    lpd_eval = vector<double>(nloaded, 3.0);
    lpd_sum = vector<double>(nloaded, 0.0);
    permerr = vector<double>(nloaded, 0);
    inter_error = vector<double>(nloaded, 0);
    
    EvaluateSLAM ESlam(_cam, date, loc_map_dir);
    rerrs = ESlam.LoadRerrorFile();
    double avg = ESlam.GetAverageRerror(rerrs);
    
    for(int j=0; j<originPOR.boat.size(); j++){
        rerrs[j] = rerr[j]*update_mult_factor;
        if(rerrs[j] < 0) {
            std::cout << "MC,L: MultiSessionOptimization::UpdateError() Something went wrong with the Rerror file. Got negative rerror."<<std::endl;
            exit(1);
        } else if (rerrs[j] < 6) { //this occurs at places in the rerr vector that are zero.
            rerrs[j] = avg*update_mult_factor;
        }
    }
}

bool LocalizeSessionToSet::CheckSave() {
    double inlier_ratio = 1.0-(1.*outliers/lpdi.localizations.size());
    if(inlier_ratio < 0.6){
        std::cout << "  Save disabled due to the inlier/outlier ratio for " << date << " with ratio " << inlier_ratio << std::endl;
        return false;
    }
    return true;
}

void LocalizeSessionToSet::SaveResults() {
    if(!CheckSave()) return;
    
    SaveOptimizationResults curSOR(_loc_map_dir + date);
    vector<vector<double> > vs;
    curSOR.SetSaveStatus();
    curSOR.SetDrawMap();
    curSOR.PlotAndSaveCurrentEstimate(landmarks, poses, vs, {});
    
    EvaluateRFlow erfinter(_cam, date, _loc_map_dir);
    erfinter.SaveEvaluation(inter_error, "/postlocalizationerror.csv");
    erfinter.VisualizeDivergenceFromLocalizations(lpdi.localizations, lpd_rerror);
    
    HopcountLog hlog(_loc_map_dir);
    hlog.SaveLocalLog(date, lpd_rerror.size(), lpdi.localizations, lpd_rerror);
}

void LocalizeSessionToSet::InlierOutlierStats() {
    std::cout << "  " << date << ": " << outliers << " outliers. of " << lpdi.localizations.size() << std::endl;
}

void LocalizeSessionToSet::LocalizeSession() {
    std::cout << "LOCALIZING A SESSION " << std::endl;
    time_t beginning,optstart,optend,end;
    time (&beginning);
    
    Initialize();
    
    for(int i=0; i<5; i++) {
        time (&optstart);
        Run();
        time (&optend);
        UpdateErrorIterative();
        InlierOutlierStats();
        time (&end);
        double optruntime = difftime (optend, optstart);
        double updateruntime = difftime (end, optend);
        double totruntime = difftime (end, beginning);
        printf("  %s total runtime. %s optimization, %s update\n", FileParsing::formattime(totruntime).c_str(), FileParsing::formattime(optruntime).c_str(), FileParsing::formattime(updateruntime).c_str());
    }
    
    //SaveResults();
}
























