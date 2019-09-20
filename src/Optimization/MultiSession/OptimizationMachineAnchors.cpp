

#include "RFlowOptimization/LocalizedPoseData.hpp"
#include "RFlowOptimization/LocalizePose.hpp"

#include "Optimization/MultiSession/OneSessionAnchor.h"
#include "Optimization/SingleSession/GTSAMInterface.h"
#include "OptimizationMachineAnchors.hpp"

using namespace std;

void OptimizationMachineAnchors::Setup(std::vector<std::vector<std::vector<double> > > * poses_,
           std::vector<std::vector<std::vector<double> > > * landmarks_,
           std::vector<std::vector<double>> * lpd_rerror_,
           ParseOptimizationResults * originPOR_,
           std::vector<LandmarkTrack> * cached_landmarks_,
           std::vector<std::vector<int> > * forwardLMap_,
           std::vector<LPDInterface> * lpdi_,
           int survey_){
    thread_state = state::LOCKED;
    
    posesTimeT1 = *poses_;
    posesTimeT2 = poses_;
    landmarks = landmarks_;
    lpd_rerror = lpd_rerror_;
    originPOR = originPOR_;
    cached_landmarks = cached_landmarks_;
    forwardLMap = forwardLMap_;
    lpdi = lpdi_;
    survey = survey_;
}

void OptimizationMachineAnchors::toLogPoseChange(double * apc, double * aoc){
    avgposchange = apc;
    avgorientchange = aoc;
}

void OptimizationMachineAnchors::SetWeight(double weight){
    _weight = weight;
}

void OptimizationMachineAnchors::Reset(){
    rfFG->Clear();
    posesTimeT1 = {};
    posesTimeT2 = nullptr;
    landmarks = nullptr;
    lpd_rerror = nullptr;
    originPOR = nullptr;
    avgposchange = nullptr;
    avgorientchange = nullptr;
    cached_landmarks = nullptr;
    forwardLMap = nullptr;
    lpdi = nullptr;
    _weight = 0.9;//2.0/3;
    survey = -1;
    thread_state = state::OPEN;
}

void OptimizationMachineAnchors::SetPercentOfLandmarks(double p) {
    percent_landmark_tracks = p;
}

void OptimizationMachineAnchors::ConstructFactorGraph() {
    LocalizePose lp(_cam);
    for(int i=0; i<originPOR->boat.size(); i++) {
        gtsam::Pose3 traj = originPOR->CameraPose(i);
        rfFG->AddPose(survey, i, traj);
        GTS.InitializePose(rfFG->GetSymbol(survey, i), traj);
        
        if(i>0) {
            gtsam::Pose3 last = originPOR->CameraPose(i-1);
            gtsam::Pose3 btwn = last.between(traj);
            rfFG->AddCustomBTWNFactor(survey, i-1, survey, i, btwn, 0.01);
        }
    }
    
    srand(std::time(0));
    for(int i=0; i<cached_landmarks->size(); i++) {
        if(rand()%100 > percent_landmark_tracks)
            continue;
        rfFG->AddLandmarkTrack(_cam.GetGTSAMCam(), (*cached_landmarks)[i]);
    }
}

//void OptimizationMachineAnchors::SetFirstAnchorStrongPrior()
//{
//    
//}

void OptimizationMachineAnchors::AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, gtsam::Pose3 offset, double noise){
    gtsam::Pose3 base = GTSAMInterface::VectorToPose(posesTimeT1[sISC][sTIME]);
    gtsam::Pose3 ptraj = base.compose(offset);
    rfFG->AddPosePrior(rfFG->GetSymbol(survey, surveyTIME), ptraj, noise);
}

int OptimizationMachineAnchors::AddDirectionalLocalization(int s, int j, int d){
    if(bFilter && (*lpd_rerror)[s][j] < 0) return 0;
    LocalizedPoseData& l = (*lpdi)[s].localizations[j];
    if(SessionToNum(l.date0) == -1 || SessionToNum(l.date1) == -1) return 0;
    double noise = 0.0001;
    if(d==Direction::BACKWARD) AddLocalization(SessionToNum(l.date0), l.s0time, SessionToNum(l.date1), l.s1time, l.GetTFP0ToP1F0(), noise);
    else AddLocalization(SessionToNum(l.date1), l.s1time, SessionToNum(l.date0), l.s0time, l.GetTFP0ToP1F0().inverse(), noise);
    return 1;
}

void OptimizationMachineAnchors::AddLocalizations(){
    int cf=0, cb=0;
    for(int j=0; j<(*lpdi)[survey].localizations.size(); j++)
        cb += AddDirectionalLocalization(survey, j, Direction::BACKWARD);
    for(int i=0; i<forwardLMap->size(); i++)
        cf += AddDirectionalLocalization((*forwardLMap)[i][0], (*forwardLMap)[i][1], Direction::FORWARD);
}

int OptimizationMachineAnchors::SessionToNum(std::string session){
    for(int i=0; i<sessiondates.size(); i++){
        if(session.compare(sessiondates[i])==0) return i;
    }
    return -1;
}

void OptimizationMachineAnchors::SessionDates(std::vector<std::string>& dates){
    sessiondates = dates;
}

void * OptimizationMachineAnchors::Run() {
    thread_state = state::RUNNING;
    ConstructFactorGraph();
    
    AddLocalizations();
    
    GTS.SetIdentifier(originPOR->_date);
    GTS.RunBundleAdjustment();
    
    
    if(landmarks != nullptr)
        (*landmarks)[survey] = GTS.GetOptimizedLandmarks(true);
    vector<vector<double> > ls = GTS.GetOptimizedLandmarks();
    std::vector<std::vector<double> > posesUpdated = GTS.GetOptimizedTrajectory(survey, (*posesTimeT2)[survey].size());
    
//    vector<vector<double> > ts = GTS.GetOptimizedTrajectory('x', num_cameras_in_traj);
//    vector<vector<double> > vs = GTS.GetOptimizedTrajectory('v', num_cameras_in_traj);
//    SOR.PlotAndSaveCurrentEstimate(ls, ts, vs, drawscale);
    
    thread_state = state::FINISHED;
    return (void *) NULL;
}

void OptimizationMachineAnchors::LogResults() { }

void OptimizationMachineAnchors::FilterBad(bool filter){
    bFilter = filter;
}
















































