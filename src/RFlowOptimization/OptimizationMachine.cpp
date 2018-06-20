

#include "LocalizedPoseData.hpp"
#include "LocalizePose.hpp"

#include "OptimizationMachine.hpp"

using namespace std;

void OptimizationMachine::Setup(std::vector<std::vector<std::vector<double> > > * poses_,
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

void OptimizationMachine::Reset(){
    rfFG->Clear();
    posesTimeT1 = {};
    posesTimeT2 = NULL;
    landmarks = NULL;
    lpd_rerror = NULL;
    originPOR = NULL;
    cached_landmarks = NULL;
    forwardLMap = NULL;
    lpdi = NULL;
    survey = -1;
    thread_state = state::OPEN;
}

void OptimizationMachine::ConstructFactorGraph() {
    //changes:
    // variables connected to an isc are eliminated.
//    cout << "   adding the survey"<<endl;
    
    LocalizePose lp(_cam);
    for(int i=0; i<originPOR->boat.size(); i++) {
        gtsam::Pose3 traj = originPOR->CameraPose(i);
        rfFG->AddPose(survey, i, traj);
        GTS.InitializePose(rfFG->GetSymbol(survey, i), traj);
        
        if(i>0) {
            gtsam::Pose3 last = originPOR->CameraPose(i-1);
            gtsam::Pose3 btwn = last.between(traj);
            //order matters; this has to be after the variables it depends on are initialized.
            rfFG->AddCustomBTWNFactor(survey, i-1, survey, i, btwn, 0.01);
        }
    }
    
    for(int i=0; i<cached_landmarks->size(); i++)
        rfFG->AddLandmarkTrack(_cam.GetGTSAMCam(), (*cached_landmarks)[i]);
}

void OptimizationMachine::AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, gtsam::Pose3 offset, double noise){
    LocalizePose lp(_cam);
    gtsam::Pose3 base = lp.VectorToPose(posesTimeT1[sISC][sTIME]);
    gtsam::Pose3 ptraj = base.compose(offset);
    rfFG->AddPosePrior(rfFG->GetSymbol(survey, surveyTIME), ptraj, noise);
}

int OptimizationMachine::AddDirectionalLocalization(int s, int j, int d){
    if((*lpd_rerror)[s][j] < 0) return 0;
    LocalizedPoseData& l = (*lpdi)[s].localizations[j];
    double noise = 0.0001;
    if(d==Direction::BACKWARD) AddLocalization(l.s0, l.s0time, l.s1, l.s1time, l.GetTFP0ToP1F0(), noise);
    else AddLocalization(l.s1, l.s1time, l.s0, l.s0time, l.GetTFP0ToP1F0().inverse(), noise);
    return 1;
}

void OptimizationMachine::AddLocalizations(){
//    cout << "   adding the localizations."<<endl;
    
    int cf=0, cb=0;
    for(int j=0; j<(*lpdi)[survey].localizations.size(); j++)
        cb += AddDirectionalLocalization(survey, j, Direction::BACKWARD);
    for(int i=0; i<forwardLMap->size(); i++)
        cf += AddDirectionalLocalization((*forwardLMap)[i][0], (*forwardLMap)[i][1], Direction::FORWARD);
//    std::cout << "    Backward  " << cb << " of " << (*lpdi)[survey].localizations.size() << std::endl;
//    std::cout << "    Forward: " << cf << " of " << forwardLMap->size()<< "."<< std::endl;
}

void * OptimizationMachine::Run() {
    thread_state = state::RUNNING;
    ConstructFactorGraph();
    
    AddLocalizations();
    
    GTS.RunBundleAdjustment();
    
    //update.
    //if this complains, then have this machine hold the data in temporary variables and copy them in the log.
    //I believe the references point to a dedicated area of the data structure. Maybe there's something to check in that though.
    (*posesTimeT2)[survey] = GTS.GetOptimizedTrajectory(survey, (*posesTimeT2)[survey].size());
    (*landmarks)[survey] = GTS.GetOptimizedLandmarks(true);
    
    thread_state = state::FINISHED;
    return (void *) NULL;
}

void OptimizationMachine::LogResults() { }


















































