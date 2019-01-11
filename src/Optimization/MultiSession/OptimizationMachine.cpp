

#include "RFlowOptimization/LocalizedPoseData.hpp"
#include "RFlowOptimization/LocalizePose.hpp"

#include "Optimization/SingleSession/GTSamInterface.h"
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

void OptimizationMachine::toLogPoseChange(double * apc, double * aoc){
    avgposchange = apc;
    avgorientchange = aoc;
}

void OptimizationMachine::SetWeight(double weight){
    _weight = weight;
}

void OptimizationMachine::Reset(){
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

void OptimizationMachine::SetPercentOfLandmarks(double p) {
    percent_landmark_tracks = p;
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
    
    srand(std::time(0));
    for(int i=0; i<cached_landmarks->size(); i++) {
        if(rand()%100 > percent_landmark_tracks)
            continue;
        rfFG->AddLandmarkTrack(_cam.GetGTSAMCam(), (*cached_landmarks)[i]);
    }
}

void OptimizationMachine::AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, gtsam::Pose3 offset, double noise){
    gtsam::Pose3 base = GTSamInterface::VectorToPose(posesTimeT1[sISC][sTIME]);
    gtsam::Pose3 ptraj = base.compose(offset);
    rfFG->AddPosePrior(rfFG->GetSymbol(survey, surveyTIME), ptraj, noise);
}

int OptimizationMachine::AddDirectionalLocalization(int s, int j, int d){
    if(bFilter && (*lpd_rerror)[s][j] < 0) return 0;
    LocalizedPoseData& l = (*lpdi)[s].localizations[j];
    if(SessionToNum(l.date0) == -1 || SessionToNum(l.date1) == -1) return 0;
    double noise = 0.0001;
    if(d==Direction::BACKWARD) AddLocalization(SessionToNum(l.date0), l.s0time, SessionToNum(l.date1), l.s1time, l.GetTFP0ToP1F0(), noise);
    else AddLocalization(SessionToNum(l.date1), l.s1time, SessionToNum(l.date0), l.s0time, l.GetTFP0ToP1F0().inverse(), noise);
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

int OptimizationMachine::SessionToNum(std::string session){
    for(int i=0; i<sessiondates.size(); i++){
        if(session.compare(sessiondates[i])==0) return i;
    }
    return -1;
    //std::cout << "OptimizationMachine::SessionToNum() Error. Session not found. " << std::endl;
    //exit(-1);
    //return -1;
}

void OptimizationMachine::SessionDates(std::vector<std::string>& dates){
    sessiondates = dates;
}

int BinarySearch(std::vector<std::vector<double> >& arr, int val){
    if(arr.size()==0) return -1;
    int s = 0;
    int e = arr.size();
    while(e-s>1){
        int med = s + (e-s)/2;
        if(((int) arr[med][3]) > val) e = med;
        else if(((int) arr[med][3]) <val) s = med;
        else return med;
    }
    if(((int) arr[s][3]) == val) return s;
    return -1;
}

void * OptimizationMachine::Run() {
    thread_state = state::RUNNING;
    ConstructFactorGraph();
    
    AddLocalizations();
    
    GTS.SetIdentifier(originPOR->_date);
    GTS.RunBundleAdjustment();
    
    std::vector<std::vector<double> > posesUpdated = GTS.GetOptimizedTrajectory(survey, (*posesTimeT2)[survey].size());
    std::vector<double> posdif((*posesTimeT2)[survey].size(), 0);
    for(int i=0; i<posesUpdated.size(); i++)
    {
        posdif[i] = pow(pow(posesUpdated[i][0] - posesTimeT1[survey][i][0], 2) +
                          pow(posesUpdated[i][1] - posesTimeT1[survey][i][1], 2) +
                          pow(posesUpdated[i][2] - posesTimeT1[survey][i][2], 2), 0.5);
        //TODO: correct this so the updated orientations wrap around pi correctly.  
        for(int j=0; j<6; ++j)
        {
            if(_weight < 1.0)
                (*posesTimeT2)[survey][i][j] = _weight*posesUpdated[i][j] + (1-_weight) * posesTimeT1[survey][i][j];
            else
                (*posesTimeT2)[survey][i][j] = posesUpdated[i][j];
        }
    }
    (*avgorientchange) = -1;
    std::sort(posdif.begin(), posdif.end());
    (*avgposchange) = posdif[posdif.size()/2];

    /*
    std::vector<std::vector<double> > landmarksUpdated = GTS.GetOptimizedLandmarks(true);
    (*landmarks)[survey].clear();
    for(int i=0; i<landmarksUpdated.size(); i++){
        int idx = BinarySearch((*landmarks)[survey], landmarksUpdated[i][3]);
        if(idx>=0){
            for(int j=0; j<3; j++){
                (*landmarks)[survey][idx][j] = weight * landmarksUpdated[i][j] + (1-weight) * (*landmarks)[survey][idx][j];
            }
        }
//        else {
//            for(int j=0; j<3; j++){
//                (*landmarks)[survey][idx][j] = weight * landmarksUpdated[i][j] + (1-weight) * (*landmarks)[survey][idx][j];
//            }
//        }
    }*/
    
//    (*posesTimeT2)[survey] = GTS.GetOptimizedTrajectory(survey, (*posesTimeT2)[survey].size());
    if(landmarks != nullptr)
        (*landmarks)[survey] = GTS.GetOptimizedLandmarks(true);
    
    thread_state = state::FINISHED;
    return (void *) NULL;
}

void OptimizationMachine::LogResults() { }

void OptimizationMachine::FilterBad(bool filter){
    bFilter = filter;
}
















































