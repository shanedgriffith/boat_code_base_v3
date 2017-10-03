/*
 * EfficientMultiSessionOptimization.cpp
 *
 *  Created on: Oct 3, 2017
 *      Author: shane
 */


#include "EfficientMultiSessionOptimization.hpp"
//
//#include <random>
//#include <string>
//#include <FileParsing/ParseFeatureTrackFile.h>
//#include <FileParsing/SaveOptimizationResults.h>
//#include <Optimization/EvaluateSLAM.h>
//
//#include "SFlowDREAM2RF.hpp"
//#include "LocalizePose.hpp"
//#include "EvaluateRFlow.hpp"
//#include "HopcountLog.hpp"
//#include <FileParsing/FileParsing.hpp>
//#include <DataTypes/LandmarkTrack.h>

using namespace std;


EfficientMultiSessionOptimization::EfficientMultiSessionOptimization(Camera& cam, std::string results_dir, std::string pftbase, std::string date):
MultiSessionOptimization(cam, results_dir, pftbase, date){}

void EfficientMultiSessionOptimization::ToggleLandmarksAtPose(int survey, int ckey, bool active) {
    //this would seem to have to be called in the update step.
    std::vector<LandmarkTrack> landmarks = cached_landmarks[survey];
    
    //all visual features between:
    //(the last of the set ending with ckey-1, the first of the set beginning with ckey+1)
    int beg = ParseFeatureTrackFile::FindLandmarkRange(landmarks, ckey+1, false);
    int end = ParseFeatureTrackFile::FindLandmarkRange(landmarks, ckey-1, true);
    //could test this by comparing to the number of visual feature tracks in the pfts
    
    int offset = 0;
    for(int i=beg; i<=end; i++) {
        while(landmarks[i].camera_keys[offset].index() < ckey) offset++;
        landmarks[i].constraint_on[offset]=active;
    }
}

void EfficientMultiSessionOptimization::TestLandmarkRange() {
    int count = 0;
    for(int survey=0; survey<dates.size(); survey++){
        std::vector<LandmarkTrack> landmarks = cached_landmarks[survey];
        for(int i=0; i<POR[survey].boat.size(); i++) {
            int beg = FindLandmarkRange(landmarks, ckey+1, false);
            int end = FindLandmarkRange(landmarks, ckey-1, true);
            
            ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[survey], POR[survey].ftfilenos[i]);
            pftf.ModifyFTFData(POR[survey].GetSubsetOf3DPoints(pftf.ids));
            if(pftf.ids.size() != end-beg+1){
                std::cout << "Found difference in the number of visual feature tracks at (" << survey << ", " << i <<"), expected " << pftf.ids.size()<<", got : " << end-beg+1 << std::endl;
                exit(-1);
            }
            count ++;
        }
    }
    std::cout << "All landmark ranges match. Tested " << count << std::endl;
    exit(1);
}

void EfficientMultiSessionOptimization::ConstructFactorGraph() {
    //changes:
    // variables connected to an isc are eliminated.
    cout << "   adding the surveys"<<endl;
    
    TestLandmarkRange();
    
    for(int survey=optstart; survey<dates.size(); survey++){
        
        bool latestsurvey = survey == POR.size()-1;
        int sidx = survey - optstart;
        
        //here, don't yet add odom constraints between eliminated variables.
        //ah, but the remaining variables need them.
        bool haveprev = false;
        LocalizePose lp(_cam);
        for(int i=0; i<POR[survey].boat.size(); i++) {
            gtsam::Pose3 traj = POR[survey].CameraPose(i);
            int lpdcur = lpdi[sidx].GetLPDIdx(i);
            if(lpdcur >= 0 && lpd_rerror[sidx][lpdcur] >= 0) {
                haveprev = false;
                continue;
            }
            rfFG->AddPose(survey, i, traj);
            GTS.InitializeValue(rfFG->GetSymbol(survey, i), &traj);
            
            if(haveprev) { //order matters; this has to be after the variables it depends on are initialized.
                gtsam::Pose3 cur1 = POR[survey].CameraPose(i);
                gtsam::Pose3 last1 = POR[survey].CameraPose(i-1);
                gtsam::Pose3 btwn = last1.between(cur1);
                rfFG->AddCustomBTWNFactor(survey, i-1, survey, i, btwn, 0.01);
            }
            haveprev = true;
        }
    }
}

void EfficientMultiSessionOptimization::AddLocalizations(){
    //changes:
    // each isc is assumed to be correct. rather than add a tf at the location of the isc,
    //  multiple tfs are added to the variable it connects with.
    cout << "   adding the localizations."<<endl;
    for(int i=0; i<lpdi.size(); i++) {
        for(int j=0; j<lpdi[i].localizations.size(); j++) {
            if(lpd_rerror[i][j] < 0) continue;
            LocalizedPoseData& lpd = lpdi[i].localizations[j];
            
            
            
//            if(lpd.s0 < optstart) { //add localization factors for locked-in surveys
//                std::vector<gtsam::Point3> p3d0 = POR[lpd.s0].GetSubsetOf3DPoints(lpd.pids);
//                rfFG->AddLocalizationFactors(_cam.GetGTSAMCam(), lpd.s1, lpd.s1time, p3d0, lpd.p2d1, lpd.rerrorp);
//            } else {
//                //rfFG->AddBTWNFactor(lpd.s0, lpd.s0time, lpd.s1, lpd.s1time, lpd.GetTFP0ToP1F0(), true);
//                double noise = pow(2, lpd_eval[i][j]/3.0) * 0.0001;
//                rfFG->AddCustomBTWNFactor(lpd.s0, lpd.s0time, lpd.s1, lpd.s1time, lpd.GetTFP0ToP1F0(), noise);
//            }
        }
    }
}

std::vector<std::vector<std::vector<double> > > EfficientMultiSessionOptimization::GetPoses(){
    std::vector<std::vector<std::vector<double> > > allposes;
    LocalizePose lp(_cam);
    for(int survey=0; survey<dates.size(); survey++) {
        std::vector<std::vector<double> > surveyposes;
        for(int i=0; i<POR[survey].boat.size(); i++) {
            vector<double> pose;
            if(rfFG->VariableExists(survey, i)) {
                pose = GTS.MAPPoseEstimate(survey, i);
            } else {
                //estimate the eliminated pose assuming the lpd is correct.
                int lpdcur = lpdi[sidx].GetLPDIdx(i);
                //TEST pose retrieval/elimination.
                if(lpdcur < 0) {
                    std::cout << "EfficientMultiSessionOptimization::GetPoses() Error. neither the variable nor the lpd is available for (" << survey <<"," <<i<<")"<<std::endl;
                    exit(-1);
                }
                
                gtsam::Pose3 refpose = lp.VectorToPose(allposes[lpd.s0][lpd.s0time]);
                gtsam::Pose3 offset = lp.GetTFP0ToP1F0();
                gtsam::Pose3 ap = refpose.compose(offset);
                pose = lp.PoseToVector(ap);
            }
            surveyposes.push_back(pose);
        }
        allposes.push_back(surveyposes);
    }
    return allposes;
}

double EfficientMultiSessionOptimization::UpdateErrorAdaptive(bool firstiter) {
    //changes:
    // calls ToggleLandmarks()
    // resolves for the map using the estimated values for the eliminated variables.
    double mult = 3;
    static vector<vector<double> > permerr;
    vector<unordered_map<int, double> > intra(dates.size());
    vector<unordered_map<int, double> > inter(dates.size());
    vector<vector<vector<double> > > poses;
    vector<vector<vector<double> > > landmarks;
    
    poses = GetPoses();
    SolveForMap sfm(_cam);
    for(int i=optstart; i<dates.size(); i++){
        int sidx = i-optstart;
        vector<vector<double> > surveylandmarks;
        for(int j=0; j<cached_landmarks[i].size(); j++) {
            std::vector<double> worldpoint = sfm.GetPoint(poses[i], cached_landmarks[i][j]);
            surveylandmarks.push_back(worldpoint);
        }
        landmarks.push_back(surveylandmarks);
        
        if(permerr.size() < dates.size()-optstart-1) permerr.push_back(vector<double>(lpdi[sidx].localizations.size(),0));
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
            
            double inter_error = erfinter.InterSurveyErrorAtLocalization(lpd, poses[sidx][lpd.s1time], landmarks, optstart);
            double intra_errorS1 = erfintraS1.OnlineRError(cached_landmarks[sidx], lpd.s1time, poses[sidx][lpd.s1time], landmarks[sidx]);
            intra[sidx][lpd.s1time] = intra_errorS1;
            double intra_errorS0 = 0;
            if(lpd.s0 >= optstart) {
                int s0idx = lpd.s0-optstart;
                auto search = intra[s0idx].find(lpd.s0time);
                if(search != intra[s0idx].end())
                    intra_errorS0 = search->second;
                else {
                    intra_errorS0 = erfintraS0.OnlineRError(cached_landmarks[lpd.s0], lpd.s0time, poses[s0idx][lpd.s0time], landmarks[s0idx]);
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
                ToggleLandmarksAtPose(i, lpd.s1time, inlier);
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















