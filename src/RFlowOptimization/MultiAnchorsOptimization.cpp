/*
 * MultiAnchorsOptimization.cpp
 *
 *  Created on: ~Aug 7, 2017
 *      Author: shane
 */

#include "MultiAnchorsOptimization.hpp"

#include <random>
#include <string>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/SaveOptimizationResults.h>
#include <Optimization/EvaluateSLAM.h>
#include <RFlowOptimization/EvaluateRFlowAnchors.hpp>

#include "SFlowDREAM2RF.hpp"
#include "LocalizePose.hpp"
#include "EvaluateRFlow.hpp"
#include "HopcountLog.hpp"
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/LandmarkTrack.h>
#include "SolveForMap.hpp"

using namespace std;

MultiAnchorsOptimization::MultiAnchorsOptimization(Camera& cam, std::string results_dir, std::string pftbase, std::string date):
_map_dir(results_dir + "maps/"), _pftbase(pftbase),
SurveyOptimizer(cam, rfFG, date, results_dir, false) {
    rfFG = new RFlowFactorGraph();
    FG = rfFG;
    
    std::cout << "Multi-Session Optimization up to " << date << std::endl;
    
    std::cout << "  Initializing.."<<std::endl;
    IdentifyOptimizationDates();
    Initialize();
    rfFG->SetLandmarkDeviation(3.0); //must be *after* initialize();
}

void MultiAnchorsOptimization::IdentifyOptimizationDates(){
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

void MultiAnchorsOptimization::Initialize() {
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
        
        A.push_back(Anchors(_cam, dates[i], POR[i], POR[i].boat.size(), POR[i].boat.size()));
        
        std::vector<LandmarkTrack> clset;
        cached_landmarks.push_back(clset);
        
        HopcountLog hlog(_map_dir);
        vector<double> rerror_set = hlog.LoadPriorRerror(dates[i], nloaded);
        lpd_rerror.push_back(rerror_set);
        vector<double> le(nloaded, 3.0);
        lpd_eval.push_back(le);
        
        int ninliers = 0;
        for(int i=0; i<rerror_set.size(); i++)
            ninliers += (rerror_set[i]==1)?1:0;
        inlier_ratio.push_back(1.*ninliers/rerror_set.size());
    }
    BuildLandmarkSet();
}


void MultiAnchorsOptimization::BuildLandmarkSet() {
    for(int survey=optstart; survey<dates.size(); survey++) {
        cache_set = survey-optstart; //update the parent class variable
        rfFG->ChangeLandmarkSet(cache_set);
        
        for(int i=0; i<POR[survey].boat.size(); i++) {
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


void MultiAnchorsOptimization::ConstructFactorGraph(bool firstiter) {
    cout << "   adding the surveys"<<endl;
    
    int ca = 0, iscs = 0;
    
    //add all the surveys to the graph
    LocalizePose lp(_cam);
    for(int survey=optstart; survey<dates.size(); survey++){
        bool latestsurvey = survey == POR.size()-1;
        int sidx = survey - optstart;
        
        for(int i=0; i<POR[survey].boat.size(); i++) {
            int aidx = A[sidx].PoseIdxToAnchorIdx(i);
            if(A[sidx].IsTransition(i)) {
                //add anchor.
                gtsam::Pose3 anc = A[sidx].GetAnchorAsPose(aidx);
                //if the latest survey and the first iteration, could use p1frame0 to estimate the anchor. (also, assuming that the set starts out with 1 per pose).
                //with anchors, it would be p1frame0 = a1p1, because a0p0 would be used for localization. so the offset from a0p0
                // p0.between(p1frame0)
                //a1 should be p1frame0 * p1.inv();
                /*if(latestsurvey && firstiter) {
                    //use an ISC to initialize the anchor if one exists..
                    int ajdx = aidx;
                    bool found = false;
                    for(int j=i; !found && aidx == ajdx && j<POR[survey].boat.size(); j++) {
                        ajdx = A[sidx].PoseIdxToAnchorIdx(j);
                        int lpdcur = lpdi[sidx].GetLPDIdx(i);
                        if(lpdcur >=0){
                            found = true;
                            gtsam::Pose3 p1 = POR[survey].CameraPose(j);
                            anc = lp.VectorToPose(lpdi[sidx].localizations[lpdcur].p1frame0).compose(p1.inverse());
                        }
                    }
                }*/
                rfFG->AddPose(survey, aidx, anc);
                GTS.InitializeValue(rfFG->GetSymbol(survey, aidx), &anc);
                ca++;
                if(aidx>0) {
                    //add an AnchorISC factor between the consecutive anchors for the odometry constraint.
                    gtsam::Pose3 cur1 = POR[survey].CameraPose(i);
                    gtsam::Pose3 last1 = POR[survey].CameraPose(i-1);
                    gtsam::Pose3 btwn = last1.between(cur1);
                    //this model may be better approximated using a chow-liu tree.
                    rfFG->AddAnchorFactor(sidx, aidx-1, sidx, aidx, last1, cur1, btwn, 0.0001);
                }
            }
            int lpdcur = lpdi[sidx].GetLPDIdx(i);
            if(lpdcur >= 0 && lpd_rerror[sidx][lpdcur] >= 0) {
                //add an AnchorISC factor between the two surveys for the ISC constraint.
                int s0idx = lpdi[sidx].localizations[lpdcur].s0-optstart;
                int a0idx = A[s0idx].PoseIdxToAnchorIdx(lpdi[sidx].localizations[lpdcur].s0time);
                gtsam::Pose3 p1 = POR[survey].CameraPose(i);
                gtsam::Pose3 p0 = POR[lpdi[sidx].localizations[lpdcur].s0].CameraPose(lpdi[sidx].localizations[lpdcur].s0time);
                double noise = pow(2, lpd_eval[sidx][lpdcur]/3.0) * 0.0001;
                rfFG->AddAnchorFactor(s0idx, a0idx, sidx, aidx, p0, p1, lpdi[sidx].localizations[lpdcur].GetTFP0ToP1F0(), noise);
                iscs++;
            }
        }
    }
    std::cout << " Factor graph of " << ca << " anchors and " << iscs << " iscs " << std::endl;
}


double MultiAnchorsOptimization::UpdateErrorAdaptive(bool firstiter) {
    double mult = 3;
    static vector<vector<double> > permerr;
    vector<unordered_map<int, double> > intra(dates.size());
    vector<EvaluateRFlowAnchors> erfintra(dates.size());
    vector<vector<vector<double> > > landmarks;
    
    SolveForMap shiftedmap(_cam);
    for(int i=0; i<dates.size(); i++){
        vector<vector<double> > updatedanchors = GTS.GetOptimizedTrajectory(i, A[i].NumAnchors());
        A[i].UpdateAnchors(updatedanchors);
        vector<vector<double> > surveylandmarks;
        for(int j=0; j<cached_landmarks[i].size(); j++) {
            //accounts for the 3D points that are all zeros?
            std::vector<double> shifted = shiftedmap.GetPoint(POR[i], A[i], cached_landmarks[i][j]);
            surveylandmarks.push_back(shifted);
        }
        landmarks.push_back(surveylandmarks);
        A[i].ModifyAnchors(surveylandmarks, rerrs[i], POR[i], _pftbase+dates[i]);
        
        permerr.push_back(vector<double>(lpdi[i].localizations.size(),0));
        
        for(int j=0; j<POR[i].boat.size(); j++){
            int aidx = A[i].PoseIdxToAnchorIdx(j);
            vector<double> anchor = A[i].anchors[aidx];
            intra[i][j] = erfintra[i].ComputeAnchorRError(anchor, POR[i], j, _pftbase+dates[i], landmarks[i]);
        }
    }
    
    double totchanges = 0;
    for(int i=0; i<dates.size(); i++){
        EvaluateRFlow erfinter(_cam);
        
        int nchanges = 0;
        int coutliers = 0;
        for(int j=0; j<lpdi[i].localizations.size(); j++) {
            LocalizedPoseData& lpd = lpdi[i].localizations[j];
            
            int aidx = A[lpd.s1].PoseIdxToAnchorIdx(lpd.s1time);
            vector<double> shifted = A[lpd.s1].ShiftPose(aidx, POR[i].boat[lpd.s1time]);
            double inter_error = erfinter.InterSurveyErrorAtLocalization(lpd, shifted, landmarks, 0);
            double intra_errorS1 = intra[i][lpd.s1time];
            double intra_errorS0 = intra[lpd.s0][lpd.s0time];
            
            double newval = std::max(3.0, std::max(std::max(inter_error, intra_errorS1), intra_errorS0));
            lpd_eval[i][j] = newval;
            if(newval > 15) coutliers++;
            
            double LPD_RERROR_THRESHOLD = rerrs[i][lpd.s1time]*mult;
            if(LPD_RERROR_THRESHOLD < 0) {
                std::cout << "RFlowSurveyOptimizer::UpdateError() Something went wrong with the Rerror file. Got negative rerror."<<std::endl;
                exit(1);
            } else if (LPD_RERROR_THRESHOLD == 0) { //this occurs at places in the rerr vector that are zero.
                LPD_RERROR_THRESHOLD = AverageRerror[i]*mult;
            }
            
            bool inlier = true;
            if(std::isnan(inter_error) ||
               inter_error > LPD_RERROR_THRESHOLD) inlier = false;
            if(std::isnan(intra_errorS0) ||
               std::isnan(intra_errorS1)) inlier = false;
            if(intra_errorS0 > LPD_RERROR_THRESHOLD || //this threshold corresponds to s1, but it's inessential to change it.
               intra_errorS1 > LPD_RERROR_THRESHOLD) permerr[i][j] = 1;
            if(permerr[i][j] > 0) inlier = false;
            
            if(lpd_rerror[i][j] == 0 ||
               (inlier && lpd_rerror[i][j] < 0) ||
               (!inlier && lpd_rerror[i][j] > 0)) {
                nchanges++;
                if(!firstiter){
                    if(inlier) std::cout << "activated   ("<<lpd.s1<<"."<<lpd.s1time << ", "<<lpd.s0<<"."<<lpd.s0time<<")" << std::endl;
                    else       std::cout << "deactivated ("<<lpd.s1<<"."<<lpd.s1time << ", "<<lpd.s0<<"."<<lpd.s0time<<")" << std::endl;
                }
            }
            
            lpd_rerror[i][j] = 1;
            if(!inlier) {lpd_rerror[i][j] = -1;}
        }
        totchanges += nchanges;
        
        erfintra[i].PrintTots("intra " + dates[i]);
        if(i > 0) {
            erfinter.PrintTots("inter");
            std::cout << "number of (virtual) outliers: " << coutliers << std::endl;
        }
        inlier_ratio[i] = 1.0-(1.*coutliers/lpdi[i].localizations.size());
    }
    exit(1);
    //returns avg num_changes.
    if(optstart==0) return (int) (totchanges/(dates.size()-1)); //convert to int to avoid unnecessary iterations due to very small changes
    return (int) (totchanges/(dates.size()-optstart));
}


void MultiAnchorsOptimization::SaveResults() {
    std::cout << "Save disabled. Update this function." << std::endl;
    return;
    
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


void MultiAnchorsOptimization::IterativeMerge() {
    time_t beginning,optstart,end;
    time (&beginning);
    LocalizePose lp(_cam);
    double last_nchanges = 10000000000;
    double avg_nchanges= last_nchanges - 1;
    for(int i=0; avg_nchanges>0 && last_nchanges>avg_nchanges && i<MAX_ITERATIONS; i++) {
        rfFG->Clear();
        
        if(i>0) std::cout << "Merging Surveys. Iteration " << i << " of " << MAX_ITERATIONS << ", nchanges in last iteration: " << avg_nchanges << std::endl;
        std::cout << "  Constructing the factor graph." << std::endl;
        ConstructFactorGraph(i==0);
        
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





/*
 ANCHORS:
 >need:
  -a new file format to save the anchors.
  -ISC factors to anchors.
  -anchor variables.
 >preprocessing
  -load all the POR files.
  -compute a set of anchors for the latest survey.
  -load the map as it’s currently loaded.
  -load the existing anchor sets.
 >saving:
  -the anchors to a file.
 */















































