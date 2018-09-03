/*
 * InitialISCAcquisition.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: shane
 */



#include "InitialISCAcquisition.hpp"

#include <random>
#include <string>

#include <FileParsing/ParseSurvey.h>
#include <ImageAlignment/FlowFrameworks/MachineManager.h>
#include "SFlowDREAM2RF.hpp"
#include "HopcountLog.hpp"
#include "LocalizePose.hpp"
#include "ImageToLocalization.hpp"
#include "Optimization/SingleSession/GTSamInterface.h"

using namespace std;

const string InitialISCAcquisition::_logname = "RFlowISC.log";

//i.e., the nonincremental approach. This approach is useful because progress between poses is otherwise unknown (unless we can measure odometry).
InitialISCAcquisition::InitialISCAcquisition(Camera& cam, std::string refdate, std::string priordate, std::string query_loc, std::string pftbase, std::string results_dir, std::string origin_dir):
_cam(cam), _refdate(refdate), _priordate(priordate), _query_loc(query_loc), _pftbase(pftbase), _save_dir(results_dir + refdate + "_" + priordate + "/"), _origin(origin_dir) {
    
    std::cout << "Adding IS constraints." << std::endl;
    Initialize();
}

void InitialISCAcquisition::WriteLog(vector<double> data) {
    // logfile: camera_key, poseloc, g-statistic, null_rejected?, ar.consistency, ar.alignment_energy_lowres, ar.alignment_energy
    string fname = _save_dir + _logname;
    FILE * fp = fopen(fname.c_str(), "a");
    if(!fp){
        std::cout << "InitialISCAcquisition::WriteLog() Something went wrong with the log."<<std::endl;
        exit(-1);
    }
    string line="";
    for(int i=0; i<data.size(); i++){
        if(i<data.size()-1) line += to_string(data[i]) + ",";
        else line += to_string(data[i]);
    }
    fprintf(fp, "%s\n", line.c_str());
    fclose(fp);
}

int InitialISCAcquisition::FindRestart() {
    string fname = _save_dir + _logname;
    int res=-1;
    ifstream fin;
    fin.open(fname);
    if(fin.is_open()) {
        fin.seekg(-2, ios_base::end);//position -2 from the end (assumes \n is the last char)
        while(fin.tellg() > 1) {
            char ch=0;
            fin.get(ch); //pos +1 from cur
            if(ch == '\n') break;
            fin.seekg(-2, ios_base::cur); //pos -2 from cur.
        }
        
        string lastLine;
        getline(fin,lastLine,',');                      // Read the current line, up to comma
        if(lastLine.length() > 6) res = stoi(lastLine);
        fin.close();
    }
    return res+1;
}

void InitialISCAcquisition::Initialize(){
    debug = true;
    if(!FileParsing::DirectoryExists(_save_dir)){
        FileParsing::MakeDir(_save_dir);
        std::cout << "InitialISCAcquisition::Initialize() Starting acquisition for " << _save_dir << std::endl;
    } else std::cout << "InitialISCAcquisition::Initialize() Continuing acquisition for " << _save_dir << std::endl;
    
    vector<string> dates = {_priordate, _refdate};

    std::cout << "Loading maps."<<std::endl;
    for(int i=0; i<dates.size(); i++){
        std::cout<<"  survey "<<dates[i]<<std::endl;
        _maps.push_back( new Map(_origin));
        _maps[_maps.size()-1]->LoadMap(dates[i]);
        
        //create an RF instance for the survey's data
        rf.push_back(new ReprojectionFlow(_cam, *_maps[_maps.size()-1]));
        
        //initialize trajectory estimates (the odom used by RF)
        ParseOptimizationResults por(_origin, dates[i]);
        SurveyData sd = { dates[i], por, 0.0 };
        survey_est.push_back(sd);
    }
    
    lpdi.LoadLocalizations(_save_dir);
}

vector<double> InitialISCAcquisition::EstimateNextPose(int survey, int time, int por1time, bool ref){
    std::cout<<"EstimateNextPose(): survey "<<survey<<", time "<<time<<std::endl;
    LocalizedPoseData * lpd = lpdi.NearestLPD(por1time);
    if(lpd == NULL || time < 0 || time >= survey_est[survey].por.boat.size()){
        std::cout << "Error. InitialISCAcquisition::EstimateNextPose() Out of bounds. No valid pose was found." << std::endl;
        exit(-1);
    }
    
    int lastsurvey = 1;
    int lasttime = lpd->s1time;
    std::vector<double> lastpose = lpd->p1frame0;
    if(ref) {
        lastsurvey = 0;
        lasttime = lpd->s0time;
        lastpose = lpd->p0frame1;
    }
    
    gtsam::Pose3 last = GTSamInterface::VectorToPose(lastpose);
    gtsam::Pose3 p_tm1 = survey_est[lastsurvey].por.CameraPose(lasttime);
    gtsam::Pose3 p_t = survey_est[survey].por.CameraPose(time);
    //    gtsam::Pose3 est = last.compose(p_tm1.between(p_t));
    
    gtsam::Pose3 est = last.compose(last.between(p_tm1)*p_tm1.between(p_t)*p_tm1.between(last));
    //above should equal below.
    //    gtsam::Pose3 est = last.compose(last.between(p_tm1)*p_tm1.between(p_t)*last.between(p_tm1).inverse());
    
    return GTSamInterface::PoseToVector(est);
}

int InitialISCAcquisition::RFViewpointSelection(vector<double>& rfpose){
    //Step 2: run rf viewpoint selection to get the ~best pose for all the surveys within three months.
    //the three month check is performed within MultiSurveyViewpointSelection.
    std::cout << "InitialISCAcquisition::RFViewpointSelection()" << std::endl;
    double g=0;
    int por0time = rf[0]->IdentifyClosestPose(survey_est[0].por.boat, rfpose, &g, false);
    if(g==0) return -1;
    return por0time;
}

int InitialISCAcquisition::IdentifyClosestPose(vector<double> pose1_est, string image1){
    //This function finds the image with the ~best alignment given a reference pose/image and multiple surveys.
    //It is much more efficient than exhaustive search. It takes either 0 or 1 rounds of the time of IR.
    //returns {survey, surveytime, gstat}
    static ImageRetrieval ir(_cam, nthreads); //These are necessarily global for efficiency. This avoids wait() when we want to terminate the threads.
    
    double verval;
    SurveyData& priorsurvey = survey_est[0];
    int por0time = ir.IdentifyClosestPose(_query_loc + priorsurvey.date, priorsurvey.por.boat, priorsurvey.por.cimage, pose1_est, image1, &verval);
    if(verval >= verification_threshold)
        return por0time;
    
    if(debug) std::cout<<"No good viewpoint was found during the IR step."<<std::endl;
    return -1;
}

std::vector<double> InitialISCAcquisition::FindLocalization(int por0time, int por1time, bool hasRF, vector<double> pose1_est) {
    //topk: vector of {snum, portime, gstat};
    vector<double> logdata = {(double) por1time, -1.0, -1.0, (double) hasRF, -1.0, 0.0};
    if(por0time<0) return logdata;
    
    LocalizedPoseData * toverify = NULL;
    if(hasRF) toverify = lpdi.NearestLPD(por1time);
    else toverify = &lpdi.most_adv_lpd;
    gtsam::Pose3 p1_tm1;
    if(toverify->IsSet()) p1_tm1 = survey_est[1].por.CameraPose(toverify->s1time);
    else std::cout << "verify isn't set.." << std::endl;
    
    std::cout << " The most adv lpd is: " << lpdi.most_adv_lpd.s1time << ". The one used for LPD verification is: " << toverify->s1time << std::endl;
    
    LocalizedPoseData res;
    double perc_dc = 0.0;
    double verified = 0.0;
    
    //run alignment and localization
    ImageToLocalization ITL(_cam, _origin, _query_loc, _pftbase);
    ITL.SetDebug();
    ITL.SetLastLPD(toverify);
    if(hasRF) {
        vector<double> pose0_est = EstimateNextPose(0, por0time, por1time, true);
        ITL.SetPoses({pose0_est, pose1_est});
        rf[0]->Reset();
        rf[1]->Reset();
        ITL.SetRF({rf[0], rf[1]});
    }
    
    ITL.SetPOR({&survey_est[0].por, &survey_est[1].por});
    ITL.SetDates({survey_est[0].date, _refdate});
    ITL.SetSurveyIDs({0, 1});
    ITL.SetPORTimes({por0time, por1time});
    ITL.SetVPose(p1_tm1);
    ITL.Setup(&res, &perc_dc, &verified);
    
    ITL.Run();
    ITL.LogResults();
   std::cout << "values: " << perc_dc << ", " << verified << std::endl; 
    //prepare to store/store the result.
    if(perc_dc > PERCENT_DENSE_CORRESPONDENCES){
        if(toverify->IsSet() && verified>=0){
            res.Save(_save_dir); //save all the verified localizations.
            back_two = lpdi.StoreLPD(_save_dir, *toverify);
            lpdi.StoreLPD(_save_dir, res);
            logdata[5] = 1.0;
        }
        
        logdata[1] = 0;
        logdata[2] = res.s0time;
        logdata[4] = perc_dc;
        lpdi.SetMostAdvLPD(res);
    }
    
    return logdata;
}

bool InitialISCAcquisition::GetConstraints(int por1time, bool hasRF){
    std::cout << "\n" << _refdate << " GetConstraints() with RF? " << hasRF <<" por1time -> " << por1time <<std::endl;
    struct timespec start, runir, end;
    
    clock_gettime(CLOCK_MONOTONIC, &start);
    SurveyData& latest = survey_est[1];
    vector<double> pose1_est = latest.por.boat[por1time];
    int por0time;
    if(hasRF) {
        pose1_est = EstimateNextPose(1, por1time, por1time, false);
        por0time = RFViewpointSelection(pose1_est);
    } else {
        string image1 = ParseSurvey::GetImagePath(_query_loc + latest.date, latest.por.cimage[por1time]);
        por0time = IdentifyClosestPose(pose1_est, image1);
    }
    std::cout << "InitialISCAcquisition::GetConstraints() find localization" << std::endl;
    clock_gettime(CLOCK_MONOTONIC, &runir);
    std::vector<double> logdata = FindLocalization(por0time, por1time, hasRF, pose1_est);
    clock_gettime(CLOCK_MONOTONIC, &end);
    WriteLog(logdata);
    
    if(debug && logdata[1]>=0) std::cout << "Attempted localization with "<<survey_est[(int)logdata[1]].date << "_"<<logdata[2]<<
        " (optimization hopcount " << survey_est[(int)logdata[1]].avg_hop_distance<< ") with quality: " <<
        ((int)1000*logdata[4])/10.0 << "% inliers"<<std::endl;
    string irtime = to_string((runir.tv_sec - start.tv_sec) + (runir.tv_nsec - start.tv_nsec)/1000000000.0);
    string altime = to_string((end.tv_sec - runir.tv_sec) + (end.tv_nsec - runir.tv_nsec)/1000000000.0);
    string tottime = to_string((end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec)/1000000000.0);
    if(debug) std::cout << "runtimes (s) "<<irtime << " + " << altime << " ~= " << tottime << std::endl;
    
    //the 4th entry is set when the localization is verified.
    return logdata[5] > 0;
}

int InitialISCAcquisition::InitialISCAcquisitionWithRF(int por1time, int dir){
    int count_fail = 0;
    int nentries = survey_est[1].por.boat.size();
    while(count_fail <= MAX_NO_ALIGN && por1time >= 0 && por1time < nentries){
        if(lpdi.GetLPDIdx(por1time) >= 0) break;
        if(GetConstraints(por1time, true)) count_fail = 0;
        else count_fail++;
        por1time += dir;
    }
    return por1time;
}

void InitialISCAcquisition::Run(int user_specified_start){
    bool debug=true;
    int nentries = survey_est[1].por.boat.size();
    bool haveconstraints = false;
    
    //figure out where to start.
    int por1time = lpdi.GetStartingPoint();
    int leftoffat = FindRestart();
    if(user_specified_start > 0) {
        por1time = user_specified_start;
    } else if(leftoffat > por1time) {
        if(leftoffat - por1time <= MAX_NO_ALIGN) {
            haveconstraints = true;
        }
        por1time = leftoffat;
    } else if(por1time > 0){
        haveconstraints = true;
    }
    
    if(debug) std::cout << "Starting IS Acquisition with " << por1time << " of " << nentries << std::endl;
    
    while(por1time < nentries){
        //forward with IR.
        if(debug) std::cout<<"Running IR"<<std::endl;
        while(!haveconstraints && por1time < nentries){
            haveconstraints = GetConstraints(por1time, false);
            por1time = por1time + 1;
        }
        if(!haveconstraints) break;
        
        if(debug) std::cout<<"Running RF"<<std::endl;
        int lcuridx = lpdi.localizations.size()-1;
        if(back_two){
            //backward from the first verified pose.
            InitialISCAcquisitionWithRF(lpdi.GetStartingPoint(lcuridx, LPDInterface::FROM::LastLPD, LPDInterface::DIRECTION::Backward), -1);
            //forward from the first verified pose.
            InitialISCAcquisitionWithRF(lpdi.GetStartingPoint(lcuridx, LPDInterface::FROM::LastLPD, LPDInterface::DIRECTION::Forward), 1);}
        //backward from the second verified pose.
        InitialISCAcquisitionWithRF(lpdi.GetStartingPoint(lcuridx, LPDInterface::FROM::CurLPD, LPDInterface::DIRECTION::Backward), -1);
        //forward from the second verified pose.
        por1time = InitialISCAcquisitionWithRF(lpdi.GetStartingPoint(lcuridx, LPDInterface::FROM::CurLPD, LPDInterface::DIRECTION::Forward), 1);
        haveconstraints = false;
    }
}






