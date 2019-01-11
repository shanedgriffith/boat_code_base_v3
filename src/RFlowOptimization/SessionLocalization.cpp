/*
 * SessionLocalization.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: shane
 */



#include "SessionLocalization.hpp"

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

const string SessionLocalization::_logname = "/RFlowISC.log";

//i.e., the nonincremental approach. This approach is useful because progress between poses is otherwise unknown (unless we can measure odometry).
SessionLocalization::SessionLocalization(const Camera& cam, std::string date, std::string query_loc, std::string pftbase, std::string results_dir):
_cam(cam), _date(date), _query_loc(query_loc), _pftbase(pftbase), _map_dir(results_dir + "maps/"), _store_dir(results_dir) {
    
    std::cout << "Acquiring IS constraints." << std::endl;
    
    if(_store_dir.compare(_map_dir) !=0 ){
        std::cout << "SessionLocalization error. The session will be localized to the preexisting map and shouldn't be part of it." << std::endl;
    }
    
    Initialize();
}

void SessionLocalization::WriteLog(vector<double> data, vector<string> paths) {
    // logfile: camera_key, poseloc, g-statistic, null_rejected?, ar.consistency, ar.alignment_energy_lowres, ar.alignment_energy
    string fname = _store_dir + _date + _logname;
    FILE * fp = fopen(fname.c_str(), "a");
    if(!fp){
        std::cout << "SessionLocalization::WriteLog() Something went wrong with the log."<<std::endl;
        exit(-1);
    }
    string line="";
    for(int i=0; i<data.size(); i++){
        if(i<data.size()-1 || paths.size()>0) line += to_string(data[i]) + ",";
        else line += to_string(data[i]);
    }
    for(int i=0; i<paths.size(); i++){
        if(i<paths.size()-1) line += paths[i] + ",";
        else line += paths[i];
    }
    fprintf(fp, "%s\n", line.c_str());
    fclose(fp);
}

int SessionLocalization::FindRestart() {
    string fname = _store_dir + _date + _logname;
    int res=-1;
    ifstream fin;
    fin.open(fname);
    if(fin.is_open()) {
        fin.seekg(-2, ios_base::end);//position -2 from the end (assumes \n is the last char)
        while(fin.tellg() > 0) {
            char ch=0;
            fin.get(ch); //pos +1 from cur
            if(ch == '\n') break;
            fin.seekg(-2, ios_base::cur); //pos -2 from cur.
        }

        string lastLine;
        try {
            getline(fin,lastLine,',');                      // Read the current line, up to comma
        if(lastLine.length() > 6) res = stoi(lastLine);
        } catch(std::exception& e){
            std::cout << "SessionLocalization::FindRestart() " << fname << " " << lastLine << "\n"<< e.what() << std::endl;
            exit(-1);
        }
        fin.close();
    }
    return res+1;
}

void SessionLocalization::Initialize(){
    debug = true;
    if(!FileParsing::DirectoryExists(_map_dir) || !FileParsing::DirectoryExists(_store_dir + _date)){
        std::cout << "SessionLocalization::Initialize() Error: Setup " << _map_dir << " and " << _store_dir + _date << std::endl;
        exit(-1);
    }
    
    vector<string> dates = FileParsing::ListFilesInDir(_map_dir, "1");
    
    std::cout << "Loading maps."<<std::endl;
    bool hasdate = false;
    for(int i=0; i<dates.size(); i++){
        if(dates[i].length() != _date.length()) continue;
        std::cout<<"  survey "<<dates[i]<<std::endl;
        _maps.push_back( new Map(_map_dir));
        _maps[_maps.size()-1]->LoadMap(dates[i]);
        
        //create an RF instance for the survey's data
        rf.push_back(new ReprojectionFlow(_cam, *_maps[_maps.size()-1]));
        
        //initialize trajectory estimates (the odom used by RF)
        ParseOptimizationResults por(_map_dir, dates[i]);
        SurveyData sd = { dates[i], por, 0.0 };
        survey_est.push_back(sd);
    }
    
    std::cout<<"  survey "<<_date<<std::endl;
    
    lpdi.LoadLocalizations(_store_dir + _date);
    _maps.push_back( new Map(_store_dir));
    _maps[_maps.size()-1]->LoadMap(_date);
    
    for(int i=0; i<nthreads; i++)
        rf_latest.push_back(new ReprojectionFlow(_cam, *_maps[_maps.size()-1]));
    
    ParseOptimizationResults por(_store_dir, _date);
    SurveyData sd = { _date, por, 0.0 };
    survey_est.push_back(sd);
    latestsurvey = survey_est.size()-1;
    SurveyData& refsurvey = survey_est[latestsurvey];
}

int SessionLocalization::DateToIndex(std::string date){
    for(int i=0; i<survey_est.size(); i++){
        if(survey_est[i].date.compare(date)==0) return i;
    }
    std::cout << "SessionLocalization::DateToIndex() Error. Date not found." << std::endl;
    exit(-1);
}

vector<double> SessionLocalization::EstimateNextPose(int survey, int time, int por1time, bool ref){
    std::cout<<"EstimateNextPose(): survey "<<survey<<", time "<<time<<std::endl;
    LocalizedPoseData * lpd = lpdi.NearestLPD(por1time);
    if(lpd == NULL || time < 0 || time >= survey_est[survey].por.boat.size()){
        std::cout << "Error. SessionLocalization::EstimateNextPose() Out of bounds. No valid pose was found." << std::endl;
        exit(-1);
    }

    int lastsurvey = DateToIndex(lpd->date1);
    int lasttime = lpd->s1time;
    std::vector<double> lastpose = lpd->p1frame0;
    if(ref) {
        lastsurvey = DateToIndex(lpd->date0);
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

std::list<int> SessionLocalization::CreateList() {
    MultiSurveyViewpointSelection msvs(0); //used solely for the withinthree check.
    std::list<int> withinthree;
    for(int i=0; i<survey_est.size()-1; i++) {
        if(msvs.WithinThreeMonths(survey_est[i].date, survey_est[survey_est.size()-1].date)){
            withinthree.push_front(i);
        }
    }
    return withinthree;
}

std::vector<std::vector<double> > SessionLocalization::RFViewpointSelection(vector<double>& rfpose){
    //Step 2: run rf viewpoint selection to get the ~best pose for all the surveys within three months.
    //the three month check is performed within MultiSurveyViewpointSelection.
    static MultiSurveyViewpointSelection msvs;
    
    int npriorsurveys = rf.size();
    std::vector<std::vector<std::vector<double> > *> poselists(npriorsurveys);
    std::vector<string> dates(npriorsurveys);
    for(int i=0; i<npriorsurveys; i++) {
        poselists[i] = &(survey_est[i].por.boat);
        dates[i] = survey_est[i].date;
    }
    
    //returns {snum, portime, gstat};
    return msvs.TopViewpoints(rf, poselists, dates, rfpose);
}

std::vector<std::vector<double> > SessionLocalization::IdentifyClosestPose(vector<double> pose1_est, string image1){
    //This function finds the image with the ~best alignment given a reference pose/image and multiple surveys.
    //It is much more efficient than exhaustive search. It takes either 0 or 1 rounds of the time of IR.
    //returns {survey, surveytime, gstat}
    static ImageRetrieval ir(_cam, nthreads); //These are necessarily global for efficiency. This avoids wait() when we want to terminate the threads.
    static std::list<int> withinthree;
    if(withinthree.size()==0) withinthree = CreateList();
    int refsurvidx = survey_est.size()-2;
    int por0time;
    std::vector<std::vector<double> > current_topk;
    std::vector<std::vector<double> > last_topk;
    

    //step 1: identify a good viewpoint using IR on the previously referenced survey
    //start at the last best survey, if no good viewpoint is found, decrement refsurvidx until one is.
    //This approach isn't exhaustive, but that's probably OK for our application
    for (std::list<int>::iterator iterator = withinthree.begin(), end = withinthree.end();
         iterator != end; ++iterator) {
        refsurvidx = *iterator;
        SurveyData refsurvey = survey_est[refsurvidx];
        double verval;
        por0time = ir.IdentifyClosestPose(_query_loc + refsurvey.date, refsurvey.por.boat, refsurvey.por.cimage, pose1_est, image1, &verval);
        if(por0time <0) continue;
        
        //return the result if it's good and there's only one reference survey.
        if(survey_est.size() == 2 && verval >= verification_threshold){
            return {{(double) refsurvidx, (double) por0time, 0.0}};
        }
        
        //Check if image retrieval verified the most covisible pose.
        for(int i=0; i<last_topk.size(); i++)
            if(last_topk[i][0] == refsurvidx){
                if(last_topk[i][1] == por0time){
                    if(debug) std::cout<<"The most covisible pose was verified."<<std::endl;
                    return last_topk;
                }
                break;
            }
        
        //acquire the covisible set for refsurvidx.por0time
        std::vector<double> rfpose = survey_est[refsurvidx].por.boat[por0time];
        current_topk = RFViewpointSelection(rfpose);
        if(current_topk.size() == 0) continue;
        
        //Check if image retrieval found a verified alignment.
        if(verval >= verification_threshold) {
            withinthree.erase(iterator);
            withinthree.push_front(refsurvidx);
            
            int maxidx = std::min(nthreads, (int) current_topk.size());
            
            //ensure the verified one made it into the top of the list.
            for(int i=0; i<maxidx; i++)
                if(current_topk[i][0] == refsurvidx)
                    return current_topk;
            
            current_topk[maxidx-1][0] = refsurvidx;
            current_topk[maxidx-1][1] = por0time;
            
            return current_topk;
        }
        
        last_topk = current_topk;
    }
    
    if(debug) std::cout<<"No good viewpoint was found during the IR step."<<std::endl;
    return {};
}

std::vector<double> SessionLocalization::FindLocalization(std::vector<std::vector<double> > topk, int por1time, bool hasRF, vector<double> pose1_est) {
    //topk: vector of {snum, portime, gstat};
    vector<double> logdata = {(double)por1time, -1.0, -1.0, (double) hasRF, -1.0, 0.0};
    if(topk.size()==0) return logdata;
    int loc_nthreads = min(nthreads, (int) topk.size());
    
    static MachineManager man;
    static vector<ImageToLocalization*> ws;
    if(ws.size() == 0) {
        for(int i=0; i<loc_nthreads; i++) {
            ws.push_back(new ImageToLocalization(_cam, _map_dir, _query_loc, _pftbase));
            ws[i]->SetDebug();
            man.AddMachine(ws[i]);
        }
    }
    
    LocalizedPoseData * toverify = NULL;
    if(hasRF) toverify = lpdi.NearestLPD(por1time);
    else toverify = &lpdi.most_adv_lpd;
    gtsam::Pose3 p1_tm1;
    if(toverify->IsSet()) p1_tm1 = survey_est[survey_est.size()-1].por.CameraPose(toverify->s1time);
    
    std::cout << " The most adv lpd is: " << lpdi.most_adv_lpd.s1time << ". The one used for LPD verification is: " << toverify->s1time << std::endl;
    
    vector<LocalizedPoseData> res(loc_nthreads);
    vector<double> perc_dc(loc_nthreads, 0.0);
    vector<double> verified(loc_nthreads, 0.0);
    
    //run alignment and localization in parallel among the topk
    for(int i=0; i<loc_nthreads; i++) {
        int tidx = man.GetOpenMachine();
        int s0 = topk[i][0];
        int por0time = topk[i][1];

        ws[tidx]->SetLastLPD(toverify);
        if(hasRF) {
            vector<double> pose0_est = EstimateNextPose(s0, por0time, por1time, true);
            ws[tidx]->SetPoses({pose0_est, pose1_est});
            rf[s0]->Reset();
            rf_latest[tidx]->Reset();
            ws[tidx]->SetRF({rf[s0], rf_latest[tidx]});
        }

        ws[tidx]->SetPOR({&survey_est[s0].por, &survey_est[latestsurvey].por});
        ws[tidx]->SetDates({survey_est[s0].date, _date});
        ws[tidx]->SetSurveyIDs({s0, latestsurvey});
        ws[tidx]->SetPORTimes({por0time, por1time});
        ws[tidx]->SetVPose(p1_tm1);
        ws[tidx]->Setup(&res[i], &perc_dc[i], &verified[i]);

        if(loc_nthreads==1) ws[tidx]->Run();
        else man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    
    //get the best result, the verified one with the least avg_hop_distance
    int bestidx = -1;
//    double leasthops = 10000000000;
    double leastrerror = 10000000000;
    double most_perc_dc = 0;
    bool hasverified = false;
    for(int i=0; i<loc_nthreads; i++) {
        //std::cout << "date: " << survey_est[topk[i][0]].date << ", perc_dc: " << perc_dc[i] << ", verified? " << verified[i] << ", set? " << res[i].IsSet() << std::endl;
        if(perc_dc[i] <= PERCENT_DENSE_CORRESPONDENCES) continue;
//        double hops = survey_est[topk[i][0]].avg_hop_distance;
        if(toverify->IsSet() && verified[i]>=0){
            res[i].Save(_store_dir + res[i].date1); //save all the verified localizations.
            if(!hasverified || leastrerror > verified[i]){
                leastrerror = verified[i];
                bestidx = i;
            }
            hasverified = true;
        } else if(!hasverified && res[i].IsSet() && most_perc_dc < perc_dc[i]){
            //if there's no verified pose, use the one with the least hops
//            leasthops = hops;
            most_perc_dc = perc_dc[i];
            bestidx = i;
        }
    }

    //prepare to store/store the result.
    if(bestidx >= 0) {
        logdata[1] = DateToIndex(res[bestidx].date0);
        logdata[2] = res[bestidx].s0time;
        logdata[4] = perc_dc[bestidx];
        if(hasverified) {
            back_two = lpdi.StoreLPD(_store_dir + _date, *toverify);
            lpdi.StoreLPD(_store_dir + _date, res[bestidx]);
            logdata[5] = 1.0;
        } //else res[bestidx].Save(_store_dir + _date, "/unverified/"); //rather than save unverified, save all the verified, and use a different naming convention.
        lpdi.SetMostAdvLPD(res[bestidx]);
    }

    //for(int i=0; i<loc_nthreads; i++) delete(ws[i]);
    return logdata;
}

bool SessionLocalization::GetConstraints(int por1time, bool hasRF){
    std::cout << "\n" << _date << " GetConstraints() with RF? " << hasRF <<" por1time -> " << por1time <<std::endl;
    struct timespec start, runir, end;

    clock_gettime(CLOCK_MONOTONIC, &start);
    SurveyData latest = survey_est[latestsurvey];
    vector<double> pose1_est = latest.por.boat[por1time];
    std::vector<std::vector<double> > topk;
    if(hasRF) {
        pose1_est = EstimateNextPose(latestsurvey, por1time, por1time, false);
        topk = RFViewpointSelection(pose1_est);
    } else {
        string image1 = ParseSurvey::GetImagePath(_query_loc + latest.date, latest.por.cimage[por1time]);
        topk = IdentifyClosestPose(pose1_est, image1);
    }

    clock_gettime(CLOCK_MONOTONIC, &runir);
    std::vector<double> logdata = FindLocalization(topk, por1time, hasRF, pose1_est);
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

int SessionLocalization::SessionLocalizationWithRF(int por1time, int dir){
    int count_fail = 0;
    int nentries = survey_est[latestsurvey].por.boat.size();
    while(count_fail <= MAX_NO_ALIGN && por1time >= 0 && por1time < nentries){
        if(lpdi.GetLPDIdx(por1time) >= 0) break;
        if(GetConstraints(por1time, true)) count_fail = 0;
        else count_fail++;
        por1time += dir;
    }
    return por1time;
}

void SessionLocalization::Run(int user_specified_start){
    bool debug=true;
    int nentries = survey_est[latestsurvey].por.boat.size();
    bool haveconstraints = false;
    
    //figure out where to start.
    int por1time = lpdi.GetStartingPoint();
    int leftoffat = FindRestart();
    if(user_specified_start > 0) {
        por1time = user_specified_start;
    } else if(leftoffat > por1time) {
        if(leftoffat - por1time <= MAX_NO_ALIGN && por1time != 0) {
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
        SessionLocalizationWithRF(lpdi.GetStartingPoint(lcuridx, LPDInterface::FROM::LastLPD, LPDInterface::DIRECTION::Backward), -1);
            //forward from the first verified pose.
        SessionLocalizationWithRF(lpdi.GetStartingPoint(lcuridx, LPDInterface::FROM::LastLPD, LPDInterface::DIRECTION::Forward), 1);}
            //backward from the second verified pose.
        SessionLocalizationWithRF(lpdi.GetStartingPoint(lcuridx, LPDInterface::FROM::CurLPD, LPDInterface::DIRECTION::Backward), -1);
            //forward from the second verified pose.
        por1time = SessionLocalizationWithRF(lpdi.GetStartingPoint(lcuridx, LPDInterface::FROM::CurLPD, LPDInterface::DIRECTION::Forward), 1);
        haveconstraints = false;
    }
}






