/*
 * AcquireISConstraints.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: shane
 */



#include "AcquireISConstraints.hpp"

#include <random>
#include <string>


#include <ImageAlignment/FlowFrameworks/MachineManager.h>
#include "SFlowDREAM2RF.hpp"
#include "HopcountLog.hpp"
#include "LocalizePose.hpp"
#include "ImageToLocalization.hpp"

using namespace std;

const string AcquireISConstraints::_logname = "/RFlowISC.log";

void AcquireISConstraints::WriteLog(vector<double> data, vector<string> paths) {
    // logfile: camera_key, poseloc, g-statistic, null_rejected?, ar.consistency, ar.alignment_energy_lowres, ar.alignment_energy
    string fname = _map_dir + _date + _logname;
    FILE * fp = fopen(fname.c_str(), "a");
    if(!fp){
        std::cout << "AcquireISConstraints::WriteLog() Something went wrong with the log."<<std::endl;
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

int AcquireISConstraints::FindRestart() {
    string fname = _map_dir + _date + _logname;
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

void AcquireISConstraints::Initialize(){
    if(!FileParsing::DirectoryExists(_map_dir)){
        std::cout << "AcquireISConstraints::Initialize() Error: Setup 'maps/'." << _map_dir << std::endl;
        exit(-1);
    }
    
    HopcountLog hlog(_map_dir);
    
    vector<string> dates = FileParsing::ListFilesInDir(_map_dir, "1");
    std::cout << "Loading maps."<<std::endl;
    bool hasdate = false;
    for(int i=0; i<dates.size(); i++){
        if(hasdate) break;
        if(dates[i].compare(_date) == 0) {hasdate=true;}
        if(dates[i].length() != _date.length()) continue;
        std::cout<<"  survey "<<dates[i]<<std::endl;
        _maps.push_back( new Map(_map_dir));
        _maps[_maps.size()-1]->LoadMap(dates[i]);
        
        //initialize trajectory estimates (the odom used by RF)
        ParseOptimizationResults por(_map_dir + dates[i]);
        SurveyData sd = { dates[i], por, 0.0 };
        sd.avg_hop_distance = hlog.LoadHopDistance(dates[i]);
        survey_est.push_back(sd);
        
        //create an RF instance for the survey's data
        if(!hasdate) rf.push_back(new ReprojectionFlow(_cam, *_maps[_maps.size()-1]));
        else {
            for(int i=0; i<nthreads; i++)
                rf_latest.push_back(new ReprojectionFlow(_cam, *_maps[_maps.size()-1]));
            latestsurvey = survey_est.size()-1;
            SurveyData& refsurvey = survey_est[latestsurvey];
        }
    }
    
    if(dates.size() <= 1 || !hasdate) {
        std::cout << "AcquireISConstraints::Initialize() Error: 'maps/' doesn't have " << _date << std::endl;
        exit(-1);
    }
    
    if(rf.size()>0) {
        //add the unoptimized survey. this is the last one in the set.
        lpdi.LoadLocalizations(_map_dir + _date);
    }
}

vector<double> AcquireISConstraints::EstimateNextPose(int survey, int time, int por1time, bool ref){
    std::cout<<"EstimateNextPose(): survey "<<survey<<", time "<<time<<std::endl;
    LocalizedPoseData * lpd = lpdi.NearestLPD(por1time);
    if(lpd == NULL || time < 0 || time >= survey_est[survey].por.boat.size()){
        std::cout << "Error. AcquireISConstraints::EstimateNextPose() Out of bounds. No valid pose was found." << std::endl;
        exit(-1);
    }

    int lastsurvey = lpd->s1;
    int lasttime = lpd->s1time;
    std::vector<double> lastpose = lpd->p1frame0;
    if(ref) {
        lastsurvey = lpd->s0;
        lasttime = lpd->s0time;
        lastpose = lpd->p0frame1;
    }

    LocalizePose lp(_cam);
    gtsam::Pose3 last = lp.VectorToPose(lastpose);
    gtsam::Pose3 p_tm1 = survey_est[lastsurvey].por.CameraPose(lasttime);
    gtsam::Pose3 p_t = survey_est[survey].por.CameraPose(time);
    gtsam::Pose3 est = last.compose(p_tm1.between(p_t));

    return lp.PoseToVector(est);
}

std::list<int> AcquireISConstraints::CreateList(MultiSurveyViewpointSelection& msvs) {
    std::list<int> withinthree;
    for(int i=0; i<survey_est.size()-1; i++) {
        if(msvs.WithinThreeMonths(survey_est[i].date, survey_est[survey_est.size()-1].date)){
            withinthree.push_front(i);
        }
    }
    return withinthree;
}

std::vector<std::vector<double> > AcquireISConstraints::IdentifyClosestPose(vector<double> pose1_est, string image1, bool run_initial_IR){
    //This function finds the image with the ~best alignment given a reference pose/image and multiple surveys.
    //It is much more efficient than exhaustive search. It takes either 0 or 1 rounds of the time of IR.
    //returns {survey, surveytime, gstat}
    static MultiSurveyViewpointSelection msvs;
    static ImageRetrieval ir(_cam, nthreads); //These are necessarily global for efficiency. This avoids wait() when we want to terminate the threads.
    static std::list<int> withinthree;
    if(withinthree.size()==0) withinthree = CreateList(msvs);
    int refsurvidx = survey_est.size()-2;
    int por0time;

    //step 1: identify a good viewpoint using IR on the previously referenced survey
    if(run_initial_IR) {
        //start at the last best survey, if no good viewpoint is found, decrement refsurvidx until one is.
        //This approach isn't exhaustive, but that's probably OK for our application
        bool found = false;
        for (std::list<int>::iterator iterator = withinthree.begin(), end = withinthree.end();
                iterator != end; ++iterator) {
            refsurvidx = *iterator;
            SurveyData refsurvey = survey_est[refsurvidx];
            double ae;
            por0time = ir.IdentifyClosestPose(_query_loc + refsurvey.date, refsurvey.por.boat, refsurvey.por.cimage, pose1_est, image1, &ae);
            if(por0time >= 0) {
                withinthree.erase(iterator);
                withinthree.push_front(refsurvidx);
                found=true;
                break;
            }
        }

        if(!found) {
            if(debug) std::cout<<"No good viewpoint was found during the IR step."<<std::endl;
            return {};
        }
    }

    //return the result if there's only one reference survey.
    if(survey_est.size() == 2 && run_initial_IR){
        return {{0.0, (double) por0time, 0}};
    }

    //Step 2: run rf viewpoint selection to get the ~best pose for all the surveys within three months.
    std::vector<double> rfpose = pose1_est;
    if(run_initial_IR) rfpose = survey_est[refsurvidx].por.boat[por0time]; //use the pose of the image found by step 1) IR.
    int npriorsurveys = rf.size();
    std::vector<std::vector<std::vector<double> > *> poselists(npriorsurveys);
    std::vector<string> dates(npriorsurveys);
    for(int i=0; i<npriorsurveys; i++) {
        poselists[i] = &(survey_est[i].por.boat);
        dates[i] = survey_est[i].date;
    }

    //returns {snum, portime, gstat};
    return msvs.TopKViewpoints(rf, poselists, dates, rfpose, nthreads);
}

std::vector<double> AcquireISConstraints::FindLocalization(std::vector<std::vector<double> > topk, int por1time, bool hasRF, vector<double> pose1_est) {
    //topk: vector of {snum, portime, gstat};
    vector<double> logdata = {(double)por1time, -1.0, -1.0, (double) hasRF, -1.0, 0.0};
    if(topk.size()==0) return logdata;
    
    MachineManager man;
    vector<ImageToLocalization*> ws;
    int loc_nthreads = min(nthreads, (int) topk.size());
    for(int i=0; i<loc_nthreads; i++) {
        ws.push_back(new ImageToLocalization(_cam, _map_dir, _query_loc, _pftbase));
        ws[i]->SetDebug();
        man.AddMachine(ws[i]);
    }
    
    LocalizedPoseData * toverify = NULL;
    if(hasRF) toverify = lpdi.NearestLPD(por1time);
    else toverify = &lpdi.most_adv_lpd;
    gtsam::Pose3 p1_tm1;
    if(toverify->IsSet()) p1_tm1 = survey_est[toverify->s1].por.CameraPose(toverify->s1time);

    std::cout << " The most adv lpd is: " << lpdi.most_adv_lpd.s1time << ". The one used for LPD verification is: " << toverify->s1time << std::endl;

    vector<LocalizedPoseData> res(topk.size());
    vector<double> perc_dc(topk.size(), 0.0);
    bool verified[topk.size()];

    //run alignment and localization in parallel among the topk
    for(int i=0; i<topk.size(); i++) {
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

        if(topk.size()==1) ws[tidx]->Run();
        else man.RunMachine(tidx);
    }
    man.WaitForMachine(true);

    //get the best result, the verified one with the least avg_hop_distance
    int bestidx = -1;
    double leasthops = 10000000000;
    bool hasverified = false;
    for(int i=0; i<topk.size(); i++) {
        if(perc_dc[i] <= PERCENT_DENSE_CORRESPONDENCES) continue;
        double hops = survey_est[topk[i][0]].avg_hop_distance;
        if(toverify->IsSet() && verified[i]){
            if(!hasverified || leasthops > hops){
                hasverified = true;
                leasthops = hops;
                bestidx = i;
            }
        } else if(!hasverified && res[i].IsSet() && leasthops > hops){
            leasthops = hops;
            bestidx = i;
        }
    }

    //prepare to store/store the result.
    if(bestidx >= 0) {
        logdata[1] = res[bestidx].s0;
        logdata[2] = res[bestidx].s0time;
        logdata[4] = perc_dc[bestidx];
        if(hasverified) {
            back_two = lpdi.StoreLPD(_map_dir, *toverify);
            lpdi.StoreLPD(_map_dir, res[bestidx]);
            logdata[5] = 1.0;
        } //else res[bestidx].Save(_map_dir + _date, "/unverified/"); //rather than save unverified, save all the verified, and use a different naming convention.
        lpdi.SetMostAdvLPD(res[bestidx]);
    }

    for(int i=0; i<loc_nthreads; i++) delete(ws[i]);
    return logdata;
}

bool AcquireISConstraints::GetConstraints(int por1time, bool hasRF){
    std::cout << "\n" << _date << " GetConstraints() with RF? " << hasRF <<" por1time -> " << por1time <<std::endl;
    struct timespec start, runir, end;

    clock_gettime(CLOCK_MONOTONIC, &start);
    SurveyData latest = survey_est[latestsurvey];
    vector<double> pose1_est = latest.por.boat[por1time];
    if(hasRF) pose1_est = EstimateNextPose(latestsurvey, por1time, por1time, false);
    string image1 = ParseSurvey::GetImagePath(_query_loc + latest.date, latest.por.cimage[por1time]);
    std::vector<std::vector<double> > topk = IdentifyClosestPose(pose1_est, image1, !hasRF);

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

int AcquireISConstraints::AcquireISConstraintsWithRF(int por1time, int dir){
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

void AcquireISConstraints::Run(int user_specified_start){
    bool debug=true;
    int nentries = survey_est[latestsurvey].por.boat.size();
    bool haveconstraints = false;
    
    //figure out where to start.
    int por1time = lpdi.GetStartingPoint();
    int leftoffat = FindRestart();
    if(user_specified_start > 0) {
        por1time = user_specified_start;
    } else if(leftoffat > por1time) {
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
        AcquireISConstraintsWithRF(lpdi.GetStartingPoint(lcuridx, lpdi.FROM::LastLPD, lpdi.DIRECTION::Backward), -1); //backward from the first verified pose.
        AcquireISConstraintsWithRF(lpdi.GetStartingPoint(lcuridx, lpdi.FROM::LastLPD, lpdi.DIRECTION::Forward), 1);} //forward from the first verified pose.
        //if(localizations[lcuridx].s1time - localizations[lcuridx-1].s1time >5) //only go backward here if overlap is small.
        AcquireISConstraintsWithRF(lpdi.GetStartingPoint(lcuridx, lpdi.FROM::CurLPD, lpdi.DIRECTION::Backward), -1); //backward from the second verified pose.
        por1time = AcquireISConstraintsWithRF(lpdi.GetStartingPoint(lcuridx, lpdi.FROM::CurLPD, lpdi.DIRECTION::Forward), 1); //forward from the second verified pose.
        haveconstraints = false;
    }
}






