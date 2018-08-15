//
//  HopcountLog
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "HopcountLog.hpp"


using namespace std;

const string HopcountLog::_locoptname = "/locoptlist.csv";


double HopcountLog::LoadHopDistance(string hopdate) {
    //loads the average hop count of one map.
    string fname = _path + hopdate + _locoptname; //_results_dir + "maps/"
    FILE * fp = OpenFile(fname.c_str(), "r", false);
    double avg_hop_distance = 0.0;
    if(fp) {
        int LINESIZE = 10000;
        char line[LINESIZE];
        fgets(line, LINESIZE-1, fp);
        if(sscanf(line, "%lf\n", &avg_hop_distance)!=1) {
            std::cout << "HopcountLog::LoadHopDistance(). Error scanning the file: " << fname << std::endl;
            exit(-1);
        }
        fclose(fp);
    }
    
    return avg_hop_distance;
}

std::vector<double> HopcountLog::LoadPriorRerror(std::string hopdate, int count){
    std::vector<double> rerror_set(count, 0.0);
    string fname = _path + hopdate + _locoptname;
    FILE * fp = OpenFile(fname.c_str(), "r", false);
    int idx = 0;
    if(fp){
        int a, b, rerr;
        double c;
        int LINESIZE = 10000;
        char line[LINESIZE];
        fgets(line, LINESIZE-1, fp);//skip the first line
        while(fgets(line, LINESIZE-1, fp)){
            
            int ret = sscanf(line, "%d, %d, %d, %lf", &a, &b, &rerr, &c);
            if(ret != 4){
                std::cout << "HopcountLog::LoadPriorRerror() error. Line formatting. " << ret << " entries in "<< line << std::endl;
                exit(-1);
            }
            rerror_set[idx++] = (double) rerr;
        }
        fclose(fp);
        if(count != idx){
            std::cout << "HopcountLog::LoadPriorRerror() error. The number of lines in the file is unexpected. Got " << idx << " of " << count  << std::endl;
            return std::vector<double>(count, 0.0);
//            exit(-1);
        }
    }
    return rerror_set;
}

std::vector<double> HopcountLog::GetAvgHopCounts() {
    //returns the average hop count of all previous maps.
    //string dir = _results_dir + "maps/";
    std::vector<double> ret;
    if(FileParsing::DirectoryExists(_path)) {
        vector<string> dates = FileParsing::ListFilesInDir(_path, "1");
        for(int i=0; i<dates.size(); i++) {
            double avghop = LoadHopDistance(dates[i]);
            ret.push_back(avghop);
        }
    }
    return ret;
}

void HopcountLog::SaveLocalLog(string hopdate, int numverified, std::vector<LocalizedPoseData>& localizations, std::vector<double> lpd_rerror) {
    std::vector<double> hopcounts = GetAvgHopCounts();
    double sum = 0.0;
    int count = 0;
    for(int i=0; i<localizations.size(); i++){
        if(lpd_rerror[i]<=0) continue;
        count++;
        sum += 0;//hopcounts[localizations[i].s0]; //
    }
    double avg_hop_distance = 1 + sum/count;
    
    string fname = _path + hopdate + _locoptname; //_results_dir + _date
    FILE * fp = OpenFile(fname.c_str(), "w");
    fprintf(fp, "%lf\n", avg_hop_distance);
    int countout = 0;
    for(int i=0; i<localizations.size(); i++){
        int verified = (i < numverified)?1:0;
        fprintf(fp, "%d, %d, %d, %lf\n", localizations[i].s1time, verified, (int)lpd_rerror[i], 0.0);//1 + hopcounts[localizations[i].s0]
    }
    
    fflush(fp);
    fclose(fp);
}

bool HopcountLog::LogExists(std::string basepath, std::string hopdate){
    return FileParsing::Exists(basepath + hopdate + _locoptname);
}














