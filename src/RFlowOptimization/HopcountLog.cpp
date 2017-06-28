//
//  ParseSurvey.cpp
//  VisualizationCode
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
    FILE * fp = OpenFile(fname.c_str(), "r");
    double avg_hop_distance = 0.0;
    if(fp) {
        int LINESIZE = 10000;
        char line[LINESIZE]="";
        fgets(line, LINESIZE-1, fp);
        if(sscanf(line, "%lf\n", &avg_hop_distance)!=1) {
            std::cout << "RFlowSurveyOptimizer::LoadHopDistance(). Error scanning the file: " << fname << std::endl;
            exit(-1);
        }
        fclose(fp);
    }
    
    return avg_hop_distance;
}

std::vector<double> HopcountLog::GetAvgHopCounts() {
    //returns the average hop count of all previous maps.
    //string dir = _results_dir + "maps/";
    std::vector<double> ret;
    if(FileParsing::DirectoryExists(_path)) {
        vector<string> dates = FileParsing::ListFilesInDir(_path, "1");
        for(int i=0; i<dates.size(); i++) {
            double avghop = LoadHopDistance(_path, dates[i]);
            ret.push_back(avghop);
        }
    }
    return ret;
}

void HopcountLog::SaveLocalLog(string hopdate, int numverified, std::vector<LocalizedPoseData>& localizations) {
    std::vector<double> hopcounts = GetAvgHopCounts(_path);
    double sum = 0.0;
    int count = 0;
    for(int i=0; i<localizations.size(); i++){
        if(lpd_rerror[i]<=0) continue;
        count++;
        sum += hopcounts[localizations[i].s0];
    }
    double avg_hop_distance = 1 + sum/count;
    
    string fname = _path + hopdate + _locoptname; //_results_dir + _date
    FILE * fp = OpenFile(fname.c_str(), "w");
    fprintf(fp, "%lf\n", avg_hop_distance);
    int countout = 0;
    for(int i=0; i<localizations.size(); i++){
        int verified = (i < numverified)?1:0;
        fprintf(fp, "%d, %d, %d, %lf\n", localizations[i].s1time, verified, (int)lpd_rerror[i], 1 + hopcounts[localizations[i].s0]);
    }
    
    fflush(fp);
    fclose(fp);
}
















