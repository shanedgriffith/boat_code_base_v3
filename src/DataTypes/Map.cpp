/*
 * Map.cpp
 *
 *  Created on: May 5, 2016
 *      Author: shane
 */

#include "Map.hpp"

#include <FileParsing/PointMap.hpp>

using namespace std;

void Map::LoadISCMap(vector<string>& dates){
    num_surveys = dates.size();

    cout << "Loading the globally consistent map." << endl;
    double sum = 0;
    for(int i=0; i<dates.size(); i++){
    	_dates.push_back(dates[i]);
        PointMap pm(_map_base + dates[i], true);
        int survey_label = stoi(dates[i]);

        for(int j=0; j<pm.ids.size(); j++){
            map.push_back(pm.landmark[j]);
            variances.push_back(pm.variance[j]);
            survey_labels.push_back(survey_label);
            landmark_ids.push_back(pm.ids[j]);
            sum += pm.variance[j];
        }
        cout << "    " << dates[i] << ": got "<< pm.landmark.size() << " points of " << pm.ids.size() << " total." << endl;
    }

    cout << "  Map summary:" << endl;
    cout << "    " <<map.size() << " Map points." <<endl;
    cout << "    " << sum/map.size() << " Average point variance"<<endl;
    cout << "    " << pow(sum/map.size(), 0.5) << " average reprojection error (roughly)" << endl;
    if(map.size() == 0) exit(1);
}

void Map::LoadMap(string date){
    ParseOptimizationResults pm(_map_base, date);
    int survey_label = stoi(date);
    
    for(int j=0; j<pm.p.size(); j++){
        if(pm.p[j].p.x()==0 && pm.p[j].p.y()==0 && pm.p[j].p.z()==0) continue;

        map.push_back(pm.p[j].p);
        variances.push_back(0);
        survey_labels.push_back(survey_label);
        landmark_ids.push_back(pm.p[j].p_id);
    }
}

/*This still have a way to utilize the reprojection error?*/
void Map::LoadStandardMap(vector<string>& dates){
	num_surveys = dates.size();

    cout << "Loading the map of regular optimization results." << endl;
    for(int i=0; i<dates.size(); i++){
        LoadMap(dates[i]);
    }

    cout << "  Map summary:" << endl;
    cout << "    " <<map.size() << " Map points." <<endl;
    if(map.size() == 0) exit(1);
}


void Map::LoadMap(vector<string>& dates, bool standard){
	if(standard)
		LoadStandardMap(dates);
	else
		LoadISCMap(dates);
}

bool Map::CheckSize(){
	if(map.size() == 0){
		cout << "Empty map. Something went wrong."<<endl;
		exit(-1);
	}
	return false;
}

int Map::NumSurveys(){
	return _dates.size();
}








































