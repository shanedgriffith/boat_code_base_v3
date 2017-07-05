/*
 * LocalizePose.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: shane
 */

#include "LPDInterface.hpp"

int LPDInterface::GetLPDIdx(int por1time){
    auto search = lpdtable.find(por1time);
    if(search != lpdtable.end()){
        return search->second;
    }
    return -1;
}

bool LPDInterface::StoreLPD(std::string path, LocalizedPoseData lpd){
    if(GetLPDIdx(lpd.s1time) < 0){
        localizations.push_back(lpd);
        lpdtable[lpd.s1time] = localizations.size()-1;
        lpd.Save(path + lpd.date1);
        return true;
    }
    return false;
}

LocalizedPoseData* LPDInterface::NearestLPD(int s1time){
    std::vector<int> dir = {-1, 1};
    for(int i=2; i<20; i++){
        int cur = dir[i%2]*(i/2) + s1time;
        if(cur < 0) continue;
        int idx = GetLPDIdx(cur);
        if(idx>=0) return &(localizations[idx]);
    }
    return NULL;
}

int LPDInterface::LoadLocalizations(std::string path){
    localizations = LocalizedPoseData::LoadAll(path); //load existing data.
    if(localizations.size()>0) most_adv_lpd = localizations[localizations.size()-1];
    for(int i=0; i<localizations.size(); i++)
        lpdtable[localizations[i].s1time] = i;
    return localizations.size();
}

int LPDInterface::GetStartingPoint(){
    if(most_adv_lpd.IsSet())
        return most_adv_lpd.s1time + 1;
    return 0;
}

int LPDInterface::GetStartingPoint(FROM u, DIRECTION d){
    int lcuridx = std::max(((int)localizations.size()-1),0);
    int direc = (d==DIRECTION::Forward)?-1:1;
    int ref = (u==FROM::CurLPD)?0:1;
    return localizations[lcuridx - ref].s1time - direc;
}

void LPDInterface::SetMostAdvLPD(LocalizedPoseData lpd){
    if(most_adv_lpd.s1time < lpd.s1time)
        most_adv_lpd = lpd;
}

//most_adv_lpd (should this logic be part of this interface?)






















































