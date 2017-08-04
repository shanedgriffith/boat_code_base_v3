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
    //given that they're sorted, starting the iteration from the end points the
    //hash table at the correct ones.
    lpdtable.reserve(localizations.size());
    for(int i=localizations.size()-1; i>=0; i--)
        lpdtable[localizations[i].s1time] = i;
    return localizations.size();
}

int LPDInterface::GetStartingPoint(){
    if(most_adv_lpd.IsSet())
        return most_adv_lpd.s1time + 1;
    return 0;
}

int LPDInterface::GetStartingPoint(int lcuridx, FROM u, DIRECTION d){
    int direc = (d==DIRECTION::Forward)?-1:1;
    int ref = localizations[lcuridx].s1time;
    //in case there are multiple localizations at s1time, loop to find the next one.
    int i=0;
    if(u!=FROM::CurLPD)
        for(; localizations[lcuridx-i].s1time == ref; i++);
    return localizations[lcuridx-i].s1time - direc;
}

void LPDInterface::SetMostAdvLPD(LocalizedPoseData lpd){
    if(most_adv_lpd.s1time < lpd.s1time)
        most_adv_lpd = lpd;
}






















































