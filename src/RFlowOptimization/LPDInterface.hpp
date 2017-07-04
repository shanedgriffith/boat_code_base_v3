/*
 * LocalizePose.hpp
 *
 *  Created on: Jan 17, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_LOCALIZEPOSE_HPP_
#define SRC_RFLOWOPTIMIZATION_LOCALIZEPOSE_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <RFlowOptimization/LocalizedPoseData.hpp>

class LPDInterface{
protected:
    std::unordered_map<int, int> lpdtable;
public:
    bool debug = false;
    
    enum class DIRECTION{Backward, Forward};
    enum class FROM{CurLPD, LastLPD};
    
    std::vector<LocalizedPoseData> localizations;
    LocalizedPoseData most_adv_lpd;
    
    LPDInterface(){}
    
    int GetLPDIdx(int por1time);
    bool StoreLPD(LocalizedPoseData lpd);
    LocalizedPoseData* NearestLPD(int s1time);
    int LoadLocalizations(std::string path);
    int GetStartingPoint();
    int GetStartingPoint(FROM u, DIRECTION d);
    void SetMostAdvLPD(LocalizedPoseData lpd);
};



#endif /* SRC_RFLOWOPTIMIZATION_LOCALIZEPOSE_HPP_ */
