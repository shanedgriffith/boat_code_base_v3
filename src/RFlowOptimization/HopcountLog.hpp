//
//  HopcountLog.h
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SRC_RFLOWOPTIMIZATION_HOPCOUNTLOG_HPP_
#define SRC_RFLOWOPTIMIZATION_HOPCOUNTLOG_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <FileParsing/FileParsing.hpp>

#include "LocalizedPoseData.hpp"

class HopcountLog: public FileParsing{
protected:
    static const std::string _locoptname;
    
    void ProcessLineEntries(int type, std::vector<std::string>& lp){}
    void ReadDelimitedFile(std::string file, int type){}
public:
    std::string _path;
    
    HopcountLog(std::string basepath):
    _path(basepath) {}
    
    static bool LogExists(std::string basepath, std::string hopdate);
    double LoadHopDistance(std::string hopdate);
    std::vector<double> LoadPriorRerror(std::string hopdate, int count);
    std::vector<double> GetAvgHopCounts();
    void SaveLocalLog(std::string hopdate, int numverified, std::vector<LocalizedPoseData>& localizations, std::vector<double> lpd_rerror);
};




#endif /* SRC_RFLOWOPTIMIZATION_HOPCOUNTLOG_HPP_ */
