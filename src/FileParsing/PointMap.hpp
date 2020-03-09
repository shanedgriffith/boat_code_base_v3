//
//  PointMap.hpp
//  SIFTFlow
//
//  Created by Shane Griffith on 2/20/16.
//  Copyright © 2016 shane. All rights reserved.
//
#pragma once


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include "FileParsing.hpp"
#include "ParseOptimizationResults.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

class PointMap: public FileParsing{
private:
    double ComputeVariance(std::vector<double>& list);
protected:
    void ProcessLineEntries(int type, std::vector<std::string> lp);
    void ReadDelimitedFile(std::string file, int type);
public:
    std::string _pointmapfile;
    bool debug = false;
    std::string _base;
    //possible date.
    std::vector<double> variance;
    std::vector<int> ids;
    std::vector<gtsam::Point3> landmark;
    
    PointMap(std::string base, bool read = false) {
        _base = base;
        _pointmapfile = _base + "/point_map.csv";
        if(debug) std::cout << "file: " << _pointmapfile << std::endl;
        if(read && Exists(_pointmapfile)) ReadDelimitedFile(_pointmapfile, 0);
    }
    
    void SetIDs(int first, int last, ParseOptimizationResults& por);
    
    void SetVariances(std::vector<std::vector<double> >& variance_list);
    
    void WritePoints();
};

