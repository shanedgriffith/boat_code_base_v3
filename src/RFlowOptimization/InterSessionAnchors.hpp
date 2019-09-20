#pragma once


#include <gtsam/geometry/Pose3.h>

#include <iostream>
#include <fstream>

#include "FileParsing/FileParsing.hpp"

class InterSessionAnchors: public FileParsing {
private:
    static const std::string anchorpath_;
    
    std::string getFileName();
    
public:
    
    std::string sessionj_, sessionk_;
    std::vector<gtsam::Pose3> anchorkj;
    std::vector<int> kindices;
    std::vector<int> jindices;
    
    InterSessionAnchors(std::string sessionj, std::string sessionk) :
    sessionj_(sessionj), sessionk_(sessionk) {}
    
    void writeAnchors(std::string directory);
    
    void readAnchors(std::string directory);
    
    void addAnchor(gtsam::Pose3 anchor, int j, int k);
    
    gtsam::Pose3 getNearestAnchor(int index, bool sessionj);
};
