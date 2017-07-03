//
//  BAOne.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 5/23/14.
//  Copyright (c) 2014 shane. All rights reserved.
//

#ifndef __BundleAdjustOneDataset__BAOne__
#define __BundleAdjustOneDataset__BAOne__

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>

class LandmarkTrack {
public:
    int key;
    bool used;
    std::vector<double> active_times;
    std::vector<gtsam::Point2> points;
    std::vector<gtsam::Symbol> camera_keys;
    
    LandmarkTrack(int k, bool touse=true):key(k), used(touse), hasSymbols(false){}
    
    void AddToTrack(double t, gtsam::Point2 p, int survey, int ckey) {
        active_times.push_back(t);
        points.push_back(p);
        gtsam::Symbol S((char) survey, ckey);
        camera_keys.push_back(S);
    }
    
    int GetKey(){return key;}
    int Length() {return active_times.size();}
};



#endif /* defined(__BundleAdjustOneDataset__BAOne__) */
