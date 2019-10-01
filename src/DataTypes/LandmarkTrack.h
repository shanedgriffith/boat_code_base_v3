//
//  BAOne.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 5/23/14.
//  Copyright (c) 2014 shane. All rights reserved.
//

#ifndef SRC_DATATYPES_LANDMARKTRACK_H_
#define SRC_DATATYPES_LANDMARKTRACK_H_

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Symbol.h>

class LandmarkTrack {
public:
    int key;
    bool used;
    std::vector<gtsam::Point2> points;
    std::vector<gtsam::Symbol> camera_keys;
    
    LandmarkTrack(int k, bool touse=true):key(k), used(touse){}
    
    void AddToTrack(gtsam::Point2 p, int survey, int ckey) {
        points.push_back(p);
        gtsam::Symbol S((char) survey, ckey);
        camera_keys.push_back(S);
    }
    
    gtsam::Point2 GetCoord(int ckey){
        if(camera_keys.size() == 0) return gtsam::Point2(-1,-1);
        int s = static_cast<int>(camera_keys[0].index());
        int e = static_cast<int>(s + camera_keys.size());
        if(ckey < s || ckey > s+e) return gtsam::Point2(-1,-1);
        return points[ckey-s];
    }
    
    int GetKey(){return key;}
    int Length() {return static_cast<int>(points.size());}
};



#endif /* SRC_DATATYPES_LANDMARKTRACK_H_ */
