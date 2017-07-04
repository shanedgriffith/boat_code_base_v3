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
    std::vector<bool> constraint_on;
    
    LandmarkTrack(int k, bool touse=true):key(k), used(touse), hasSymbols(false){}
    
    void AddToTrack(gtsam::Point2 p, int survey, int ckey, bool use_constraint = true) {
        points.push_back(p);
        gtsam::Symbol S((char) survey, ckey);
        camera_keys.push_back(S);
        constraint_on.push_back(use_constraint);
    }
    
    void Toggle(int survey, int ckey){
        for(int i=camera_keys.size()-1; i>=0; i--){
            if(camera_keys[i].index()==ckey && camera_keys[i].chr() == (char) survey){
                constraint_on = !constraint_on;
                return;
            }
        }
        std::cout << "LandmarkTrack::Toggle() Error. The constraint wasn't found." << std::endl;
        exit(-1);
    }
    
    int GetKey(){return key;}
    int Length() {return points.size();}
};



#endif /* SRC_DATATYPES_LANDMARKTRACK_H_ */
