//
//  SolveForMap.hpp
//
//
//  Created by Shane Griffith on 8/16/17.
//  Copyright (c) 2017 shane. All rights reserved.
//

#ifndef SRC_RFLOWOPTIMIZATION_SOLVEFORMAP_HPP_
#define SRC_RFLOWOPTIMIZATION_SOLVEFORMAP_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

#include <gtsam/nonlinear/Symbol.h>//using symbols to identify factors
#include <gtsam/base/debug.h>


#include <utility>
#include <FileParsing/ParseOptimizationResults.h>
#include <DataTypes/LandmarkTrack.h>
#include "Anchors.hpp"

//this code should be independent because it will be used in multiple places, not just the optimization step.
class SolveForMap {
private:
    gtsam::Pose3 VectorToPose(std::vector<double>& p);
    
    Camera& _cam;
public:
    
    SolveForMap(Camera cam): _cam(cam){}
    
    std::vector<double> GetPoint(ParseOptimizationResults& POR, Anchors& anchors, LandmarkTrack& landmark);
    std::vector<double> GetPoint(std::vector<std::vector<double> >& poses, LandmarkTrack& landmark);
};



#endif /* SRC_RFLOWOPTIMIZATION_SOLVEFORMAP_HPP_ */













