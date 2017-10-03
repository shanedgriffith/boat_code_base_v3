/*
 * EfficientMultiSessionOptimization.hpp
 *
 *  Created on: Oct 3, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_EFFICIENTMULTISESSIONOPTIMIZATION_HPP_
#define SRC_RFLOWOPTIMIZATION_EFFICIENTMULTISESSIONOPTIMIZATION_HPP_


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>
//
//#include <Optimization/SurveyOptimizer.h>
//#include <Optimization/GTSamInterface.h>
//#include <FileParsing/ParseOptimizationResults.h>
//#include <DataTypes/Camera.hpp>
//
//#include "RFlowFactorGraph.hpp"
//#include "LPDInterface.hpp"
//#include "LocalizedPoseData.hpp"
#include "MultiSessionOptimization.hpp"

//extend MSO?
/*GOAL:
  To work like MSO, but eliminate pose-graph variables if they are part of an ISC.
 Also works somewhat like the anchor approach, in that the map is resolved after the optimization.
 */
class EfficientMultiSessionOptimization: public MultiSessionOptimization {
protected:
    
//    int FindLandmarkRange(std::vector<LandmarkTrack>& landmarks, int ckey, bool end);
    void ToggleLandmarksAtPose(int survey, int ckey, bool active);
    void TestLandmarkRange();
    
    void ConstructFactorGraph();
    std::vector<std::vector<std::vector<double> > > GetPoses();
    double UpdateErrorAdaptive(bool firstiter);
public:
    
    EfficientMultiSessionOptimization(Camera& cam, std::string results_dir, std::string pftbase, std::string date = "");
    
//    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_EFFICIENTMULTISESSIONOPTIMIZATION_HPP_ */
