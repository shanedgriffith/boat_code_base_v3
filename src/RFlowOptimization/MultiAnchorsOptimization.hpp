/*
 * MultiAnchorsOptimization.hpp
 *
 *  Created on: Aug 6, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_MULTIANCHORSOPTIMIZATION_HPP_
#define SRC_RFLOWOPTIMIZATION_MULTIANCHORSOPTIMIZATION_HPP_


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

#include <Optimization/SurveyOptimizer.h>
#include <Optimization/GTSamInterface.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <DataTypes/Camera.hpp>

#include "RFlowFactorGraph.hpp"
#include "LPDInterface.hpp"
#include "LocalizedPoseData.hpp"

class MultiAnchorsOptimization: public SurveyOptimizer {
protected:
    
public:
    std::string _map_dir;
    std::string _pftbase;
    
    MultiAnchorsOptimization(Camera& cam, std::string results_dir, std::string pftbase, std::string date = "");
    
    ~MultiAnchorsOptimization(){
        //delete(rfFG);
    }
    
};


#endif /* SRC_RFLOWOPTIMIZATION_MULTIANCHORSOPTIMIZATION_HPP_ */
