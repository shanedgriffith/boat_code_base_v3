/*
 * EvaluateRFlowAnchors.hpp
 *
 *  Created on: Aug 18, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_EVALUATERFLOWANCHORS_HPP_
#define SRC_RFLOWOPTIMIZATION_EVALUATERFLOWANCHORS_HPP_

#include <dirent.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <cstdint>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <DataTypes/Camera.hpp>

#include <Optimization/EvaluateSLAM.h>
#include <FileParsing/ParseOptimizationResults.h>
#include "LocalizedPoseData.hpp"

class EvaluateRFlowAnchors: public EvaluateRFlow {
protected:
    double ComputeReprojectionError(std::vector<std::vector<double>> p_subset);
    std::vector<std::vector<double>> GetSubsetOfPoints(const std::vector<std::vector<double> >& landmarks, const vector<int>& ids_subset);
    int GetIndexOfFirstPoint(const std::vector<std::vector<double> >& landmarks, int id);
    
public:
    std::string _results_dir;
    EvaluateRFlowAnchors(Camera& cam, std::string date, std::string results_dir):
    EvaluateRFlow(cam, date, results_dir) {}
    
    double InterSurveyErrorAtLocalization(const LocalizedPoseData& localization, const std::vector<std::vector<std::vector<double> > >& landmarks={});
    double OnlineRError(ParseOptimizationResults& POR, int idx, std::string _pftset, const std::vector<std::vector<double> >& landmarks);
    double ComputeAnchorRError(vector<double>& anchor, ParseOptimizationResults& POR, int idx, std::string _pftset, const std::vector<std::vector<double> >& landmarks);
};



#endif /* SRC_RFLOWOPTIMIZATION_EVALUATERFLOWANCHORS_HPP_ */
