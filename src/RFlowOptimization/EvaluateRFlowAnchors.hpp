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

#include <RFlowOptimization/EvaluateRFlow.hpp>
#include <FileParsing/ParseOptimizationResults.h>
#include "LocalizedPoseData.hpp"

class EvaluateRFlowAnchors: public EvaluateRFlow {
protected:
    double ComputeReprojectionError(std::vector<std::vector<double>> p_subset);
    std::vector<std::vector<double>> GetSubsetOfPoints(const std::vector<std::vector<double> >& landmarks, const std::vector<int>& ids_subset);
    int GetIndexOfFirstPoint(const std::vector<std::vector<double> >& landmarks, int id);
    double ComputeNewReprojectionError(std::vector<double>& anchor, std::vector<double>& pose, const std::vector<std::vector<double> >& landmarks, ParseFeatureTrackFile& PFT);
    
public:
    EvaluateRFlowAnchors(Camera& cam, std::string date):
    EvaluateRFlow(cam, date, "") {}
    
    double InterSurveyErrorAtLocalization(const LocalizedPoseData& localization, const std::vector<std::vector<std::vector<double> > >& landmarks={});
    double OnlineRError(ParseOptimizationResults& POR, int idx, std::string _pftset, const std::vector<std::vector<double> >& landmarks);
    double ComputeAnchorRError(std::vector<double>& anchor, ParseOptimizationResults& POR, int idx, std::string _pftset, const std::vector<std::vector<double> >& landmarks);
};



#endif /* SRC_RFLOWOPTIMIZATION_EVALUATERFLOWANCHORS_HPP_ */
