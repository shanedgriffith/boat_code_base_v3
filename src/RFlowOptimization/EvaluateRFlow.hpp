/*
 * EvaluateRFlow.hpp
 *
 *  Created on: Feb 16, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_EVALUATERFLOW_HPP_
#define SRC_RFLOWOPTIMIZATION_EVALUATERFLOW_HPP_

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

class EvaluateRFlow: public EvaluateSLAM {
protected:
    std::vector<gtsam::Point3> GetSubsetOf3DPoints(std::vector<std::vector<double> >& landmarks, std::vector<int>& ids_subset);
    int GetIndexOfFirstPoint(std::vector<std::vector<double> >& landmarks, int id);
    
public:
    std::string _results_dir;
    EvaluateRFlow(Camera& cam, std::string date, std::string results_dir):
    _results_dir(results_dir), 
    EvaluateSLAM(cam, date, results_dir) {}
    
    void Evaluate();
    
    std::vector<double> InterSurveyErrorAtLocalization(LocalizedPoseData& localization, std::vector<double>& boat, std::vector<std::vector<std::vector<double> > >& landmarks = {}, int optstart = INT_MAX);
    
    double OnlineRError(ParseOptimizationResults& POR, int idx, std::string _pftbase, const std::vector<double>& pose, const std::vector<std::vector<double> >& landmarks);
    
    void VisualizeDivergenceFromLocalizations(std::vector<LocalizedPoseData>& localizations, std::vector<double>& error);
    void VisualizeFrameChange(std::vector<std::vector<double> >& traj, std::vector<LocalizedPoseData>& localizations);
};



#endif /* SRC_RFLOWOPTIMIZATION_EVALUATERFLOW_HPP_ */
