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
#include <DataTypes/LandmarkTrack.h>

#include <Optimization/SingleSession/EvaluateSLAM.h>
#include <FileParsing/ParseOptimizationResults.h>
#include "LocalizedPoseData.hpp"

class EvaluateRFlow: public EvaluateSLAM {
protected:
    std::vector<gtsam::Point3> GetSubsetOf3DPoints(const std::vector<std::vector<double> >& landmarks, const std::vector<int>& ids_subset);
    int GetIndexOfFirstPoint(const std::vector<std::vector<double> >& landmarks, int id);
    
public:
    EvaluateRFlow(Camera& cam, std::string date, std::string results_dir):
    EvaluateSLAM(cam, date, results_dir) {}
    
    double InterSurveyErrorAtLocalization(const std::vector<double>& boat, const std::vector<std::vector<double> >& landmarks, const std::vector<gtsam::Point2>& p2d1, const std::vector<int> pids, const std::vector<double> rerrorp);
    
    double OnlineRError(std::vector<LandmarkTrack>& cached_landmarks, int idx, const std::vector<double>& pose, const std::vector<std::vector<double> >& landmarks);
    
    void VisualizeDivergenceFromLocalizations(std::vector<LocalizedPoseData>& localizations, std::vector<double>& error);
    void VisualizeFrameChange(std::vector<std::vector<double> >& traj, std::vector<LocalizedPoseData>& localizations);
};



#endif /* SRC_RFLOWOPTIMIZATION_EVALUATERFLOW_HPP_ */
