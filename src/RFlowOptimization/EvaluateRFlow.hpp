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

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <DataTypes/Camera.hpp>

#include <Optimization/EvaluateSLAM.h>
#include "LocalizedPoseData.hpp"

class EvaluateRFlow: public EvaluateSLAM {
private:
    std::vector<double> MeasureReprojectionError(std::vector<double>& boat, std::vector<gtsam::Point2>& orig_imagecoords, std::vector<gtsam::Point3>& p, std::vector<double>& rerror);
    std::vector<double> MeasureReprojectionError(gtsam::Pose3 tf, std::vector<gtsam::Point2>& orig_imagecoords, std::vector<gtsam::Point3>& p, std::vector<double>& rerror);
    double UpdateTots(std::vector<double>& tots, std::vector<double>& stats);
    void PrintTots(std::vector<double> tots, std::string name);
    vector<gtsam::Point3> GetSubsetOf3DPoints(std::vector<std::vector<double> >& landmarks, vector<int>& ids_subset);
    int GetIndexOfFirstPoint(std::vector<std::vector<double> >& landmarks, int id);
public:
    std::string _results_dir;
    EvaluateRFlow(Camera& cam, std::string date, std::string results_dir):
    _results_dir(results_dir), 
    EvaluateSLAM(cam, date, results_dir) {}
    
    void Evaluate();
    
    std::vector<double> InterSurveyErrorAtLocalizations(std::vector<LocalizedPoseData>& localizations, std::vector<std::vector<double> >& traj);
    vector<double> EvaluateRFlow::InterSurveyErrorAtLocalizations(std::vector<LocalizedPoseData>& localizations, vector<vector<double> >& traj, std::vector<std::vector<std::vector<double> > >& landmarks, int optstart);
    std::vector<double> IntraSurveyErrorAtLocalizations(std::vector<LocalizedPoseData>& localizations, std::string _pftbase);
    vector<double> IntraSurveyErrorAtLocalizations(std::vector<std::vector<double> >& poses, std::vector<std::vector<double> >& landmarks, std::vector<LocalizedPoseData>& localizations, ParseOptimizationResults& POR, std::string _pftbase);
    void VisualizeDivergenceFromLocalizations(std::vector<LocalizedPoseData>& localizations, std::vector<double>& error);
    void VisualizeFrameChange(std::vector<std::vector<double> >& traj, std::vector<LocalizedPoseData>& localizations);
};



#endif /* SRC_RFLOWOPTIMIZATION_EVALUATERFLOW_HPP_ */
