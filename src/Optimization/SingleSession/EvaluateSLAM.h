//
//  EvaluateSLAM.h
//  SIFTFlow
//
//  Created by Shane Griffith on 1/28/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#pragma once

#include <dirent.h>
#include <stdio.h>
#include <iostream>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <DataTypes/Camera.hpp>
#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseOptimizationResults.h>

class EvaluateSLAM: public FileParsing {
protected:
    static const std::string reprofile;
    
    double UpdateTots(std::vector<double>& stats);
    
    std::vector<double> tots;
    const Camera& _cam;
public:
    bool debug;
    double badthreshold; //values are for display only.
    double avgbadthreshold;
    std::string _date;
    std::string _results_dir;
    
    EvaluateSLAM(const Camera& cam, std::string date, std::string results_dir):
        debug(false), badthreshold(50), avgbadthreshold(15), tots(7, 0.0),
    _cam(cam), _date(date), _results_dir(results_dir){}
    
    std::vector<double> ErrorForSurvey(std::string _pftbase, bool save = false);
    
    double MeasureReprojectionError(const std::vector<double>& boat, const std::vector<gtsam::Point2>& orig_imagecoords, const std::vector<gtsam::Point3>& p, const std::vector<double>& rerror = {});
    double OfflineRError(ParseOptimizationResults& POR, int idx, std::string _pftbase);
    
    void SaveEvaluation(std::vector<double> evaluation, std::string altname="");
    std::vector<double> LoadRerrorFile();
    double GetAverageRerror(std::vector<double> rerrors);
    void PrintTots(std::string name = "", bool compact = false);
};
