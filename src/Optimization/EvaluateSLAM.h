//
//  EvaluateSLAM.h
//  SIFTFlow
//
//  Created by Shane Griffith on 1/28/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SRC_OPTIMIZATION_EVALUATESLAM_H_
#define SRC_OPTIMIZATION_EVALUATESLAM_H_

#include <dirent.h>
#include <stdio.h>
#include <iostream>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <DataTypes/Camera.hpp>
#include <FileParsing/FileParsing.hpp>

class EvaluateSLAM: public FileParsing {
protected:
    static const std::string reprofile;
    
    vector<double> tots(6, 0.0);
    Camera& _cam;
public:
    bool debug;
    double badthreshold; //values are for display only.
    double avgbadthreshold;
    std::string _date;
    std::string _results_dir;
    
    EvaluateSLAM(Camera& cam, std::string date, std::string results_dir):
        debug(false), badthreshold(50), avgbadthreshold(15),
    _cam(cam), _date(date), _results_dir(results_dir){}
    
    std::vector<double> ErrorForSurvey(std::string _pftbase, bool save = false);
    
    double MeasureReprojectionError(std::vector<double>& boat, std::vector<gtsam::Point2>& orig_imagecoords, std::vector<gtsam::Point3>& p, const std::vector<double>& rerror = {});
    
    double OfflineRError(ParseOptimizationResults& POR, int idx, std::string _pftbase);
    
    void SaveEvaluation(std::vector<double> evaluation, std::string altname="");
    std::vector<double> LoadRerrorFile();
    double GetAverageRerror(std::vector<double> rerrors);
};

#endif /* SRC_OPTIMIZATION_EVALUATESLAM_H_ */
