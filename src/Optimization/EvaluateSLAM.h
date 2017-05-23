//
//  EvaluateSLAM.h
//  SIFTFlow
//
//  Created by Shane Griffith on 1/28/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __SIFTFlow__EvaluateSLAM__
#define __SIFTFlow__EvaluateSLAM__

#include <dirent.h>
#include <stdio.h>
#include <iostream>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <DataTypes/Camera.hpp>
#include <FileParsing/FileParsing.hpp>

extern const std::string base;
extern const std::string siftloc;

class EvaluateSLAM: public FileParsing {
protected:
    static const std::string reprofile;
    
    std::string formattime(double seconds);
    
    Camera& _cam;
public:
    bool debug;
    double badthreshold; //values are for display only.
    double avgbadthreshold;
    std::string _date;
    
    EvaluateSLAM(Camera& cam, std::string date):
        debug(false), badthreshold(50), avgbadthreshold(15) , _cam(cam), _date(date){}
    
    void Evaluate();

    std::vector<double> MeasureReprojectionError(std::vector<double>& boat, std::vector<gtsam::Point2>& orig_imagecoords, std::vector<gtsam::Point3>& p);
    std::vector<double> ErrorForSurvey();
    void SaveEvaluation(std::vector<double> evaluation, std::string altname="");
    static std::vector<double> LoadRerrorFile(std::string base, std::string date);
};

#endif /* defined(__SIFTFlow__EvaluateSLAM__) */
