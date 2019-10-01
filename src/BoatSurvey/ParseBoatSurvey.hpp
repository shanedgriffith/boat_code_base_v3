//
//  ParseBoatSurvey.h
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SRC_BOATSURVEY_PARSEBOATSURVEY_HPP_
#define SRC_BOATSURVEY_PARSEBOATSURVEY_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <FileParsing/ParseFeatureTrackFile.h>
#include <DataTypes/Camera.hpp>
#include <FileParsing/ParseSurvey.h>

class ParseBoatSurvey: public ParseSurvey{
protected:
    std::vector<double> GetRotationMatrix(double X, double Y, double Z);
    std::vector<double> ComposeRotationMatrices(std::vector<double> A, std::vector<double> B);
    std::vector<double> RotationMatrixToRPY(std::vector<double> R);
    std::vector<double> GetCameraPose(double x, double y, double theta, double camera_pan, double tilt);
    
    std::vector<double> omega;
    std::vector<double> cam_pan;
    std::vector<int> imageno;

    void ProcessLineEntries(int type, std::vector<std::string>& lp);
    void ReadDelimitedFile(std::string file, int type);
public:
    
    ParseBoatSurvey(std::string base, std::string pftbase, std::string date):
    ParseSurvey(base, pftbase, date) {
        constant_velocity = true;
        default_start = gtsam::Point3(296866.7554855264, 5442698.88922645, 0);
        
        ReadDelimitedFile(_base + date + _auxfile, 0);
        
        if(timings.size() == 0) {
            std::cout << "ParseBoatSurvey(): Survey Error. The AUX file is empty. path: " << _base + _auxfile << std::endl;
        }
    }
    
    static Camera GetCamera();
    int GetImageNumber(int auxidx);
    int GetIndexOfImage(int image);
    double GetAvgAngularVelocity(int sidx, int eidx);
    bool Useable(int cidx, int lcidx);
    std::vector<double> GetDrawScale();
    void PlayPoses();
};

#endif /* SRC_BOATSURVEY_PARSEBOATSURVEY_HPP_ */









