//
//  ParseSurvey.h
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __VisualizationCode__ParseSurvey__
#define __VisualizationCode__ParseSurvey__

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include "FileParsing.hpp"
#include "ParseFeatureTrackFile.h"
#include <DataTypes/Camera.hpp>

class ParseSurvey: public FileParsing{
protected:
    gtsam::Point3 default_start;
    
    bool constant_velocity;
    
    
    double AngleDistance(double a, double b);
    
    std::vector<std::vector<double> > poses;
    
    virtual void ProcessLineEntries(int type, std::vector<std::string>& lp) = 0;
    virtual void ReadDelimitedFile(std::string file, int type) = 0;
public:
    std::string _base;
    std::string _pftbase;
    static const std::string _auxfile;
    std::vector<double> timings;
    
    ParseSurvey(std::string base, std::string pftbase):
    _base(base), _pftbase(pftbase) {}
    
    virtual int GetImageNumber(int auxidx) = 0;
    virtual int GetIndexOfImage(int image) = 0;
    virtual double GetAvgAngularVelocity(int sidx, int eidx) = 0;
    virtual bool Useable(int idx){return true;}
    virtual std::vector<double> GetDrawScale()=0;
    
    void SetPFTBase(std::string pftbase){_pftbase = pftbase;}
    ParseFeatureTrackFile GetFeatureTrackFile(Camera& _cam, int imageno);
    
    static int GetImageNumberFromImagePath(std::string imagepath);
    static std::vector<double> PoseToVector(gtsam::Pose3& cam);
    static std::string GetImagePath(std::string base, int no, bool makedir=false);
    
    bool ConstantVelocity(){return constant_velocity;}
    int NumPoses() {return poses.size();}
    std::vector<double> GetPose(int i);
    std::vector<std::vector<double> >& Poses(){return poses;}
    gtsam::Pose3 CameraPose(int idx);
    
    bool CheckCameraTransition(int cidx, int lcidx);
    int FindSynchronizedAUXIndex(double querytime, int from_idx);
    ParseFeatureTrackFile LoadVisualFeatureTracks(Camera& _cam, int& index);
    
};




#endif /* defined(__VisualizationCode__ParseSurvey__) */
