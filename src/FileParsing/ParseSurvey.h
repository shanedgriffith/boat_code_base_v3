//
//  ParseSurvey.h
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SRC_FILEPARSING_PARSESURVEY_H_
#define SRC_FILEPARSING_PARSESURVEY_H_

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
    
    int IdentifyFirstPFTFileAtOrAfter(int index);
    int IdentifyLastPFTFile();
    double AngleDistance(double a, double b);
    
    std::vector<std::vector<double> > poses;
    
    virtual void ProcessLineEntries(int type, std::vector<std::string>& lp) = 0;
    virtual void ReadDelimitedFile(std::string file, int type) = 0;
public:
    std::string _base;
    std::string _pftbase;
    std::string _date;
    static const std::string _auxfile;
    std::vector<double> timings;
    
    ParseSurvey(std::string base, std::string pftbase, std::string date):
    _base(base), _pftbase(pftbase), _date(date) {}
    
    virtual int GetImageNumber(int auxidx) = 0;
    virtual int GetIndexOfImage(int image) = 0;
    virtual double GetAvgAngularVelocity(int sidx, int eidx) = 0;
    virtual bool Useable(int cidx, int lcidx){return true;}
    virtual std::vector<double> GetDrawScale()=0;
    
    void SetPFTBase(std::string pftbase){_pftbase = pftbase;}
    bool CheckGap(int last_auxidx, int next_auxidx);
    static int GetImageNumberFromImagePath(std::string imagepath);
    static std::string GetImagePath(std::string base, int no, bool makedir=false);
    
    bool ConstantVelocity(){return constant_velocity;}
    int NumPoses() {return static_cast<int>(poses.size());}
    std::vector<double> GetPose(int i);
    std::vector<std::vector<double> >& Poses(){return poses;}
    gtsam::Pose3 CameraPose(int idx);
    
    bool CheckCameraTransition(int cidx, int lcidx);
    int FindSynchronizedAUXIndex(double querytime, int from_idx);
    ParseFeatureTrackFile LoadVisualFeatureTracks(Camera& _cam, int& index, bool gap = false);
    
};




#endif /* SRC_FILEPARSING_PARSESURVEY_H_ */
