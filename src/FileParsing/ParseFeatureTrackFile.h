//
//  ParseFeatureTrackFile.h
//  VisualizationCode
//
//  Created by Shane Griffith on 6/10/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __VisualizationCode__ParseFeatureTrackFile__
#define __VisualizationCode__ParseFeatureTrackFile__

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"

#include <DataTypes/Camera.hpp>

#include "FileParsing.hpp"

class ParseFeatureTrackFile: public FileParsing{
private:
    int _no;
    std::string _base;
    
    void ReadNewTypeFile(std::string file);
    void Load();
    
	Camera& _cam;
protected:
    void ProcessLineEntries(int type, std::vector<std::string> lp);
    void ReadDelimitedFile(std::string file, int type);
public:
    std::string siftfile;
    std::vector<int> ids;
    std::vector<gtsam::Point2> imagecoord;
    double time;
    
    /*Ended up reverting to something like this due to the implicit copy problem. I got 
     around it using pointers, but that approach was questionable. There's probably a 
     better way to do this (smart pointers?).
     */
    ParseFeatureTrackFile(Camera& cam): _cam(cam), time(-1) {}
    
    ParseFeatureTrackFile(Camera& cam, std::string base, int no):
    	_cam(cam), _no(no), _base(base), time(-1)
    {
        siftfile = GetFeatureTrackFilePath(base, no);
        Load();
    }
    
    //this constructor is needed in case the file is downloaded and not in its usual location.
    ParseFeatureTrackFile(Camera& cam, std::string file) :
    _cam(cam), time(-1), siftfile(file)
    {
        Load();
    }
    void Reset();
    void Next(int no);
    int GetIndexOfPoint(int point_id);
    static std::string GetFeatureTrackFilePath(std::string base, int no, bool makedir=false);
    
    double GetDisplacementFrom(ParseFeatureTrackFile& compared);
    gtsam::Point2 GetAverageOpticalFlowFrom(ParseFeatureTrackFile& compared);
    void WriteFeatureTrackFile();
    void SetPFTContents(std::string base, int ftnum, double timestamp, std::vector<int>& _ids, std::vector<cv::Point2f>& _points);
};



#endif /* defined(__VisualizationCode__ParseFeatureTrackFile__) */
