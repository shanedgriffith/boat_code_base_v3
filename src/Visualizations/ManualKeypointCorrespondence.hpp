//
//  ManualKeypointCorrespondence.hpp
//  boat_code_base_v2
//
//  Created by Shane on 1/11/19.
//  Copyright © 2019 Shane. All rights reserved.
//

#pragma once

#include <stdio.h>
#include <string>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <DataTypes/Map.hpp>
#include <DataTypes/Camera.hpp>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


class ManualKeypointCorrespondence {
private:
    void AddPair();
    void UpdateProjection(cv::Mat imgr, cv::Mat imgt);
    void UpdateLocalizedPose();
    bool ProcessLabel(char c);
    void RedrawImages(cv::Mat img);
    void SortPoints();
    void SaveCorrespondences();
    std::string GetFilename(std::string d0, int p0, std::string d1, int p1);
    
    bool accepted;
    std::string _d0, _d1;
    int _p0, _p1;
    ParseOptimizationResults _por0, _por1;
    ParseFeatureTrackFile _pftf;
    Map _m;
    bool _show_preview;
    cv::Mat _preview;
    
    int leftpidx;
    cv::Point2i right;
    bool redraw;
    
    std::vector<double> _lpose;
    std::vector<int> indices;
    std::vector<cv::Point2i> correspondences;
    int w, h;
    Camera& _cam;
public:
    std::string _query_loc, _pftbase, _map_base, _save_dir;
    ManualKeypointCorrespondence(Camera& cam, std::string query_loc, std::string pftbase, std::string map_base, std::string save_dir) :
    _cam(cam), _m(map_base), _pftf(cam), _query_loc(query_loc), _pftbase(pftbase), _map_base(map_base), _save_dir(save_dir), leftpidx(-1), right(cv::Point2f(-1,-1)), accepted(false), _show_preview(false) {};
    
    void RunManualCorrespondence(std::string d0, int p0, std::string d1, int p1);
    
    void ShowImagePair(cv::Mat& im1, cv::Mat& im2);
    
    int GetNumCorrespondences(){return correspondences.size();}
    
    std::string CorrespondencesToString();
    
    void SetPoint(int x, int y);
    
    void GetLocalizationList();
    
};








