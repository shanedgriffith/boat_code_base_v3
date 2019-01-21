//
//  ManualImageCorrespondence.hpp
//  boat_code_base_v2
//
//  Created by Shane on 1/11/19.
//  Copyright © 2019 Shane. All rights reserved.
//

#pragma once

#include <stdio.h>
#include <string>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


class ManualImageCorrespondence {
private:
    void AddPair();
    bool ProcessLabel(char c);
    void RedrawImages(cv::Mat img);
    
    cv::Point2f left;
    cv::Point2f right;
    bool redraw;
    
    std::vector<std::vector<cv::Point2f> > correspondences;
    int w, h;
public:
    std::string _query_loc, _map_base, _save_dir;
    ManualImageCorrespondence(std::string query_loc, std::string map_base, std::string save_dir) :
    _query_loc(query_loc), _map_base(map_base), _save_dir(save_dir), left(cv::Point2f(-1,-1)), right(cv::Point2f(-1,-1)) {};
    
    void RunManualCorrespondence(std::string d0, int p0, std::string d1, int p1);
    
    void ShowImagePair(cv::Mat& im1, cv::Mat& im2);
    
    int GetNumCorrespondences(){return correspondences.size();}
    
    std::string CorrespondencesToString();
    
    void SetPoint(int x, int y);
    
    
};




