//
//  ManualImageCorrespondence.cpp
//  boat_code_base_v2
//
//  Created by Shane on 1/11/19.
//  Copyright © 2019 Shane. All rights reserved.
//

#include "ManualImageCorrespondence.hpp"
#include <Visualizations/FlickeringDisplay.h>
#include <FileParsing/FileParsing.hpp>
#include <Visualizations/IMDraw.hpp>
#include <FileParsing/ParseSurvey.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <ImageAlignment/DREAMFlow/ImageOperations.h>

static void
MouseFunc(int event, int x, int y, int flags, void* userdata) {
    ManualImageCorrespondence* mic = static_cast<ManualImageCorrespondence*>(userdata);
    if  ( event == cv::EVENT_LBUTTONDOWN ) {
        mic->SetPoint(x,y);
        std::cout << "mouse clicked " << x << ", " << y << std::endl;
    }
}

void
ManualImageCorrespondence::SetPoint(int x, int y) {
    if(x < w)
        left = cv::Point2f(x,y);
    else
        right = cv::Point2f(x-w,y);
    redraw = true;
}

void
ManualImageCorrespondence::AddPair() {
    if(left.x != -1 and right.x !=-1) {
        std::vector<cv::Point2f> pair = {left, right};
        correspondences.push_back(pair);
    }
    left = cv::Point2f(-1,-1);
    right = cv::Point2f(-1,-1);
}

bool
ManualImageCorrespondence::ProcessLabel(char c) {
    switch(c) {
        case 'q':
            exit(1);
            return false;
        case 's':
            AddPair();
            return false;
        case 'n':
            AddPair();
            return true;
        default:
            return true;
    }
    return true;
}

void
ManualImageCorrespondence::RedrawImages(cv::Mat img) {
    cv::Point2f ro(w,0);
    for(int i=0; i<correspondences.size(); ++i) {
        CvScalar color = IMDraw::GetLandmarkColor(i*10);
        circle(img, correspondences[i][0], 3, color, -1, 8, 0);
        circle(img, correspondences[i][1] + ro, 3, color, -1, 8, 0);
    }
    
    CvScalar defcolor = CV_RGB(255,255,255);
    if(left.x != -1) {
        circle(img, left, 5, defcolor, -1, 8, 0);
        circle(img, left, 4, CV_RGB(255,0,0), -1, 8, 0);
    }
    if(right.x != -1) {
        circle(img, right+ro, 5, defcolor, -1, 8, 0);
        circle(img, right+ro, 4, CV_RGB(255,0,0), -1, 8, 0);
    }
}

void
ManualImageCorrespondence::ShowImagePair(cv::Mat& im1, cv::Mat& im2) {
    w = im1.cols;
    h = im1.rows;
    
    cv::Mat comb = FlickeringDisplay::CombinedImage(im1, im2);
    
    std::string window_name = "manual correspondence";
    
    cv::namedWindow(window_name);
    cv::moveWindow(window_name, 0, 340);
    ManualImageCorrespondence * mic = this;
    cv::setMouseCallback(window_name, MouseFunc, (void *)(mic));
    bool cont = true;
    while(cont) {
        cv::Mat displayed = comb.clone();
        RedrawImages(displayed);
        cv::imshow(window_name, displayed);
        char c = cvWaitKey(30);
        cont = ProcessLabel(c);
    }
    std::cout << "finished labeling" << std::endl;
    cv::destroyWindow(window_name);
}

std::string
ManualImageCorrespondence::CorrespondencesToString() {
    std::string data = "";
    for(int i=0; i<correspondences.size(); ++i) {
        data += std::to_string(correspondences[i][0].x) + ", " +
                std::to_string(correspondences[i][0].y) + ", " +
                std::to_string(correspondences[i][1].x) + ", " +
                std::to_string(correspondences[i][1].y) + "\n";
    }
    return data;
}

void
ManualImageCorrespondence::RunManualCorrespondence(std::string d0, int p0, std::string d1, int p1) {
    ParseOptimizationResults por0(_map_base, d0);
    ParseOptimizationResults por1(_map_base, d1);
    std::string image0 = ParseSurvey::GetImagePath(_query_loc + d0, por0.cimage[p0]);
    std::string image1 = ParseSurvey::GetImagePath(_query_loc + d1, por1.cimage[p1]);
    cv::Mat im0 = ImageOperations::Load(image0);
    cv::Mat im1 = ImageOperations::Load(image1);
    
    std::cout << "showing pair " << std::endl;
    ShowImagePair(im0, im1);
    std::string data = CorrespondencesToString();
    
    std::string savef = _save_dir + d0 + "_" + std::to_string(p0) + "-" + d1 + "_" + std::to_string(p1) + ".txt";
    FileParsing::AppendToFile(savef, data, true);
}









