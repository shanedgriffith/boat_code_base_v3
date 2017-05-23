//
//  FeatureMatchElimination.hpp
//  SIFTFlow
//
//  Created by Shane Griffith on 2/28/16.
//  Copyright © 2016 shane. All rights reserved.
//

#ifndef FeatureMatchElimination_hpp
#define FeatureMatchElimination_hpp

#include <stdio.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <DataTypes/Camera.hpp>

/*
 Code from:
 https://github.com/BloodAxe/OpenCV-Tutorial/blob/master/OpenCV%20Tutorial/FeatureDetectionClass.h

 TODO: First convert the points to camera coordinates for better results.
    See http://stackoverflow.com/questions/25251676/opencv-findfundamentalmat-very-unstable-and-sensitive
 */
class FeatureMatchElimination{
protected:
    std::vector<cv::Point2f> p1;
    std::vector<cv::Point2f> p2;
    
    cv::Mat F;
    cv::Mat E;
    
    int CountInliers(std::vector<unsigned char>& labels);

public:
    bool debug = false;
    int IdentifyInliersAndOutliers(Camera& _cam, std::vector<cv::Point2f>& points1,
                                                  std::vector<cv::Point2f>& points2,
                                                std::vector<unsigned char>& inliers);
    bool AreInliersMeaningful(int v);
    void CorrectMatches(std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);
    vector<cv::Vec3f> GetDenseEpipolarLines(std::vector<cv::Point2f>& orig, cv::Mat& F);
    cv::Mat GetFundamentalMat();
    cv::Mat GetEssentialMat();
    
    FeatureMatchElimination(){}
};
#endif /* FeatureMatchElimination_hpp */
