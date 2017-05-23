/*
 * RFlowFME.hpp
 *
 *  Created on: Sep 30, 2016
 *      Author: shane
 */

#ifndef SRC_IMAGEALIGNMENT_DREAMFLOW_RFLOWFME_HPP_
#define SRC_IMAGEALIGNMENT_DREAMFLOW_RFLOWFME_HPP_

#include <stdio.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "FeatureMatchElimination.hpp"
#include "DataTypes/Camera.hpp"

/*
 Calculates epipolar constraints and returns them in the form of a winsize*winsize hypothesis space around each pixel. This is used in an MRF.
 */
class RFlowFME: public FeatureMatchElimination{

private:
    bool debug = false;

    int _h=-1;
    int _width, _height;
    double stdp=10.0;

    double EvaluateGaussian(int dline, double stdp);
    void GetHypothesisSpaceOutlier(std::vector<double>& hypspace, cv::Point2f center, cv::Point2f offset, cv::Vec3f line);
    double EvaluateGaussian(int fx, int fy, double ux, double uy, double stdp);
    void GetHypothesisSpaceInlier(std::vector<double>& hypspace, double cx=0, double cy=0);
    bool ComputeHypothesisSpaceFirst(std::vector<std::vector<double> >& hypspace, std::vector<cv::Point2f>& orig_sparse, std::vector<cv::Point2f>& mapped_sparse, cv::Mat& flow, std::vector<unsigned char>& labels);
    bool ComputeHypothesisSpace(std::vector<std::vector<double> >& hypspace, std::vector<cv::Point2f>& mapped, std::vector<unsigned char>& labels);
    void TransformFlow(cv::Mat& flow, std::vector<cv::Point2f>& orig, vector<cv::Point2f>& mapped);
    Camera& _cam;
public:
    RFlowFME(Camera& cam, int width, int height):
    _cam(cam), _width(width), _height(height), FeatureMatchElimination(){}

    bool IdentifyHypothesisSpace(std::vector<std::vector<double> >& hypspace, cv::Mat& flow, int h);
    bool IdentifyHypothesisSpace(std::vector<std::vector<double> >& hypspace, std::vector<cv::Point2f>& orig_sparse, std::vector<cv::Point2f>& mapped_sparse, cv::Mat& flow, int h);

    void setSTD(float std);
    void SetHypSpace(int hypspace);
};

#endif /* SRC_IMAGEALIGNMENT_DREAMFLOW_RFLOWFME_HPP_ */
