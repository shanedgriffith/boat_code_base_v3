/*
 * DrawPoints.hpp
 *
 *  Created on: May 2, 2017
 *      Author: shane
 */
#pragma once

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class IMDraw{
private:
    int psize;
public:
    cv::Mat& _image;
    IMDraw(cv::Mat& image):
        _image(image), psize(4) {}
    static CvScalar GetLandmarkColor(int id=-1);
    void DrawPoint(double x, double y, int id=-1);
    void DrawArrow(double x, double y, double endx, double endy, int id=-1);
    void SetPointSize(int s) {psize = s;}
//    void SetColor(CvScalar col) {default_color = col;}
};
