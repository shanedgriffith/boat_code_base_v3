/*
 * DrawPoints.cpp
 *
 *  Created on: May 2, 2017
 *      Author: shane
 */

#include "IMDraw.hpp"

using namespace cv;

CvScalar IMDraw::GetLandmarkColor(int id){
    if(id==-1) return default_color;
    int red = (71*(id%10) + id%255)%255;
    int green = (111*(id%10) + (2*id)%255)%255;
    int blue = (27*(id%10) + (3*id)%255)%255;
    return CV_RGB(red, green, blue);
}

void IMDraw::DrawPoint(double x, double y, int id){
    CvScalar col = GetLandmarkColor(id);
    circle(_image, cv::Point(x, y), psize, col, -1);
}

void IMDraw::DrawArrow(double x, double y, double endx, double endy, int id){
    CvScalar col = GetLandmarkColor(id);
    arrowedLine(_image, cv::Point2f(x, y), cv::Point2f(endx, endy), col);
}
