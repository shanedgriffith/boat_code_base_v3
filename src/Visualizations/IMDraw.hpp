/*
 * DrawPoints.hpp
 *
 *  Created on: May 2, 2017
 *      Author: shane
 */

#ifndef SRC_VISUALIZATIONS_IMDRAW_HPP_
#define SRC_VISUALIZATIONS_IMDRAW_HPP_

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class IMDraw{
private:
    int psize;
    CvScalar default_color;
public:
    cv::Mat& _image;
    IMDraw(cv::Mat& image):
        _image(image), default_color(CV_RGB(255,0,0)), psize(4) {}
    CvScalar GetLandmarkColor(int id=-1);
    void DrawPoint(double x, double y, int id=-1);
    void DrawArrow(double x, double y, double endx, double endy, int id=-1);
    void SetPointSize(int s) {psize = s;}
    void SetColor(CvScalar col) {default_color = col;}
};



#endif /* SRC_VISUALIZATIONS_IMDRAW_HPP_ */
