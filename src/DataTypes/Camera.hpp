/*
 * Camera.hpp
 *
 *  Created on: Dec 15, 2016
 *      Author: shane
 */

#pragma once

#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3DS2.h>

class Camera{
public:
    double _fx, _fy, _cx, _cy;
    int _w, _h;
    
    boost::shared_ptr<gtsam::Cal3DS2> k;
    
    Camera(double fx, double fy, double cx, double cy, int w, int h);
    
    ~Camera() {}
    
    void SetDistortion(double k1, double k2, double p1, double p2, double k3);
    bool InsideImage(int x, int y) const;
    bool InsideImage(gtsam::Point2 p) const;
    gtsam::Point2 NormalizedToPixel(gtsam::Point2 p) const;
    gtsam::Point2 PixelToNormalized(gtsam::Point2 p) const;
    gtsam::Point2 ProjectToImage(gtsam::Point3 p) const;
    cv::Mat IntrinsicMatrix() const;
    int w() const;
    int h() const;
    boost::shared_ptr<gtsam::Cal3DS2> GetGTSAMCam() const;
};

