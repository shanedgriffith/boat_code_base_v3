/*
 * Camera.cpp
 *
 *  Created on: Dec 15, 2016
 *      Author: shane
 */

#include "Camera.hpp"

Camera::
Camera(double fx, double fy, double cx, double cy, int w, int h)
: _fx(fx)
, _fy(fy)
, _cx(cx)
, _cy(cy)
, _w(w)
, _h(h)
, k(boost::make_shared<gtsam::Cal3DS2>(_fx, _fy, 0.0, _cx, _cy, 0, 0, 0, 0))
{}

bool
Camera::
InsideImage(int x, int y) const
{
    return x >=0 && y >= 0 && x <_w && y < _h;
}

bool
Camera::
InsideImage(gtsam::Point2 p) const
{
    return InsideImage(p.x(), p.y());
}

gtsam::Point2
Camera::
ProjectToImage(gtsam::Point3 p) const
{
    if(p.z()<0) return gtsam::Point2(-1, -1);
    return gtsam::Point2((_fx*p.x() + _cx*p.z())/p.z(), (_fy*p.y() + _cy*p.z())/p.z());
}

cv::Mat
Camera::
IntrinsicMatrix() const
{
    cv::Mat K(3, 3, CV_64F, cv::Scalar(0));
    double * data = (double*)K.data;
    data[0] = _fx;
    data[2] = _cx;
    data[4] = _fy;
    data[5] = _cy;
    data[8] = 1.0;
    return K;
}

gtsam::Point2
Camera::
NormalizedToPixel(gtsam::Point2 p) const
{
    return k->uncalibrate(p);
}

gtsam::Point2
Camera::
PixelToNormalized(gtsam::Point2 p) const
{
    return k->calibrate(p);
}

int
Camera::
w() const{
    return _w;
}

int
Camera::
h() const{
    return _h;
}

boost::shared_ptr<gtsam::Cal3DS2>
Camera::
GetGTSAMCam() const
{
    return k;
}

void
Camera::
SetDistortion(double k1, double k2, double p1, double p2, double k3)
{
    k = boost::make_shared<gtsam::Cal3DS2>(_fx, _fy, 0.0, _cx, _cy, k1, k2, p1, p2);
}





