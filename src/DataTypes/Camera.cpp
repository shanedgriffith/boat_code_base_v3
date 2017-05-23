/*
 * Camera.cpp
 *
 *  Created on: Dec 15, 2016
 *      Author: shane
 */

#include "Camera.hpp"

bool Camera::InsideImage(int x, int y){
	return x >=0 && y >= 0 && x <_w && y < _h;
}

bool Camera::InsideImage(gtsam::Point2 p){
	return InsideImage(p.x(), p.y());
}

gtsam::Point2 Camera::ProjectToImage(gtsam::Point3 p){
	return gtsam::Point2((_fx*p.x() + _cx*p.z())/p.z(), (_fy*p.y() + _cy*p.z())/p.z());
}

cv::Mat Camera::CameraMatrix(){
	cv::Mat cam(3, 4, CV_64F, cv::Scalar(0));
	double * data = (double*)cam.data;
    data[0] = _fx;
    data[2] = _cx;
    data[5] = _fy;
    data[6] = _cy;
    data[10] = 1.0;
	return cam;
}

cv::Mat Camera::IntrinsicMatrix(){
    cv::Mat K(3, 3, CV_64F, cv::Scalar(0));
    double * data = (double*)K.data;
    data[0] = _fx;
    data[2] = _cx;
    data[4] = _fy;
    data[5] = _cy;
    data[8] = 1.0;
    return K;
}

cv::Point2f Camera::NormalizedToPixel(cv::Point2f p){
    cv::Point2f pix;
    pix.x = p.x*_fx + _cx;
    pix.y = p.y*_fy + _cy;
    return pix;
}

cv::Point2f Camera::PixelToNormalized(cv::Point2f p){
	//using undistort points accounts for the camera distortion.
	//if the image is already rectified, the distortion coefficients should be set to zero.
	std::vector<cv::Point2f> pvec = {p};
	cv::undistortPoints(pvec, pvec, IntrinsicMatrix(), Distortion());
	return pvec[0];
//    cv::Point2f norm;
//    norm.x = (p.x-_u)/_fx;
//    norm.y = (p.y-_v)/_fy;
//    return norm;
}

int Camera::w(){
	return _w;
}

int Camera::h(){
	return _h;
}

gtsam::Cal3_S2::shared_ptr Camera::GetGTSAMCam(){
//    if(!k)
//        k = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(_fx, _fy, 0.0, _u, _v));
    return k;
}

void Camera::SetDistortion(double k1, double k2, double p1, double p2, double k3){
    double * d = (double *) distCoeffs.data;
    d[0] = k1;
    d[1] = k2;
    d[2] = p1;
    d[3] = p2;
    d[4] = k3;
}

cv::Mat Camera::Distortion(){
    return distCoeffs;
}





