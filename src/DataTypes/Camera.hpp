/*
 * Camera.hpp
 *
 *  Created on: Dec 15, 2016
 *      Author: shane
 */

#ifndef SRC_DATATYPES_CAMERA_HPP_
#define SRC_DATATYPES_CAMERA_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>

class Camera{
public:
    cv::Mat distCoeffs;
	double _fx, _fy, _cx, _cy;
	int _w, _h;
    gtsam::Cal3_S2::shared_ptr k;
	Camera(double fx, double fy, double cx, double cy, int w, int h):
		_fx(fx), _fy(fy), _cx(cx), _cy(cy), _w(w), _h(h),
        k(new gtsam::Cal3_S2(_fx, _fy, 0.0, _cx, _cy)),
        distCoeffs(5, 1, CV_64F, cv::Scalar::all(0))
    {}
    
    ~Camera() {
//        delete(k); //check. this right for deleting the shared_ptr object?
    }
    
    void SetDistortion(double k1, double k2, double p1, double p2, double k3) const;
    cv::Mat Distortion() const;
	bool InsideImage(int x, int y) const;
	bool InsideImage(gtsam::Point2 p) const;
    cv::Point2f NormalizedToPixel(cv::Point2f p) const;
    cv::Point2f PixelToNormalized(cv::Point2f p) const;
	gtsam::Point2 ProjectToImage(gtsam::Point3) const;
    cv::Mat IntrinsicMatrix() const;
	int w() const;
	int h() const;
    gtsam::Cal3_S2::shared_ptr GetGTSAMCam() const;
};

#endif /* SRC_DATATYPES_CAMERA_HPP_ */
