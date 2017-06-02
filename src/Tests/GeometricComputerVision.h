//
//  GeometricComputerVision.h
//  BeginningSLAM
//
//  Created by Shane Griffith on 1/15/14.
//  Copyright (c) 2014 Shane Griffith. All rights reserved.
//

#ifndef __BeginningSLAM__GeometricComputerVision__
#define __BeginningSLAM__GeometricComputerVision__

//#include <iostream>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>           //camera position
#include <gtsam/geometry/Point3.h>          //landmark coordinate
#include <gtsam/geometry/Point2.h>          //camera observation
#include <gtsam/geometry/SimpleCamera.h>    //calibration and performs projections

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

const int WORLD_TO_IMAGE = 0;
const int IMAGE_TO_WORLD = 1;

/*This file was motivated by the number of algorithms at
 http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO4/tutorial.html
 on the topic of geometric computer vision.
 */

cv::Mat_<double> LinearLSTriangulation(cv::Point3d u,       //homogenous image point (u,v,1)
                                   cv::Matx34d P,       //camera 1 matrix
                                   cv::Point3d u1,      //homogenous image point in 2nd camera
                                   cv::Matx34d P1       //camera 2 matrix
);


cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u,          //homogenous image point (u,v,1)
                                            cv::Matx34d P,          //camera 1 matrix
                                            cv::Point3d u1,         //homogenous image point in 2nd camera
                                            cv::Matx34d P1          //camera 2 matrix
);

gtsam::Point3 ProjectImageToWorld(gtsam::Point2 u, gtsam::Pose3 ucam, gtsam::Point2 v, gtsam::Pose3 vcam, 	);
gtsam::Point2 ProjectWorldToImage(gtsam::Pose3 cam, cv::Matx33d calib, gtsam::Point3 pw);
cv::Matx34d GetCameraMatrix(gtsam::Pose3 cam, cv::Matx33d calib);
cv::Matx33d GetRotationMatrix(cv::Point3d rotation);
cv::Point3d GetEulerAngles(cv::Matx33d rmatrix);
cv::Point3d GetTranslationFromTransform(cv::Matx34d transform);

















#endif /* defined(__BeginningSLAM__GeometricComputerVision__) */
