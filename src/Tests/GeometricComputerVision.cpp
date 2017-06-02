//
//  GeometricComputerVision.cpp
//  BeginningSLAM
//
//  Created by Shane Griffith on 1/15/14.
//  Copyright (c) 2014 Shane Griffith. All rights reserved.
//

#include "GeometricComputerVision.h"


using namespace cv;
using namespace std;
using namespace gtsam;

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 
 From: http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/
 */
Mat_<double> LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
                                   Matx34d P,       //camera 1 matrix
                                   Point3d u1,      //homogenous image point in 2nd camera
                                   Matx34d P1       //camera 2 matrix
)
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
    
    Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)-P(0,3)),
                      -(u.y*P(2,3)-P(1,3)),
                      -(u1.x*P1(2,3)-P1(0,3)),
                      -(u1.y*P1(2,3)-P1(1,3)));
    
    Mat_<double> X;
    solve(A,B,X,DECOMP_SVD);
    return X;
}


/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 
 From: http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/
 
 After features are matched in an image, and given the camera matrix for each point, this function calculates the 3-D position
 of X that was projected onto u and u1 in the two different images.
 */
Mat_<double> IterativeLinearLSTriangulation(Point3d u,    //homogenous image point (u,v,1)
                                            Matx34d P,          //camera 1 matrix
                                            Point3d u1,         //homogenous image point in 2nd camera
                                            Matx34d P1)          //camera 2 matrix
{
    double EPSILON = 0.001;
    
    double wi = 1, wi1 = 1;
    Mat_<double> X(4,1);
    for (int i=0; i<10; i++)
    { //Hartley suggests 10 iterations at most
        Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
        
        //recalculate weights
        double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
        double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
        
        //breaking point
        if(fabs(wi - p2x) <= EPSILON && fabs(wi1 - p2x1) <= EPSILON) break;
        
        wi = p2x;
        wi1 = p2x1;
        
        //reweight equations and solve
        Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
        
        solve(A,B,X_,DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    }
    return X;
}


Matx33d ConvertMatrix(Matrix m)
{
    return Matx33d(m(0,0), m(0,1), m(0,2), m(1,0), m(1,1), m(1,2), m(2,0), m(2,1), m(2,2));
}


Matx34d ConvertPose(Pose3 cam)
{
    Matrix4 T = cam.matrix();
    Matx33d rot(T(0,0), T(0,1), T(0,2),
                T(1,0), T(1,1), T(1,2),
                T(2,0), T(2,1), T(2,2));
    Matx31d t(T(0,3), T(1,3), T(2,3));
    
    Matx33d rot_T;
    transpose(rot, rot_T);
    Matx31d t_new = -1*rot_T * t;
    return Matx34d(rot_T(0,0), rot_T(0,1), rot_T(0,2), t_new(0,0),
                    rot_T(1,0), rot_T(1,1), rot_T(1,2), t_new(1,0),
                    rot_T(2,0), rot_T(2,1), rot_T(2,2), t_new(2,0));
}


Matx34d GetCameraMatrix(Pose3 cam, Matx33d calib)
{
    Matrix4 T = cam.matrix();
    Matx33d rot(T(0,0), T(0,1), T(0,2),
                T(1,0), T(1,1), T(1,2),
                T(2,0), T(2,1), T(2,2));
    Matx31d t(T(0,3), T(1,3), T(2,3));
    
    Matx33d rot_T;
    transpose(rot, rot_T);
    Matx31d t_new = -1*rot_T * t;
    Matx34d inverse(rot_T(0,0), rot_T(0,1), rot_T(0,2), t_new(0,0),
                    rot_T(1,0), rot_T(1,1), rot_T(1,2), t_new(1,0),
                    rot_T(2,0), rot_T(2,1), rot_T(2,2), t_new(2,0));
    
    return calib * inverse;
}


Point3 ProjectImageToWorld(Point2 u, Pose3 ucam, Point2 v, Pose3 vcam, Matrix calib)
{
    /*A GTSam/SLAM interface for projecting points to 3D*/
    Matx33d cv_calib = ConvertMatrix(calib);
//    Matx34d cv_ucam = ConvertPose(ucam);
//    Matx34d cv_vcam = ConvertPose(vcam);
//    Matx34d umat = cv_calib * cv_ucam;
    //    Matx34d vmat = cv_calib * cv_vcam;
    Matx34d umat = GetCameraMatrix(ucam, cv_calib);
    Matx34d vmat = GetCameraMatrix(vcam, cv_calib);
    Point3d uhom(u.x(),u.y(),1.0);
    Point3d vhom(v.x(),v.y(),1.0);

    Mat_<double> res = IterativeLinearLSTriangulation(uhom, umat, vhom, vmat);
    return Point3(res(0,0), res(1,0), res(2,0));
}


Point2 ProjectWorldToImage(Pose3 cam, Matx33d calib, Point3 pw)
{
    Matx41d pworld(pw.x(), pw.y(), pw.z(), 1);
    Matx31d res = GetCameraMatrix(cam, calib) * pworld;
    if(res(2,0)==0) {cout << "Error. The point could not be triangulated. It may be at the camera center.\n"; exit(-1);}
    if(res(2,0) < 0) {cout << "Error. Chierality constraint. The projected depth is negative: ("<<res(0,0)<<", "<<res(1,0)<<", "<<res(2,0)<<"\n"; exit(-1);}
    return Point2(res(0,0)/res(2,0), res(1,0)/res(2,0));
}


Matx33d GetRotationMatrix(Point3d rotation)
{
    /*Converts the three orientation angles into a rotation matrix.
     see http://www.songho.ca/opengl/gl_anglestoaxes.html
     */
    double X = rotation.x;
    double Y = rotation.y;
    double Z = rotation.z;
    
    Matx33d R(cos(Y)*cos(Z), -cos(Y)*sin(Z), sin(Y),
              sin(X)*sin(Y)*cos(Z) + cos(X)*sin(Z), -sin(X)*sin(Y)*sin(Z)+cos(X)*cos(Z), -sin(X)*cos(Y),
              -cos(X)*sin(Y)*cos(Z)+sin(X)*sin(Z), cos(X)*sin(Y)*sin(Z)+sin(X)*cos(Z), cos(X)*cos(Y)
              );
    
    return R;
}


Point3d GetEulerAngles(Matx33d rmatrix)
{
    /*Extracts the euler angles from the rotation matrix.
     see https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles.pdf
     */
    double theta1 = atan2(rmatrix(1,2), rmatrix(2,2));
    double c2 = pow(pow(rmatrix(0,1),2) + pow(rmatrix(0,1),2),0.5);
    double theta2 = atan2(-rmatrix(0,2), c2);
    double s1 = sin(theta1);
    double c1 = cos(theta1);
    double theta3 = atan2(s1*rmatrix(2,0)-c1*rmatrix(1,0), c1*rmatrix(1,1)-s1*rmatrix(2,1));
    return Point3d(theta1, theta2, theta3);
}


Point3d GetTranslationFromTransform(Matx34d transform)
{
    return Point3d(transform(0,3), transform(1,3), transform(2,3));
}
