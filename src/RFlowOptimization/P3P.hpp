#pragma once


/*
 Removed the TooN dependency.
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include "DataTypes/Camera.hpp"

class P3P
{
private:
    static void solveQuartic( Eigen::Matrix<double,5,1> factors, Eigen::Vector4d & realRoots );

public:
    P3P() {}
    virtual ~P3P() {}
    
    /*
     feature vectors are the uncalibrated 2D points.
     worldPoints are the 3D points.
     solutions are the 4 possible result poses.
     */
    static int computePoses( const std::vector<gtsam::Vector3>& featureVectors, const std::vector<gtsam::Point3>& worldPoints, std::vector<gtsam::Pose3>& solutions );
};

