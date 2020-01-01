#pragma once


/*
 Removed the TooN dependency.
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>

#include "DataTypes/Camera.hpp"

class P3P
{
private:
    
    static void
    solveQuartic(Eigen::Matrix<double,5,1> factors, Eigen::Vector4d & realRoots);
    
    /*
     feature vectors are the uncalibrated 2D points. (normalized image coordinated as Unit3; see backprojectToInfinity())
     worldPoints are the 3D points.
     solutions are the 4 possible result poses.
     */
    std::vector<gtsam::Pose3>
    computePoses(const std::vector<gtsam::Vector3>& featureVectors);
    
    gtsam::Pose3
    disambiguatePoses(const std::vector<gtsam::Pose3>& poses, gtsam::Point3& point3d, gtsam::Point2& point2d);
    
    std::vector<gtsam::Vector3>
    pixelsToVectors();
    
    // Verify that world points are not colinear (parallel lines; fewer than three unique points)
    bool
    suitableSet();
    
    const Camera& cam_;
    const std::vector<gtsam::Point3>& world_points_;
    const std::vector<gtsam::Point2>& p2d_subset_;
    
public:
    
    P3P(const Camera& cam, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset);
    
    std::tuple<bool, gtsam::Pose3>
    run();
    
};

