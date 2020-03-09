#pragma once


#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>


class EpipolarGeometry
{
private:
    
    gtsam::EssentialMatrix
    computeEssentialMatrix(const gtsam::Pose3& pose_time_0, const gtsam::Pose3& pose_time_1);
    
    double
    distanceToEpipolarLine(const gtsam::Point2& cp0, const gtsam::Point2& cp1, const gtsam::EssentialMatrix& E);
    
    gtsam::Matrix3
    computeFFromEAndK(const gtsam::EssentialMatrix& E, boost::shared_ptr<gtsam::Cal3DS2> cam);
    
    gtsam::Vector3
    computeEpiline(const gtsam::Matrix3& F, const gtsam::Point2& p);
    
    gtsam::Vector3
    uncalibrateEpiline(const gtsam::Vector3& vec);
    
    gtsam::EssentialMatrix E;
    
    gtsam::Matrix3 F;
    
public:
    
    EpipolarGeometry(const gtsam::Pose3& pose_time_0, const gtsam::Pose3& pose_time_1, boost::shared_ptr<gtsam::Cal3DS2> cam);
    
    double
    fundamentalMatrixError(const gtsam::Point2& p0, const gtsam::Point2& p1, boost::shared_ptr<gtsam::Cal3DS2> cam);
    
    
    
};
