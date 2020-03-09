#include "EpipolarGeometry.h"


EpipolarGeometry(const gtsam::Pose3& pose_time_0, const gtsam::Pose3& pose_time_1, boost::shared_ptr<gtsam::Cal3DS2> cam)
: E(computeEssentialMatrix(pose_time_0, pose_time_1))
{
    
}

double
EpipolarGeometry::
distanceToEpipolarLine(const gtsam::Point2& cp0, const gtsam::Point2& cp1, const gtsam::EssentialMatrix& E)
{
    gtsam::Vector3 p0h(cp0.x(), cp0.y(), 1);
    gtsam::Vector3 p1h(cp1.x(), cp1.y(), 1);
    return p1h.transpose() * E.matrix() * p0h;
}

gtsam::EssentialMatrix
EpipolarGeometry::
computeEssentialMatrix(const gtsam::Pose3& pose_time_0, const gtsam::Pose3& pose_time_1)
{
    gtsam::Pose3 btwn = pose_time_0.between(pose_time_1);
    gtsam::EssentialMatrix E = gtsam::EssentialMatrix::FromPose3(btwn);
    return E;
}

gtsam::Matrix3
EpipolarGeometry::
computeFFromEAndK(const gtsam::EssentialMatrix& E, boost::shared_ptr<gtsam::Cal3DS2> cam)
{
    gtsam::Matrix3 m = E.matrix();
    //convert E to F.
    gtsam::Matrix3 K = cam->K();
    gtsam::Matrix3 Kinv = K.inverse();
    return Kinv.transpose() * m * Kinv;
}

double
EpipolarGeometry::
fundamentalMatrixError(const gtsam::Point2& p0, const gtsam::Point2& p1, boost::shared_ptr<gtsam::Cal3DS2> cam)
{
    gtsam::EssentialMatrix E = computeEssentialMatrix(pose0, pose1);
    
    gtsam::Matrix3 F = computeFFromEAndK(E, cam);
    
    gtsam::Vector3 epiline = computeEpiline(F, p0);
    //gtsam::Vector3 p0h = gtsam::EssentialMatrix::Homogeneous(p0);
    gtsam::Vector3 p1h(p1.x(), p1.y(), 1);
    
    return fabs(epiline.transpose() * p1h);
}




