

#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include "SolveForMap.hpp"

/*
 //    SmartProjectionPoseFactor<Pose3, Point3, Cal3_S2> sppf(1, -1, false, false, boost::none, HESSIAN, 1e10,20);
 //    SmartProjectionPoseFactor(const double rankTol = 1,
 //                              const double linThreshold = -1, const bool manageDegeneracy = false,
 //                              const bool enableEPI = false, boost::optional<POSE> body_P_sensor = boost::none,
 //                              LinearizationMode linearizeTo = HESSIAN, double landmarkDistanceThreshold = 1e10,
 //                              double dynamicOutlierRejectionThreshold = -1) :
 //LinearizationMode linearizeTo = HESSIAN, double landmarkDistanceThreshold = 1e10, double dynamicOutlierRejectionThreshold = -1
 //landmarkDistanceThreshold - if the landmark is triangulated at a distance larger than that the factor is considered degenerate
 //dynamicOutlierRejectionThreshold - if this is nonnegative the factor will check if the average reprojection error is smaller than this threshold after triangulation,
 //  and the factor is disregarded if the error is large
 int ldist = (int) vals[Param::MAX_LANDMARK_DIST]; //this threshold specifies the distance between the camera and the landmark.
 int onoise = (int) vals[Param::MAX_ALLOWED_OUTLIER_NOISE]; //the threshold specifies at what point factors are discarded due to reprojection error.
 gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> sppf(1, -1, false, false, boost::none, gtsam::HESSIAN, ldist, onoise);
 
 add(const Point2 measured_i, const Key poseKey_i, const SharedNoiseModel noise_i, const boost::shared_ptr<CALIBRATION> K_i)
 
 to build cameras:
 typedef PinholeCamera<CALIBRATION> Camera;
 typedef std::vector<Camera> Cameras;

 use:
 typename Base::Cameras cameras(const Values& values);
 the keys in the values list corresponds to the anchors, not the poses. those have to be computed.
 
 to triangulate:
 triangulateSafe(const Cameras& cameras);
 
 to get the reprojection error:
 double totalReprojectionError(const Cameras& cameras);
 
 to get the result:
 boost::optional<Point3> point()
 
 */
using namespace std;

gtsam::Pose3 SolveForMap::VectorToPose(std::vector<double>& p){
    return gtsam::Pose3(gtsam::Rot3::ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

std::vector<double> SolveForMap::GetPoint(ParseOptimizationResults& POR, Anchors& anchors, LandmarkTrack& landmark){
    double dimensions = 2.0; //there are two dimensions to an image observation
    double dev = 3.0;
    gtsam::noiseModel::Isotropic::shared_ptr pixelNoise = gtsam::noiseModel::Isotropic::Sigma(dimensions, dev);
    
    int ldist = 100; //this threshold specifies the distance between the camera and the landmark.
    int onoise = 100; //the threshold specifies at what point factors are discarded due to reprojection error.
    gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> sppf(1, -1, false, false, boost::none, gtsam::HESSIAN, ldist, onoise);
    
    gtsam::Cal3_S2::shared_ptr calib = _cam.GetGTSAMCam();
    
    //can use double totalReprojectionError(const Cameras& cameras, const Point3& point)
    typedef gtsam::PinholeCamera<gtsam::Cal3_S2> Camera;
    typedef std::vector<Camera> Cameras;
    Cameras cameras;
    for(int i=0; i<landmark.Length(); i++){
        int pidx = landmark.camera_keys[i].index();
        vector<double> pose = POR.boat[pidx];
        int aidx = anchors.PoseIdxToAnchorIdx(pidx);
        vector<double> shifted = anchors.ShiftPose(aidx, pose);
        sppf.add(landmark.points[i], landmark.camera_keys[i], pixelNoise, calib);
        gtsam::Pose3 gtpose = VectorToPose(shifted);
//        gtsam::Pose3 gtpose = POR.CameraPose(pidx);
        cameras.push_back(Camera(gtpose, *calib));
    }
    
    //TODO: test.
    // and this version of the reprojection error may or may not be within the same limits.
    sppf.triangulateSafe(cameras);
    boost::optional<gtsam::Point3> p = sppf.point();
    gtsam::Point3 p3D(0,0,0);
    if(p) p3D = *p;
    double rerror_whitened = sppf.totalReprojectionError(cameras);
    return {p3D.x(), p3D.y(), p3D.z(), (double) landmark.key, rerror_whitened};
}

std::vector<double> SolveForMap::GetPoint(std::vector<std::vector<double> >& poses, LandmarkTrack& landmark){
    double dimensions = 2.0; //there are two dimensions to an image observation
    double dev = 3.0;
    gtsam::noiseModel::Isotropic::shared_ptr pixelNoise = gtsam::noiseModel::Isotropic::Sigma(dimensions, dev);
    
    int ldist = 100; //this threshold specifies the distance between the camera and the landmark.
    int onoise = 100; //the threshold specifies at what point factors are discarded due to reprojection error.
    gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> sppf(1, -1, false, false, boost::none, gtsam::HESSIAN, ldist, onoise);
    
    gtsam::Cal3_S2::shared_ptr calib = _cam.GetGTSAMCam();
    
    //can use double totalReprojectionError(const Cameras& cameras, const Point3& point)
    typedef gtsam::PinholeCamera<gtsam::Cal3_S2> Camera;
    typedef std::vector<Camera> Cameras;
    Cameras cameras;
    for(int i=0; i<landmark.Length(); i++){
        int pidx = landmark.camera_keys[i].index();
        gtsam::Pose3 gtpose = VectorToPose(poses[pidx]);
        cameras.push_back(Camera(gtpose, *calib));
        sppf.add(landmark.points[i], landmark.camera_keys[i], pixelNoise, calib);
    }
    
    //TODO: test.
    // and this version of the reprojection error may or may not be within the same limits.
    sppf.triangulateSafe(cameras);
    boost::optional<gtsam::Point3> p = sppf.point();
    gtsam::Point3 p3D(0,0,0);
    if(p) p3D = *p;
    double rerror_whitened = sppf.totalReprojectionError(cameras);
    return {p3D.x(), p3D.y(), p3D.z(), (double) landmark.key, rerror_whitened};
}




