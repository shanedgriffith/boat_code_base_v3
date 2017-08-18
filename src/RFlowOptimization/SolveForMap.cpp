

#include <gtsam/geometry/triangulation.h>


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

void MultiAnchorsOptimization::SolveForCameras() {
    /*
     typedef CameraSet<CAMERA> Cameras;
     typename Base::Cameras cameras;
     Pose3 pose; K_ is the calibration.
     Camera camera(pose, K_);
     cameras.push_back(camera);
     TriangulationResult triangulateSafe(const std::vector<CAMERA>& cameras,
     const std::vector<Point2>& measured,
     const TriangulationParameters& params)
     //what about the noise model?
     */
    TriangulationParameters tp(1, false, 100, 100);
    
    
}

/*
 PinholeCamera<gtsam::Cal3_S2> Camera;
 std::vector<Camera> Cameras;
 index()
 
 */


gtsam::Point3 SolveForMap::GetPoint(ParseOptimizationResult& POR, std::vector<Anchors>& anchors, LandmarkTrack& lt){
    //BOOST_PAIR? {point3 and double}
    //symbol
    
    //can use double totalReprojectionError(const Cameras& cameras, const Point3& point)
    
    
    
}


/**
 * Collect all cameras involved in this factor
 * @param values Values structure which must contain camera poses corresponding
 * to keys involved in this factor
 * @return vector of Values
 */
typename Base::Cameras cameras(const Values& values) const {
    typename Base::Cameras cameras;
    size_t i=0;
    BOOST_FOREACH(const Key& k, this->keys_) {
        Pose3 pose = values.at<Pose3>(k);
        typename Base::Camera camera(pose, *K_all_[i++]);
        cameras.push_back(camera);
    }
    return cameras;
}





