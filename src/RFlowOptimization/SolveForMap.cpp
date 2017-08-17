

#include <gtsam/geometry/triangulation.h>

#include "SolveForMap.hpp"



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



