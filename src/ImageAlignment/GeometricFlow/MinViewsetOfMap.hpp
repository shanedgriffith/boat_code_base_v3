/*
 * MinViewsetOfMap.hpp
 *
 *  Created on: Jun 21, 2018
 *      Author: shane
 */

#ifndef SRC_GEOMETRICFLOW_MINVIEWSETOFMAP_HPP_
#define SRC_GEOMETRICFLOW_MINVIEWSETOFMAP_HPP_


/*
 The min viewset according to a least viewed map point heuristic.
 */
class MinViewsetOfMap{
private:
    
    gtsam::Pose3 VectorToPose(std::vector<double>& p);
    void CountMapPointViews(std::vector<double>& pose, std::vector<int>& mappoint_vcounter);
    int FindLeastViewedMapPoint(std::vector<bool>& mappoint_vcounter);
    int CountUnviewedPointsForPose(gtsam::Pose3 tf, std::vector<bool>& mappoint_vcounter);
    int FindBestPoseToAdd(std::vector<int>& remaining_poses, int leastviewedmp, std::vector<bool>& mappoint_vcounter);
    
    
    std::vector<std::vector<double> >& _poses;
    Camera& _cam;
    Map& _map;
public:
    
    MinViewsetOfMap(Camera& cam, Map& map, std::vector<std::vector<double> >& poses):
    _cam(cam), _map(map), _poses(poses)
    {}
    
    //return a vector of indices to the minviewset
    std::vector<int> ComputeMinViewset();
    
};


#endif /* SRC_GEOMETRICFLOW_MINVIEWSETOFMAP_HPP_ */
