/*
 * MinViewsetOfMap.hpp
 *
 *  Created on: Jun 21, 2018
 *      Author: shane
 */

#ifndef SRC_GEOMETRICFLOW_MINVIEWSETOFMAP_HPP_
#define SRC_GEOMETRICFLOW_MINVIEWSETOFMAP_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>

#include <DataTypes/Map.hpp>
#include <DataTypes/Camera.hpp>

/*
 The min viewset according to a least viewed map point heuristic.
 TODO: fix.
 */
class MinViewsetOfMap{
private:
    double pose_distance_threshold = 5.0; //meters
    double pose_angle_threshold = 20.0;//degrees.
    double view_distance_threshold = 20;//50.0; // meters
    
    bool DistanceCriterion(std::vector<double>& pose1, std::vector<double>& pose2);
    void CountMapPointViews(std::vector<double>& pose, std::vector<int>& mappoint_vcounter, std::vector<int>& mptopose);
    int FindLeastViewedMapPoint(std::vector<int>& mappoint_vcounter);
    int CountUnviewedPointsForPose(gtsam::Pose3 tf, int poseidx, std::vector<int>& mappoint_vcounter, std::vector<int>& mptopose);
    int FindBestPoseToAdd(std::vector<int>& remaining_poses, int leastviewedmp, std::vector<int>& mappoint_vcounter, std::vector<int>& mptopose);
    void SetToViewed(std::vector<double>& pose, std::vector<int>& mappoint_vcounter, std::vector<int>& mptopose);
    
    std::vector<LandmarkTrack> BuildLandmarkTracks();
    int BinarySearch(std::vector<LandmarkTrack>& tracks, int l);
    std::vector<int> MapPointToPose(std::vector<int>& mappoint_vcounter, std::vector<LandmarkTrack>& tracks);
    
    
    std::vector<std::vector<double> >& _poses;
    Camera& _cam;
    Map& _map;
public:
    std::string _map_dir;
    std::string _pftbase;
    std::string _date;
    
    MinViewsetOfMap(Camera& cam, Map& map, std::vector<std::vector<double> >& poses, std::string date, std::string map_dir, std::string pftbase):
    _cam(cam), _map(map), _poses(poses),
    _date(date), _map_dir(map_dir), _pftbase(pftbase)
    {}
    
    //return a vector of indices to the minviewset
    std::vector<int> ComputeMinViewset();
    
    std::vector<int> ComputeMinCoViewset();
    
    void PrintStatistics(std::vector<int>& viewset);
    
};


#endif /* SRC_GEOMETRICFLOW_MINVIEWSETOFMAP_HPP_ */
