/*
 * MinViewsetOfMap.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: shane
 */


std::vector<int> MinViewsetOfMap::ComputeMinViewset(){
    std::vector<int>& remaining_poses(_poses.size(), 0);
    for(int i=0; i<_poses.size(); i++)
        remaining_poses[i] = i;
    
    std::vector<int> mappoint_vcounter(_map.map.size(), 0);
    
    std::vector<int> viewset;
    
    //while some map point is unviewed
    while(1){
        //iterate through all the poses and update the list of map point counters.
        for(int i=0; i<remaining_poses.size(); i++)
            CountMapPointViews(_poses[remaining_poses[i]], mappoint_vcounter);
        
        //find the min in the counter list. (stop and return the result if -1 is returned)
        int minviewed = FindLeastViewedMapPoint(mappoint_vcounter);
        if(minviewed==-1) break;
        
        //iterate through all the poses to find the one that views the min-viewed map point, yet maximizes the number of unviewed points.
        int bestposeidx = FindBestPoseToAdd(remaining_poses, minviewed);
        viewset.push_back(remaining_poses[bestposeidx]);
        
        //add the pose to the set and mark the map points that it views as viewed.
        remaining_poses.erase(remaining_poses.begin()+bestposeidx, remaining_poses.begin()+bestposeidx + 1);
    }
    
    std::sort(viewset);
    return viewset;
}

gtsam::Pose3 MinViewsetOfMap::VectorToPose(std::vector<double>& p){
    return gtsam::Pose3(gtsam::Rot3::Ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

void MinViewsetOfMap::CountMapPointViews(std::vector<double>& pose, std::vector<int>& mappoint_vcounter){
    gtsam::Pose3 tf = VectorToPose(pose);
    
    for(int j=0; j<_map.map.size(); j++) {
        if(mappoint_vcounter[j] == -1) continue; //already viewed
        if(_map.map[j].x()==0.0 && _map.map[j].y()==0.0 && _map.map[j].z()==0.0) {
            mappoint_vcounter[j] = -1;
            continue;
        }
        gtsam::Point3 res = tf.transform_to(_map.map[j]);
        gtsam::Point2 coord = _cam.ProjectToImage(res);
        if(_cam.InsideImage(coord))
            mappoint_vcounter[j]++;
    }
}

int MinViewsetOfMap::FindLeastViewedMapPoint(std::vector<bool>& mappoint_vcounter){
    int minc = 1000000;
    int minidx = -1;
    for(int i=0; i<mappoint_vcounter.size(); i++){
        if(mappoint_vcounter[j] < 1) continue; //already viewed
        if(mappoint_vcounter[j] < minc){
            minc = mappoint_vcounter[j];
            minidx = i;
        }
    }
    return minidx;
}

int MinViewsetOfMap::CountUnviewedPointsForPose(gtsam::Pose3 tf, std::vector<bool>& mappoint_vcounter){
    int nviewed = 0;
    for(int j=0; j<_map.map.size(); j++) {
        if(mappoint_vcounter[j] == -1) continue; //already viewed
        gtsam::Point3 res = tf.transform_to(_map.map[j]);
        gtsam::Point2 coord = _cam.ProjectToImage(res);
        if(_cam.InsideImage(coord))
            nviewed++;
    }
    return nviewed;
}

int MinViewsetOfMap::FindBestPoseToAdd(std::vector<int>& remaining_poses, int leastviewedmp, std::vector<bool>& mappoint_vcounter){
    int maxviewed=0;
    int maxidx = -1;
    for(int i=0; i<remaining_poses.size(); i++){
        gtsam::Pose3 tf = VectorToPose(_poses[remaining_poses[i]]);
        gtsam::Point3 res = tf.transform_to(_map.map[leastviewedmp]);
        gtsam::Point2 coord = _cam.ProjectToImage(res);
        if(_cam.InsideImage(coord)) {
            nviewed = CountUnviewedPointsForPose(tf, mappoint_vcounter);
            if(nviewed > maxviewed){
                nviewed = maxviewed;
                maxidx = i;
            }
        }
    }
    return maxidx;
}







