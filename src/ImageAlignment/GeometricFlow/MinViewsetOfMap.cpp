/*
 * MinViewsetOfMap.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: shane
 */

#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <ImageAlignment/GeometricFlow/MinViewsetOfMap.hpp>
#include <FileParsing/ParseOptimizationResults.h>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <DataTypes/LandmarkTrack.h>
#include <algorithm>
#include "Optimization/SingleSession/GTSAMInterface.h"

bool MinViewsetOfMap::DistanceCriterion(std::vector<double>& pose1, std::vector<double>& pose2){
    double dist = pow(pow(pose1[0] - pose2[0], 2) + pow(pose1[1] - pose2[1],2), 0.5);
    if (dist > pose_distance_threshold) {
        // Don't try to match points more than 20m away, GPS is not
        // that bad.
        return false;
    }
    if (fabs(remainder(pose1[5]-pose2[5],2*M_PI)) > pose_angle_threshold*M_PI/180) {
        // If we're not looking remotely in the same direction, no
        // point trying to match
        return false;
    }
    return true;
}

void MinViewsetOfMap::CountMapPointViews(std::vector<double>& pose, std::vector<int>& mappoint_vcounter, std::vector<int>& mptopose){
    gtsam::Pose3 tf = GTSAMInterface::VectorToPose(pose);
    
    for(int j=0; j<_map.map.size(); j++) {
        if(_map.map[j].x()==0.0 && _map.map[j].y()==0.0 && _map.map[j].z()==0.0) {
            mappoint_vcounter[j] = -1;
            continue;
        }
        if(!DistanceCriterion(_poses[mptopose[j]], pose)) continue;
        gtsam::Point3 res = tf.transform_to(_map.map[j]);
        gtsam::Point2 coord = _cam.ProjectToImage(res);
        if(_cam.InsideImage(coord))
            mappoint_vcounter[j]++;
    }
}

int MinViewsetOfMap::FindLeastViewedMapPoint(std::vector<int>& mappoint_vcounter){
    int minc = 1000000;
    int minidx = -1;
    for(int i=0; i<mappoint_vcounter.size(); i++){
        if(mappoint_vcounter[i] < 1) continue;
        if(mappoint_vcounter[i] < minc){
            minc = mappoint_vcounter[i];
            minidx = i;
        }
    }
    return minidx;
}

int MinViewsetOfMap::CountUnviewedPointsForPose(gtsam::Pose3 tf, int poseidx, std::vector<int>& mappoint_vcounter, std::vector<int>& mptopose){
    int nviewed = 0;
    for(int j=0; j<_map.map.size(); j++) {
        if(mappoint_vcounter[j] <= 0) continue; //already viewed
        if(!DistanceCriterion(_poses[mptopose[j]], _poses[poseidx])) continue;
        gtsam::Point3 res = tf.transform_to(_map.map[j]);
        gtsam::Point2 coord = _cam.ProjectToImage(res);
        if(_cam.InsideImage(coord))
            nviewed++;
    }
    return nviewed;
}

int MinViewsetOfMap::FindBestPoseToAdd(std::vector<int>& remaining_poses, int leastviewedmp, std::vector<int>& mappoint_vcounter, std::vector<int>& mptopose){
    int maxviewed=0;
    int maxidx = -1;
    for(int i=0; i<remaining_poses.size(); i++){
        gtsam::Pose3 tf = GTSAMInterface::VectorToPose(_poses[remaining_poses[i]]);
        gtsam::Point3 res = tf.transform_to(_map.map[leastviewedmp]);
        if(!DistanceCriterion(_poses[mptopose[leastviewedmp]], _poses[remaining_poses[i]])) continue;
        gtsam::Point2 coord = _cam.ProjectToImage(res);
        if(_cam.InsideImage(coord)) {
            int nviewed = CountUnviewedPointsForPose(tf, remaining_poses[i], mappoint_vcounter, mptopose);
            if(nviewed > maxviewed){
                maxviewed = nviewed;
                maxidx = i;
            }
        }
    }
    std::cout << "Number of points viewed by the pose: " << maxviewed << std::endl;
    return maxidx;
}

void MinViewsetOfMap::SetToViewed(std::vector<double>& pose, std::vector<int>& mappoint_vcounter, std::vector<int>& mptopose){
    gtsam::Pose3 tf = GTSAMInterface::VectorToPose(pose);
    
    for(int j=0; j<_map.map.size(); j++) {
        if(mappoint_vcounter[j] <= 0) continue; //already viewed
        if(!DistanceCriterion(_poses[mptopose[j]], pose)) continue;
        gtsam::Point3 res = tf.transform_to(_map.map[j]);
        gtsam::Point2 coord = _cam.ProjectToImage(res);
        if(_cam.InsideImage(coord))
            mappoint_vcounter[j] = -1;
    }
}

std::vector<LandmarkTrack> MinViewsetOfMap::BuildLandmarkTracks(){
    ParseOptimizationResults POR(_map_dir, _date);
    
    std::vector<LandmarkTrack> inactive;
    std::vector<LandmarkTrack> active;
    for(int i=0; i<POR.boat.size(); i++) {
        ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + _date, POR.ftfilenos[i]);
        std::vector<gtsam::Point3> p3d = POR.GetSubsetOf3DPoints(pftf.ids);
        pftf.ModifyFTFData(p3d);
        std::vector<LandmarkTrack> newinactive = pftf.ProcessNewPoints(0, i, active);
        for(int j=0; j<newinactive.size(); j++)
            inactive.push_back(newinactive[j]);
    }
    for(int j=0; j<active.size(); j++)
        inactive.push_back(active[j]);
    
    //sort the landmarks by the landmark key
    std::qsort(&inactive[0], inactive.size(), sizeof(LandmarkTrack), [](const void* a, const void* b) {
        const LandmarkTrack* arg1 = static_cast<const LandmarkTrack*>(a);
        const LandmarkTrack* arg2 = static_cast<const LandmarkTrack*>(b);
        
        if(arg1->key < arg2->key) return -1;
        else if(arg1->key > arg2->key) return 1;
        else {
            std::cout << "MinViewsetOfMap::BuildLandmarkTracks() Error. Two different landmarks with the same key!" << std::endl;
            exit(-1);
        }
        return 0;
    });
    
    return inactive;
}

int MinViewsetOfMap::BinarySearch(std::vector<LandmarkTrack>& tracks, int l){
    int s = 0;
    int e = tracks.size();
    while(e-s>1){
        int med = s+(e-s)/2;
        if(tracks[med].key>l) e = med;
        else if(tracks[med].key<l) s = med;
        else return med;
    }
    return s;
}

std::vector<int> MinViewsetOfMap::MapPointToPose(std::vector<int>& mappoint_vcounter, std::vector<LandmarkTrack>& tracks){
    std::vector<int> mptopose(_map.map.size(), -1);
    
    int count_valid=0;
    double avgdist = 0;
    
    double maxdist = 0;
    for(int i=0; i<_map.map.size(); i++) {
        int mid = _map.landmark_ids[i];
        
        int trackidx = BinarySearch(tracks, mid);
        if(tracks[trackidx].key!=mid){std::cout << "MapPointToTrack Error(). Map point not found " << mid << std::endl; exit(-1);}
        if(tracks[trackidx].camera_keys.size()==0){std::cout << "MapPointToTrack Error(). No cameras " << mid << std::endl; exit(-1);};
        if(tracks[trackidx].camera_keys.size() < 10)
            mappoint_vcounter[i] = -1;
        
        /* use the camera pose in the middle. */
        mptopose[i] = (tracks[trackidx].camera_keys[0] + tracks[trackidx].camera_keys.size())/2;
        /*
        double ad =0;
        for(int j=tracks[trackidx].camera_keys[0]; j<(tracks[trackidx].camera_keys[0] + tracks[trackidx].camera_keys.size()); j++){
            gtsam::Pose3 tf = GTSAMInterface::VectorToPose(_poses[j]);
            gtsam::Point3 res = tf.transform_to(_map.map[i]);
            double dist = tf.range(res);
            ad+=dist;
        }
        ad /= tracks[trackidx].camera_keys.size();
        if(ad > maxdist) maxdist = ad;
        if(ad < 50) count_valid++;
        avgdist += ad; */
        /* use the camera pose with the minimum distance.*/
        /*double mindist = 1000000;
        int minidx = -1;
        for(int j=tracks[trackidx].camera_keys[0]; j<(tracks[trackidx].camera_keys[0] + tracks[trackidx].camera_keys.size()); j++){
            gtsam::Pose3 tf = GTSAMInterface::VectorToPose(_poses[j]);
            gtsam::Point3 res = tf.transform_to(_map.map[i]);
            double dist = tf.range(res);
            if(dist < mindist){
                mindist = dist;
                minidx = j;
            }
        }
        mptopose[i] = minidx;*/
    }
    avgdist /= _map.map.size();
//    std::cout << "average point distance: " << avgdist << ", lt 50:" << count_valid << ", maxdist: " << maxdist << std::endl;
//    exit(1);
    return mptopose;
}

std::vector<int> MinViewsetOfMap::ComputeMinViewset() {
    std::cout << "Compute Min View Set Using Map Points:" << std::endl;
    std::vector<int> viewset;
    std::vector<int> mappoint_vcounter(_map.map.size(), 0);
    std::vector<int> remaining_poses(_poses.size(), 0);
    
    //acquire the set of landmark tracks
    std::vector<LandmarkTrack> tracks = BuildLandmarkTracks();
    if(tracks.size()==0) {std::cout << "MinViewsetOfMap::ComputeMinViewset() Error building landmark tracks." << std::endl; exit(-1);}
    
    //match each map point to a landmark track
    std::vector<int> mptopose = MapPointToPose(mappoint_vcounter, tracks);
    
    std::cout << "  Counting map point views. " << std::endl;
    //iterate through all the poses to create the list of map point views.
    for(int i=0; i<_poses.size(); i++) {
        remaining_poses[i] = i;
        CountMapPointViews(_poses[i], mappoint_vcounter, mptopose);
    }
    
    //while some map point is unviewed
    std::cout << "  Building the viewset." << std::endl;
    while(1){
        //find the min in the counter list. (stop and return the result if -1 is returned)
        int minviewed = FindLeastViewedMapPoint(mappoint_vcounter);
        if(minviewed==-1) break;
        std::cout << "least viewed map point: " << minviewed << " with " << mappoint_vcounter[minviewed] << std::endl;
        
        //iterate through all the poses to find the one that views the min-viewed map point, yet maximizes the number of unviewed points.
        int bestposeidx = FindBestPoseToAdd(remaining_poses, minviewed, mappoint_vcounter, mptopose);
        viewset.push_back(remaining_poses[bestposeidx]);
        
        SetToViewed(_poses[remaining_poses[bestposeidx]], mappoint_vcounter, mptopose);
        
        //add the pose to the set and mark the map points that it views as viewed.
        remaining_poses.erase(remaining_poses.begin()+bestposeidx, remaining_poses.begin()+bestposeidx + 1);
        
        std::cout << "added to viewset: " << viewset[viewset.size()-1] << std::endl;
    }
    
    std::sort(viewset.begin(), viewset.end());
    return viewset;
}


std::vector<int> MinViewsetOfMap::ComputeMinCoViewset(){
    std::vector<int> viewset;
    
    ReprojectionFlow rf(_cam, _map);
    viewset.push_back(0);
    
    int last = 0;
    for(int i=1; i<_poses.size(); i++){
//        std::cout << "iter " << i << std::endl;
        double gstatistic = rf.GStatisticForPose(_poses[i], _poses[last]);
        if(gstatistic < 1) {
            viewset.push_back(i);
            last = i;
//            std::cout << "added: " << i << std::endl;
        }
    }
    std::cout << "viewset size: " << viewset.size() << std::endl;
    
    
    return viewset;
}



void MinViewsetOfMap::PrintStatistics(std::vector<int>& viewset){
    std::cout << "Viewset has " << viewset.size() << " poses." << std::endl;
    std::cout << "Set: ";
    for(int i=0; i<viewset.size(); i++){
        std::cout << viewset[i] << ", ";
    }
    std::cout << std::endl;
    
    ReprojectionFlow rf(_cam, _map);
    
    std::cout << "Covisibility: " << std::endl;
    for(int i=1; i<viewset.size(); i++){
        double gstatistic = rf.GStatisticForPose(_poses[viewset[i]], _poses[viewset[i-1]]);
        std::cout << viewset[i] << "-" << viewset[i-1] << ": " << gstatistic << std::endl;
    }
}

