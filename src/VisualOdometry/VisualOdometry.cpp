//
//  VisualOdometry.cpp
//  boat_code_base
//
//  Created by Shane Griffith on 3/18/17.
//  Copyright © 2017 shane. All rights reserved.
//

#include "VisualOdometry.hpp"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include "FileParsing/ParseSurvey.h"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <RFlowOptimization/LocalizePose.hpp>

using namespace std;

void VisualOdometry::RemoveLandmarkFromList(int list_idx) {
    active.erase(active.begin() + list_idx, active.begin() + list_idx + 1);
}

void VisualOdometry::AddLandmarkTracks(vector<LandmarkTrack>& landmarks, int survey){
    for(int i=0; i<landmarks.size(); i++){
        AddLandmarkTrack(_cam.GetGTSAMCam(), landmarks[i].key, landmarks[i].points, landmarks[i].camera_keys, landmarks[i].used, survey);
    }
}

vector<LandmarkTrack> VisualOdometry::ProcessNewPoints(int ckey, ParseFeatureTrackFile& pft) {
    vector<LandmarkTrack> inactive;
    
    int next_entry = 0;
    int lasti=0;
    for(int i=0; i<pft.ids.size(); i++) {
        //remove features that aren't tracked anymore
        //add to the entry using the info from the new frame.
        while(active.size() > next_entry && active[next_entry].GetKey() < pft.ids[i]) {
            if(debug)cout << "Removed landmark " << active[next_entry].key << endl;
            if(active[next_entry].Length()>1) inactive.push_back(active[next_entry]);
            RemoveLandmarkFromList(next_entry);
        }
        
        //create a new entry if its key is greater than anything that's active
        if(active.size() == next_entry) {
            lasti=i;
            break;
        } else if(pft.ids[i] < active[next_entry].key) {
            cout << "ActiveFactors Error: Something is wrong with " << pft.siftfile << ". Landmark " << pft.ids[i] << " wasn't tracked in at least one frame before this." << endl;
            cout << " Alternatively, the rest of the active factors may not have been added before adding the next survey."<<endl;
            exit(-1);
        } else if(pft.ids[i] == active[next_entry].key) {
            //accumulate info about the landmark (should be the only remaining case)
            active[next_entry].AddToTrack(pft.time, pft.imagecoord[i], ckey);
            if(debug) cout << "landmark measurement for " << active[next_entry].key << endl;
            //inc next_entry.
            next_entry++;
        }
    }
    
    //add the rest
    srand(time(NULL));
    for(int i=lasti; i<pft.ids.size(); i++) {
        //used to limit the size of the optimization problem for inter-survey optimization
        bool used = true;//(rand()%100 < percent_of_tracks);
        LandmarkTrack lt(pft.ids[i], used);
        lt.AddToTrack(pft.time, pft.imagecoord[i], ckey);
        active.push_back(lt);
    }
    return inactive;
}

void VisualOdometry::ConstructGraph(gtsam::Pose3 cam, ParseFeatureTrackFile& PFT) {
    /*Construct the graph (camera, visual landmarks, velocity, and time)
     */
    //add the poses
    poses.push_back(cam);
    for(int i=0; i<poses.size(); i++) {
        gtsam::Symbol symb('x', i);
        AddPose(symb, poses[i]);
        if(i > 0) {
            gtsam::Symbol symbprev('x', i-1);
            AddOdom(symbprev, poses[i-1], symb, poses[i]);
        }
    }
    
    //process the landmark measurement
    vector<LandmarkTrack> inactive = ProcessNewPoints(posenum, PFT);
    landmark_sets.push_back(inactive);
    for(int i=0; i<landmark_sets.size(); i++) {
        AddLandmarkTracks(landmark_sets[i]);
    }
    AddLandmarkTracks(active);
}

void VisualOdometry::AddPose(gtsam::Symbol symb, gtsam::Pose3 pguess){
    std::vector<double> noise = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5};
    gtsam::Vector6 v6 = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&noise[0]), 6, 1);
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas(v6);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(symb, pguess, poseNoise));
    initEst.insert(symb, pguess);
}

void VisualOdometry::AddOdom(gtsam::Symbol symb0, gtsam::Pose3 pguess0, gtsam::Symbol symb1, gtsam::Pose3 pguess1){
    std::vector<double> noise = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5};
    gtsam::Vector6 v6 = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&noise[0]), 6, 1);
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas(v6);
    gtsam::Pose3 odom = pguess0.between(pguess1);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symb0, symb1, odom, poseNoise));
}

void VisualOdometry::AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, int landmark_key, vector<gtsam::Point2>& points, vector<int> camera_keys, bool used, int survey){
    /*Add the landmark track to the graph.*/
    
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
    int ldist = 5; //this threshold specifies the distance between the camera and the landmark.
    int onoise = 10; //the threshold specifies at what point factors are discarded due to reprojection error.
    gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> sppf(1, -1, false, false, boost::none, gtsam::HESSIAN, ldist, onoise);
    
    //camera_keys[i]-posenum + poses.size();
    int tosub = 0;
    if(posenum > N_PAST_SETS)
        tosub = posenum - N_PAST_SETS;
//    int base_num = poses.size() - posenum - 1; //max(posenum, N_PAST_SETS);
    for(int i=0; i<points.size(); i++) {
        if(camera_keys[i] < posenum) continue;
        gtsam::Symbol s('x', camera_keys[i]-tosub);
        sppf.add(points[i], s, pixelNoise, k);
    }
    
    landmark_factors.push_back(sppf);
    landmark_keys.push_back(landmark_key);
    graph.add(sppf);
}

gtsam::Values VisualOdometry::RunBA(){
    gtsam::Values result;
    try {
        //result = gtsam::LevenbergMarquardtOptimizer(graph, initEst).optimize();
        result = gtsam::DoglegOptimizer(graph, initEst).optimize();
    } catch(const std::exception& ex) {
        if(debug) std::cout<<"Pose triangulation failed. \n"<<std::endl;
    }
    return result;
}

void VisualOdometry::Reset() {
    if(poses.size() > N_PAST_SETS) {
        poses.erase(poses.begin(), poses.begin()+poses.size()-N_PAST_SETS);
    }
    if(landmark_sets.size() > N_PAST_SETS-1) {
        landmark_sets.erase(landmark_sets.begin(), landmark_sets.begin()+landmark_sets.size()-(N_PAST_SETS-1));
    }
    landmark_factors.clear();
    landmark_keys.clear();
    graph.resize(0);
    initEst.clear();
}

gtsam::Pose3 VisualOdometry::PoseFromEssential(ParseFeatureTrackFile& last, ParseFeatureTrackFile& latest){
    //using the method to recover the pose from the essential matrix. (for the initial poses)
    std::vector<cv::Point2f> p2d0;
    std::vector<cv::Point2f> p2d1;
    
    int ci = 0;
    for(int i=0; i<last.ids.size(); i++){
        while(latest.ids[ci]<last.ids[i]) ci++;
        if(latest.ids[ci] != last.ids[i]) continue;
        p2d0.push_back(cv::Point2f(last.imagecoord[i].x(), last.imagecoord[i].y()));
        p2d1.push_back(cv::Point2f(latest.imagecoord[ci].x(), latest.imagecoord[ciß].y()));
    }
    
    if(p2d0.size() < 15)
        return gtsam::Pose3::identity();
    
    double focal = 1.0;
    cv::Point2d pp(0, 0);
    
    cv::Mat E, R, t, mask;
    std::vector<cv::Point2f> p0(p2d0.size());
    std::vector<cv::Point2f> p1(p2d1.size());
    cv::undistortPoints(p2d0, p0, _cam.IntrinsicMatrix(), _cam.Distortion());
    cv::undistortPoints(p2d1, p1, _cam.IntrinsicMatrix(), _cam.Distortion());
    E = findEssentialMat(cv::Mat(p1), cv::Mat(p0), focal, pp, cv::RANSAC, 0.999, 0.0001,  mask);
    cv::recoverPose(E, cv::Mat(p1), cv::Mat(p0), R, t, focal, pp, mask);
    //note: at this point, t is the unit translation. The relative translation has to be recovered using the triangulated 3D points and their distances.
    //alternatively, using the IMU, the absolute translation can be recovered.
    //the camera height is useful if the points are triangulated, but that's a lot of error. it's useful if the ground plane is estimated, but that is also prone to error.
    //TBD.
    
    gtsam::Rot3 rot(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
    gtsam::Point3 trans(t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2));
    return gtsam::Pose3(rot, trans);
}

gtsam::Pose3 VisualOdometry::PnP(gtsam::Values& result, gtsam::Pose3 est, ParseFeatureTrackFile& latest) {
    /* Use the solution from optimization to determine the location of the next pose.
     */
    if(latest.ids.size() < 8) return est; //too few points.
    
    std::vector<gtsam::Point3> p3d;
    std::vector<gtsam::Point2> p2d1;
    int ci=0;
    int nnonzero = 0;
    int i=landmark_keys.size()-1;
    while(landmark_keys[i]>latest.ids[0]) i--;
    for(; i<landmark_keys.size(); i++) {
        while(latest.ids[ci] < landmark_keys[i]) ci++;
        if(latest.ids[ci] != landmark_keys[i]) continue;
        
        gtsam::Point3 p = landmark_factors[i].point(result).get();
        p3d.push_back(p);
        p2d1.push_back(latest.imagecoord[ci]);
        if(p.x() == 0 && p.y() == 0 && p.z() == 0) nnonzero++;
    }
    
    if(p3d.size() - nnonzero == 0) {
        std::cout << "Points are zero. cannot run pnp." << std::endl;
        std::cout << "Estimating pose from the essential matrix instead." << std::endl;
        return PoseFromEssential(lastPFT, latest);
    }
    if(p3d.size() < 8) return est; //too few points.
    
    std::vector<double> inliers(p3d.size(), 1);
    LocalizePose lp(_cam);
    std::vector<double> pguess = lp.PoseToVector(est);
    std::vector<std::vector<double> > res = lp.UseBAIterative(pguess, p3d, p2d1, inliers);
    if(res.size()==0) return est; //no solution was found.
    return lp.VectorToPose(res[0]);
}

void VisualOdometry::CopyLast(ParseFeatureTrackFile& PFT){
    lastPFT.Reset();
    for(int i=0; i<PFT.ids.size(); i++){
        lastPFT.ids.push_back(PFT.ids[i]);
        lastPFT.imagecoord.push_back(PFT.imagecoord[i]);
    }
}

gtsam::Pose3 VisualOdometry::GetNextOdom(ParseFeatureTrackFile& PFT){
    gtsam::Pose3 est, estnewpose = _prior;
    if(posenum > 0) est = poses[poses.size()-1].compose(last_odom);
    else est = _prior;
    if(posenum > 1) {
        estnewpose = PoseFromEssential(lastPFT, PFT);
        //Xiao: this method still has a scale ambiguity. How you want to solve it is still TBD.
        last_odom = poses[poses.size()-1].between(estnewpose);
    } else if(posenum > 1) {
        //for PnP to work, the initial estimates have to be close.
        estnewpose = PnP(result, est, PFT); //note, there's a chance PnP could fail.
        last_odom = poses[poses.size()-1].between(estnewpose);
    } else last_odom = gtsam::Pose3::identity();
    
    Reset();
    ConstructGraph(estnewpose, PFT);
    if(posenum > 0) result = RunBA();
    
    posenum++;

    CopyLast(PFT);
    return last_odom;
}

void VisualOdometry::PrintVec(std::vector<double> p){
    for(int i=0; i<p.size(); i++){
        std::cout << p[i]<<",";
    }
    std::cout << std::endl;
}

std::vector<double> VisualOdometry::PoseToVector(gtsam::Pose3& cam){
    return {cam.x(), cam.y(), cam.z(), cam.rotation().roll(), cam.rotation().pitch(), cam.rotation().yaw()};
}









