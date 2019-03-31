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

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <RFlowOptimization/LocalizePose.hpp>

#include "Optimization/SingleSession/GTSamInterface.h"

using namespace std;

void VisualOdometry::AddLandmarkTracks(vector<LandmarkTrack>& landmarks){
    for(int i=0; i<landmarks.size(); i++){
        AddLandmarkTrack(_cam.GetGTSAMCam(), landmarks[i].key, landmarks[i].points, landmarks[i].camera_keys, landmarks[i].used);
    }
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
    vector<LandmarkTrack> inactive = PFT.ProcessNewPoints((int)'x', posenum, active);
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

void VisualOdometry::AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, int landmark_key, vector<gtsam::Point2>& points, vector<gtsam::Symbol> camera_keys, bool used){
    /*Add the landmark track to the graph.*/
    
    //landmarkDistanceThreshold - if the landmark is triangulated at a distance larger than that the factor is considered degenerate
    //dynamicOutlierRejectionThreshold - if this is nonnegative the factor will check if the average reprojection error is smaller than this threshold after triangulation,
    //  and the factor is disregarded if the error is large
    int ldist = 5; //this threshold specifies the distance between the camera and the landmark.
    int onoise = 10; //the threshold specifies at what point factors are discarded due to reprojection error.
    
#ifdef GTSAM4
    gtsam::SmartProjectionParams params;
    params.setLandmarkDistanceThreshold(ldist);
    params.setDynamicOutlierRejectionThreshold(onoise);
    gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> sppf(pixelNoise, k, boost::none, params);
#else
    gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> sppf(1, -1, false, false, boost::none, gtsam::HESSIAN, ldist, onoise); //GTSAM 3.2.1
#endif
    
    //camera_keys[i]-posenum + poses.size();
    int tosub = 0;
    if(posenum > N_PAST_SETS)
        tosub = posenum - N_PAST_SETS;
//    int base_num = poses.size() - posenum - 1; //max(posenum, N_PAST_SETS);
    for(int i=0; i<points.size(); i++) {
        int idx  = camera_keys[i].index();
        if(idx < posenum) continue;
        gtsam::Symbol s(camera_keys[i].chr(), idx-tosub);
#ifdef GTSAM4
        sppf.add(points[i], s); //GTSAM 4.0
#else
        sppf.add(points[i], s, pixelNoise, k); //GTSAM 3.2.1
#endif
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

cv::Mat
VisualOdometry::triangulatePointSet(std::vector<cv::Point2f>& p0, std::vector<cv::Point2f>& p1, cv::Mat P1) {
    cv::Mat P0 = cv::Mat::eye(3, 4, P1.type());
    cv::Mat Q;
    triangulatePoints(P0, P1, p0, p1, Q);
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    cv::Mat W = Q(cv::Range(0, 3), cv::Range::all());
    if( W.checkVector(3) < 0)
        cv::transpose(W,W);
    return W;
}

std::pair<double, int> VisualOdometry::getReprojectionError(cv::Mat , std::vector<cv::Point2f>& p2d1, cv::Mat P1) {
    double INLIER_THRESHOLD = 6;
    
    cv::Mat P0 = cv::Mat::eye(3, 4, P1.type());
    cv::Mat Q;
    triangulatePoints(P0, P1, p0, p1, Q);
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    cv::Mat W = Q(cv::Range(0, 3), cv::Range::all());
    if( W.checkVector(3) < 0)
        cv::transpose(W,W);
    cv::Mat R(3,3, P1.type());
    cv::Mat t(3,1, P1.type());
    R = P1(cv::Range::all(), cv::Range(0, 3));
    t = P1.col(3);
    std::vector<cv::Point2f> projected(W.checkVector(3));
    cv::projectPoints(W, R, t, _cam.IntrinsicMatrix(), _cam.Distortion(), cv::Mat(projected));
    
    int cin = 0;
    double sum = 0;
    int nnan = 0;
    for(int i=0; i<p2d1.size(); i++) {
        if(isnan(projected[i].x)) {
            nnan++;
            continue;
        }
        double d = pow(pow(projected[i].x - p2d1[i].x, 2.0) + pow(projected[i].y - p2d1[i].y, 2.0), 0.5);
        sum += d;
        if(d < INLIER_THRESHOLD)
            cin++;
    }
    
    if(nnan == p2d1.size()) {
        return std::make_pair(100000000000000, 0);
    }
    
    return std::make_pair(sum/p2d1.size(), cin);
}

std::pair<std::vector<cv::Point2f>, std::vector<int>>
VisualOdometry::findOverlappingPointSet(std::vector<gtsam::Point2>& ic1, std::vector<int>& id1, std::vector<int>& ids) {
    std::vector<cv::Point2f> subpoints;
    std::vector<int> subids;
    
    int ci = 0;
    for(int i=0; i<ids.size(); i++){
        while(id1[ci]<ids[i]) ci++;
        if(id1[ci] != ids[i]) continue;
        subpoints.push_back(cv::Point2f(ic1[ci].x(), ic1[ci].y()));
        ci++;
    }
    return std::make_pair(subpoints, subids);
}

std::pair<gtsam::Pose3, int> VisualOdometry::PoseFromEssential(ParseFeatureTrackFile& last, ParseFeatureTrackFile& latest) {
    double MIN_REQ_CORRESPONDENCES = 5; // for the five point algorithm
    //using the method to recover the pose from the essential matrix. (for the initial poses)
//    std::vector<cv::Point2f> p2d0;
//    std::vector<cv::Point2f> p2d1;
//    
//    int ci = 0;
//    for(int i=0; i<last.ids.size(); i++){
//        while(latest.ids[ci]<last.ids[i]) ci++;
//        if(latest.ids[ci] != last.ids[i]) continue;
//        p2d0.push_back(cv::Point2f(last.imagecoord[i].x(), last.imagecoord[i].y()));
//        p2d1.push_back(cv::Point2f(latest.imagecoord[ci].x(), latest.imagecoord[ci].y()));
//        ci++;
//    }
    
    std::pair<std::vector<cv::Point2f>, std::vector<int>> res1 = findOverlappingPointSet(last.imagecoord, last.ids, latest.ids);
    std::pair<std::vector<cv::Point2f>, std::vector<int>> res2 = findOverlappingPointSet(latest.imagecoord, latest.ids, res1.second);
    std::vector<cv::Point2f> p2d0 = res1.first;
    std::vector<cv::Point2f> p2d1 = res2.first;
    
    if(p2d0.size() < MIN_REQ_CORRESPONDENCES)
        return std::make_pair(gtsam::Pose3::identity(), 0);
    
    std::vector<cv::Point2f> p0(p2d0.size());
    std::vector<cv::Point2f> p1(p2d1.size());
    cv::undistortPoints(p2d0, p0, _cam.IntrinsicMatrix(), _cam.Distortion());
    cv::undistortPoints(p2d1, p1, _cam.IntrinsicMatrix(), _cam.Distortion());
    
    double focal = 1.0;
    cv::Point2d pp(0, 0);
    std::vector<unsigned char> inliers(p2d0.size(), 0);
    cv::Mat E, R, t, mask;
    E = findEssentialMat(cv::Mat(p0), cv::Mat(p1), focal, pp, cv::RANSAC, 0.999, 0.0001,  cv::Mat(inliers));
    cv::recoverPose(E, cv::Mat(p2d1), cv::Mat(p2d0), _cam.IntrinsicMatrix(), R, t, cv::Mat(inliers));
    
    cv::Mat P(3, 4, R.type());
    P(cv::Range::all(), cv::Range(0, 3)) = R * 1.0;
    P.col(3) = t * 1.0;
    std::pair<double, int> reprojected = getReprojectionError(p0, p1, p2d1, P);
    std::cout << "reprojection error: " << reprojected.first << ", inliers: " << reprojected.second << std::endl;
    
    gtsam::Rot3 rot(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
    gtsam::Point3 trans(t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2));
    
    return std::make_pair(gtsam::Pose3(rot, trans), reprojected.second);
}

int VisualOdometry::testAndVerifyVO(std::vector<ParseFeatureTrackFile&> t) {
    std::vector<std::vector<cv::Point2f>> p2d(t.size());
    std::vector<int> ids;
    std::vector<cv::Mat> P;
    
    findOverlappingPointSet();
    
    std::pair<std::vector<cv::Point2f>, std::vector<int>> res1 = findOverlappingPointSet(t[0].imagecoord, t[0].ids, t[1].ids);
    std::pair<std::vector<cv::Point2f>, std::vector<int>> res2 = findOverlappingPointSet(t[1].imagecoord, t[1].ids, res1.second);
    std::pair<std::vector<cv::Point2f>, std::vector<int>> res3 = findOverlappingPointSet(t[2].imagecoord, t[2].ids, res1.second);
    std::vector<cv::Point2f> p2d0 = res1.first;
    std::vector<cv::Point2f> p2d1 = res2.first;
    
    P[2] = PoseFromEssential(t[1], t[2]);
    P[1] = PoseFromEssential(t[0], t[1]);
    P[0] =  = cv::Mat::eye(3, 4, P[1].type());
    
    
    
    
    
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
        std::pair<gtsam::Pose3, int> vop = PoseFromEssential(lastPFT, latest);
        return vop.first;
    }
    if(p3d.size() < 8) return est; //too few points.
    
    std::vector<double> inliers(p3d.size(), 1);
    LocalizePose lp(_cam);
    std::vector<double> pguess = GTSamInterface::PoseToVector(est);
    std::vector<std::vector<double> > res = lp.UseBAIterative(pguess, p3d, p2d1, inliers);
    if(res.size()==0) return est; //no solution was found.
    return GTSamInterface::VectorToPose(res[0]);
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
        std::pair<gtsam::Pose3, int> vop = PoseFromEssential(lastPFT, PFT);
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

std::pair<double, int> VisualOdometry::KeypointChange(ParseFeatureTrackFile& last, ParseFeatureTrackFile& latest) {
    if(latest.ids.size()==0 or last.ids.size()==0){
        return std::make_pair(-1, -1);
    }
    
    double sumdist = 0;
    int count = 0;
    int ci = 0;
    for(int i=0; i<last.ids.size(); i++){
        while(latest.ids[ci]<last.ids[i]) ci++;
        if(latest.ids[ci] != last.ids[i]) continue;
        
        double dist = last.imagecoord[i].dist(latest.imagecoord[ci]);
        sumdist += dist;
        count++;
    }
    return std::make_pair(sumdist/count, count);
}

void VisualOdometry::PrintVec(std::vector<double> p){
    for(int i=0; i<p.size(); i++){
        std::cout << p[i]<<",";
    }
    std::cout << std::endl;
}









