/*
 * LocalizePose.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: shane
 */




#include <gtsam/base/Matrix.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/NoiseModel.h>

#include <random>
#include <unordered_map>
#include <cmath>

#include "LocalizationFactor.h"
#include "VirtualBetweenFactor.h"
#include "LocalizePose.hpp"

void LocalizePose::PrintVec(std::vector<double> p){
    for(int i=0; i<p.size(); i++){
        std::cout << p[i]<<",";
    }
    std::cout << std::endl;
}

std::vector<double> LocalizePose::PoseToVector(gtsam::Pose3& cam){
    return {cam.x(), cam.y(), cam.z(), cam.rotation().roll(), cam.rotation().pitch(), cam.rotation().yaw()};
}

gtsam::Pose3 LocalizePose::VectorToPose(std::vector<double>& p){
    return gtsam::Pose3(gtsam::Rot3::ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

bool LocalizePose::EmptyPose(gtsam::Pose3& p){
    gtsam::Pose3 zeros = gtsam::Pose3::identity();
    return zeros.equals(p);
}

double LocalizePose::MeasureReprojectionError(std::vector<double>& pnppose, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<unsigned char>& inliers){
    gtsam::Pose3 gtp = VectorToPose(pnppose);
    double sumall=0;
    double sumin=0;
    int cin=0;

    int count=0;
    for(int i=0; i<p3d.size(); i++){
        gtsam::Point3 tfp = gtp.transform_to(p3d[i]);
        gtsam::Point2 res = _cam.ProjectToImage(tfp);
        if(!_cam.InsideImage(res)) continue;
        double dist = res.dist(p2d[i]);
        sumall+=dist;
        count++;
        if(inliers[i]){
            sumin+=dist;
            cin++;
        }
    }
    if(debug) std::cout<<"num points: " << count << ", reprojection error: "<<sumall/count<<", inliers only: "<<sumin/cin<< ", "<< cin << " of " << count << " are inliers." << std::endl;

    return sumall/count;
}

std::vector<double> LocalizePose::Maximization(gtsam::Pose3& gtp, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers, double err){
    int nchanges=0;
    int ninliers = 0;
    double sumall=0;
    double sumin=0;
    for(int i=0; i<p3d.size(); i++){
        gtsam::Point3 tfp = gtp.transform_to(p3d[i]);
        gtsam::Point2 res = _cam.ProjectToImage(tfp);
        double dist = res.dist(p2d[i]);
        if(dist > err || !_cam.InsideImage(res)){
            if(inliers[i]>=0.0) nchanges++;
            inliers[i] = -1;
            if(!_cam.InsideImage(res)) continue;
        }
        else{
            if(inliers[i]<0) nchanges++;
            inliers[i] = err;//dist; //I seem to get more reliable estimates using err, rather than dist, here.
            sumin+=dist;
            ninliers++;
        }
        sumall+=dist;
    }
    return {(double)nchanges, (double)ninliers, sumall/p3d.size(), sumin/ninliers};
}

std::vector<std::vector<double> > LocalizePose::DualIterativeBA(gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 p1frame0, gtsam::Pose3 p0frame1,
        std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
        std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1){

    std::vector<double> posevals(4, 0.0);
    int minpiter = -1;
    double best_score;
    
    int nchanges=1;
    int i=0;
    double err = ACCEPTABLE_TRI_RERROR;
    double val = ACCEPTABLE_INTERPOSE_VAR;
    for(; nchanges > 0 && i < MAX_ITERS; i++){
        bool success = DualBA(val,
                              p0, p1frame0, f3d, f2d1, rerror0,
                              p1, p0frame1, b3d, b2d0, rerror1);
        if(!success) break;
        std::vector<double> pv0 = Maximization(p1frame0, f3d, f2d1, rerror0, err);
        std::vector<double> pv1 = Maximization(p0frame1, b3d, b2d0, rerror1, err);
        
        posevals[0] = pv0[0]+pv1[0];
        posevals[1] = pv0[1]+pv1[1];
        posevals[2] = (pv0[2]*f3d.size()+pv1[2]*b3d.size())/(f3d.size()+b3d.size());
        posevals[3] = (pv0[3]*pv0[1] + pv1[3]*pv1[1])/(pv0[1]+pv1[1]);

        nchanges = posevals[0];
        if(debug) {
            printf("iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
                   (int)i, (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)(f3d.size()+b3d.size()));
        }
    }
    
    if(i==0) return {};
    
    if(debug) {
        printf("iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int)i, (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)(f3d.size()+b3d.size()));
    }

    posevals[0] = i;
    return {PoseToVector(p1frame0), PoseToVector(p0frame1), posevals};
}

bool LocalizePose::DualBA(double val,
        gtsam::Pose3 p0, gtsam::Pose3& p1frame0, std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
        gtsam::Pose3 p1, gtsam::Pose3& p0frame1, std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1){
    
    gtsam::Symbol symb1('x', 0);
    gtsam::Symbol symb3('x', 1);
    
    std::vector<double> flexible = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5};//0.1, 0.1, 0.25};
    
    AddLocalizationFactors(symb1, f3d, f2d1, rerror0);
    AddLocalizationFactors(symb3, b3d, b2d0, rerror1);
    AddPose(symb1, p1frame0, flexible);
    AddPose(symb3, p0frame1, flexible);
    
    //The method doesn't converge unless the poses are already close to their final values.
    //An alternative is to start this constraint loose and then tighten it after a few iterations, but
    //that more frequently caused exceptions during optimization.
    gtsam::Vector6 v6;
    v6.setConstant(val);
    gtsam::noiseModel::Diagonal::shared_ptr btwnnoise = gtsam::noiseModel::Diagonal::Sigmas(v6);
    graph.add(VirtualBetweenFactor(symb1, symb3, p0, p1, btwnnoise));
    
    gtsam::Values result = RunBA();
    if(result.size()==0) return false;
    
    p1frame0 = result.at<gtsam::Pose3>(symb1);
    p0frame1 = result.at<gtsam::Pose3>(symb3);
    return true;
}

std::vector<std::vector<double> > LocalizePose::UseBAIterative(std::vector<double> pguess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers){
    //TODO: make this function consistent with the DualIterativeBA(). (and update RANSAC_BA() as well)
    //EM approach to finding the best pose.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
    gtsam::Pose3 best_pose;
    std::vector<double> best_posevals;
    double best_score;
    int minpiter = -1;
    gtsam::Pose3 estp = VectorToPose(pguess);
    
    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p1frame0.
    std::vector<double> posevalsransac = RANSAC_BA(estp, p3d, p2d, inliers);
    if(posevalsransac[1]<0.000001) return {};

    //measure rerror with the previous set of inliers, if it's good, update the set of inliers.
    int iters = 0;
    int nchanges=1;
    double err = ACCEPTABLE_TRI_RERROR;
    while(err>6.0 ||(nchanges > 0 && iters < MAX_ITERS)){
        UseBA(estp, p3d, p2d, inliers);
        if(EmptyPose(estp)) break;
        std::vector<double> posevals = Maximization(estp, p3d, p2d, inliers, err);
        
        if(iters==0 || posevals[3]<best_score){
            best_score = posevals[3];
            best_pose = estp;
            best_posevals = posevals;
            minpiter = iters;
        }
        iters++;
        nchanges = posevals[0];

        if(debug) {
            printf("iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
                    (int)iters, (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)p3d.size());
        }
    }
    if(minpiter != iters-1) Maximization(best_pose, p3d, p2d, inliers, err); //reset the inliers.
    if(best_posevals.size()==0)return {};
    best_posevals[0] = iters;
    return {PoseToVector(best_pose), best_posevals};
}

void LocalizePose::UseBA(gtsam::Pose3& pguess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers){
    gtsam::Symbol symb('x', 0);
    std::vector<double> flexible = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5};//0.1, 0.1, 0.25};
    AddPose(symb, pguess, flexible);
    AddLocalizationFactors(symb, p3d, p2d, inliers);
    gtsam::Values result = RunBA();
    if(result.size()==0) pguess = gtsam::Pose3::identity();
    else pguess = result.at<gtsam::Pose3>(symb);
}

void LocalizePose::AddPose(gtsam::Symbol symb, gtsam::Pose3 pguess, std::vector<double> noise){
    gtsam::Vector6 v6 = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&noise[0]), 6, 1);
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas(v6);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(symb, pguess, poseNoise));
    initEst.insert(symb, pguess);
}

void LocalizePose::AddLocalizationFactors(gtsam::Symbol symb, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers){
    for(int i=0; i<p2d.size(); i++){
        if(inliers[i]<0.0) continue;
        gtsam::noiseModel::Isotropic::shared_ptr measurementNoise1 = gtsam::noiseModel::Isotropic::Sigma(2, inliers[i]);
        graph.push_back(LocalizationFactor<gtsam::Pose3, gtsam::Cal3_S2>(p2d[i], p3d[i], measurementNoise1, symb, _cam.GetGTSAMCam()));
    }
}

gtsam::Values LocalizePose::RunBA(){
    gtsam::Values result;
    try{
        //result = gtsam::LevenbergMarquardtOptimizer(graph, initEst).optimize();
        result = gtsam::DoglegOptimizer(graph, initEst).optimize();
    }catch(const std::exception& ex){
        if(debug) std::cout<<"Pose triangulation failed. \n"<<std::endl;//<<ex.what()
    }
    graph.resize(0);
    initEst.clear();
    return result;
}

void LocalizePose::GenerateRandomSet(int n, std::vector<int>& rset){
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<int> uniform_dist(0, n-1);
    int added = 0;
    std::unordered_map<int, bool> used;
    while(added<rset.size()){
        int cur = uniform_dist(e1);
        auto search = used.find(cur);
        if(search != used.end()) continue;
        used[cur] = true;
        rset[added] = cur;
        added++;
    }
}

std::vector<double> LocalizePose::RANSAC_BA(gtsam::Pose3& p1guess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers){
    //EM approach to finding the best pose.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
    gtsam::Pose3 best_pose = p1guess;
    std::vector<int> rset(MIN_CORRESPONDENCES);
    std::vector<gtsam::Point3> subp3d(rset.size());
    std::vector<gtsam::Point2> subp2d1(rset.size());
    std::vector<double> subinliers(rset.size(), ACCEPTABLE_TRI_RERROR);
    std::vector<double> best_posevals(4, 0.0);
    int iters=0;
    int last_save_iter = 0;
    double err = ACCEPTABLE_TRI_RERROR;
    for(; iters<MAX_RANSAC_ITERS; iters++){
        GenerateRandomSet(p3d.size(), rset);
        for(int j=0; j<rset.size(); j++){
            subp3d[j] = p3d[rset[j]];
            subp2d1[j] = p2d1[rset[j]];
        }
        gtsam::Pose3 estp = best_pose;
        UseBA(estp, subp3d, subp2d1, subinliers);
        if(EmptyPose(estp)) continue;
        std::vector<double> posevals = Maximization(estp, p3d, p2d1, inliers, err);
        if(posevals[1]>best_posevals[1]){
            swap(best_posevals, posevals);
            best_pose = estp;
            last_save_iter = iters;
        }

        if(best_posevals[1]/p3d.size()>RANSAC_PERC_DC || iters-last_save_iter>=RANSAC_IMPROV_ITERS) break;
    }

    if(debug) {
        printf("iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int)iters, (int)best_posevals[0], best_posevals[2], best_posevals[3], (int)best_posevals[1], (int)p3d.size());
    }

    p1guess = best_pose;
    best_posevals[0] = iters;
    return best_posevals;
}

std::vector<std::vector<double> > LocalizePose::RobustDualBA(std::vector<double> p0, std::vector<double> p1,
                                                            std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1, std::vector<double>& rerrorp,
                                                            std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerrorb){
    gtsam::Pose3 gp0 = VectorToPose(p0);
    gtsam::Pose3 gp1 = VectorToPose(p1);
    gtsam::Pose3 p1frame0(gp1.rotation(), gp0.translation());
    
    if(p3d.size()<MIN_CORRESPONDENCES || b3d.size()<MIN_CORRESPONDENCES)  {
        std::cout << "Localization failed due to sizes: " <<p3d.size() << ", " << b3d.size() << std::endl;
        return {};
    }

    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p1frame0.
    std::vector<double> posevals1f0 = RANSAC_BA(p1frame0, p3d, p2d1, rerrorp);
    if(posevals1f0[1]<0.000001){
        std::cout << "Localization failed due to RANSAC failure on set p"<<std::endl;
        return {};
    }
    
    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p0frame1.
    //gtsam::Pose3 p0frame1 = gp1.compose(p1frame0.between(gp0)); //this also looks wrong.
    gtsam::Pose3 p0frame1 = gp1.compose(gp1.between(p1frame0)*p1frame0.between(gp0)*p1frame0.between(gp1));
    std::vector<double> posevals0f1 = RANSAC_BA(p0frame1, b3d, b2d0, rerrorb);
    if(posevals0f1[1]<0.000001) {
        std::cout << "Localization failed due to RANSAC failure on set b"<<std::endl;
        return {};
   }

    //iterative localization (like EM), started with the good initial estimates found using RANSAC.
    return DualIterativeBA(gp0, gp1, p1frame0, p0frame1, p3d, p2d1, rerrorp, b3d, b2d0, rerrorb);
}

