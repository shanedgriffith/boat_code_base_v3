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

#include "Optimization/SingleSession/GTSAMInterface.h"
#include "Optimization/MultiSession/LocalizationFactor.h"
#include "Optimization/MultiSession/VirtualBetweenFactor.h"
#include "LocalizePose.hpp"
#include "P3P.hpp"

#include "FileParsing/ParseOptimizationResults.h"
#include "FileParsing/ParseFeatureTrackFile.h"

void LocalizePose::PrintVec(std::vector<double> p){
    for(int i=0; i<p.size(); i++){
        std::cout << p[i]<<",";
    }
    std::cout << std::endl;
}

bool LocalizePose::EmptyPose(gtsam::Pose3& p){
    gtsam::Pose3 zeros = gtsam::Pose3::identity();
    return zeros.equals(p);
}

double LocalizePose::MeasureReprojectionError(std::vector<double>& pnppose, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<unsigned char>& inliers){
    gtsam::Pose3 gtp = GTSAMInterface::VectorToPose(pnppose);
    double sumall=0;
    double sumin=0;
    int cin=0;

    int count=0;
    for(int i=0; i<p3d.size(); i++){
        gtsam::Point3 tfp = gtp.transform_to(p3d[i]);
        gtsam::Point2 res = _cam.ProjectToImage(tfp);
        if(!_cam.InsideImage(res)) continue;
        double dist = res.distance(p2d[i]);
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
        double dist = res.distance(p2d[i]);
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

double LocalizePose::GetBestValueForInterposeVar(gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 p1frame0, gtsam::Pose3 p0frame1,
                                     std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
                                                 std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1){
    
    std::vector<double> posevals(4, 0.0);
    std::vector<double> inliers0(rerror0.size(), ACCEPTABLE_TRI_RERROR);
    std::vector<double> inliers1(rerror1.size(), ACCEPTABLE_TRI_RERROR);
    double bestval = 0.005;
    int maxinliers = -1;
    double err = ACCEPTABLE_TRI_RERROR;
    //0.25, 0.125, 0.0625, 0.03125, 0.0156, 0.078
    for(double val=0.005; val <= 0.4; val*=2){
        bool success = DualBA(val,
                         p0, p1frame0, f3d, f2d1, rerror0,
                         p1, p0frame1, b3d, b2d0, rerror1);
        if(!success) continue;
        
        std::vector<double> pv0 = Maximization(p1frame0, f3d, f2d1, inliers0, err);
        std::vector<double> pv1 = Maximization(p0frame1, b3d, b2d0, inliers1, err);
        
        posevals[0] = pv0[0]+pv1[0];
        posevals[1] = pv0[1]+pv1[1];
        posevals[2] = (pv0[2]*f3d.size()+pv1[2]*b3d.size())/(f3d.size()+b3d.size());
        posevals[3] = (pv0[3]*pv0[1] + pv1[3]*pv1[1])/(pv0[1]+pv1[1]);
        
        if(posevals[1] > maxinliers) {
            maxinliers = posevals[1];
            bestval = val;
        }
    }
    
    return bestval;
}

std::vector<std::vector<double> > LocalizePose::DualIterativeBA(gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 p1frame0, gtsam::Pose3 p0frame1,
        std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
        std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1){

    std::vector<double> posevals(4, 0.0);
    int minpiter = -1;
    double best_score;
    
    bool success = false;
    int nchanges=1;
    int i=0;
    double err = ACCEPTABLE_TRI_RERROR;
    double val = GetBestValueForInterposeVar(p0, p1, p1frame0, p0frame1, f3d, f2d1, rerror0, b3d, b2d0, rerror1);
    if(debug) std::cout << "Best Value For Interpose Var: " << val << std::endl;
    for(; nchanges > 0 && i < MAX_ITERS; i++){
        success = DualBA(val,
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
    
    if(debug && success) {
        printf("iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int) (i-1), (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)(f3d.size()+b3d.size()));
    }
    
    posevals[0] = i;
    return {GTSAMInterface::PoseToVector(p1frame0), GTSAMInterface::PoseToVector(p0frame1), posevals};
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
    
    if(p3d.size() < MIN_CORRESPONDENCES)
    {
        return {};
    }
    gtsam::Pose3 best_pose;
    std::vector<double> best_posevals;
    double best_score;
    int minpiter = -1;
    gtsam::Pose3 estp = GTSAMInterface::VectorToPose(pguess);
    
    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p1frame0.
    std::vector<double> posevalsransac = RANSAC_P3P(estp, p3d, p2d, inliers);
    if(posevalsransac[1]<0.000001) return {};
    
    //measure rerror with the previous set of inliers, if it's good, update the set of inliers.
    int iters = 0;
    int nchanges=1;
    double err = ACCEPTABLE_TRI_RERROR;
    //while(err>6.0 ||(nchanges > 0 && iters < MAX_ITERS)){
    for(int i=0; nchanges > 0 || i < MAX_ITERS; i++) {
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
            printf("bai iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
                    (int)iters, (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)p3d.size());
        }
    }
    if(minpiter != iters-1) Maximization(best_pose, p3d, p2d, inliers, err); //reset the inliers.
    if(best_posevals.size()==0)return {};
    best_posevals[0] = iters;
    return {GTSAMInterface::PoseToVector(best_pose), best_posevals};
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
    std::vector<int> rset(15); //15 because it's using BA to solve for the pose. MIN_CORRESPONDENCES);
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
        
        //makes RANSAC faster by 10x (1ms to 10ms), but it's less consistent
        //if(best_posevals[1]/p3d.size()>RANSAC_PERC_DC || iters-last_save_iter>=RANSAC_IMPROV_ITERS) break;
    }
    
    if(debug) {
        printf("ransac iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int)iters, (int)best_posevals[0], best_posevals[2], best_posevals[3], (int)best_posevals[1], (int)p3d.size());
    }

    //have to reset the inliers.
    std::vector<double> posevals = Maximization(best_pose, p3d, p2d1, inliers, err);
    
    p1guess = best_pose;
    best_posevals[0] = iters;
    return best_posevals;
}

gtsam::Pose3 LocalizePose::disambiguatePoses(const std::vector<gtsam::Pose3>& poses, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d)
{
    int bestpidx = -1;
    double bestdist = 100000000;
    for(int i=0; i<poses.size(); ++i)
    {
        double sumd = 0;
        for(int j=3; j<p3d.size(); ++j)
        {
            gtsam::Point3 tfp = poses[i].transform_to(p3d[i]);
            gtsam::Point2 res = _cam.ProjectToImage(tfp);
            sumd += res.distance(p2d[i]);
        }
        
        if(sumd < bestdist)
        {
            bestdist = sumd;
            bestpidx = i;
        }
    }
    
    if(bestpidx == -1)
    {
        std::cout << "LocalizePose::disambiguatePoses() failed. the reprojection error of the extra set of points is greater than 100000000 for the four pose candidates. " << std::endl;
        return gtsam::Pose3::identity();
    }
    
    return poses[bestpidx];
}

gtsam::Pose3 LocalizePose::RunP3P(std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1)
{
    /*should have 4 points. the first three are used for the estimation. the fourth for disambiguation.*/
    std::vector<gtsam::Vector3> featureVectors(p3d.size());
    
    static gtsam::Cal3_S2::shared_ptr gtcam = _cam.GetGTSAMCam();
    static gtsam::Pose3 p; //the identity pose.
    gtsam::PinholeCamera<gtsam::Cal3_S2> pc(p, *gtcam);
    for(int j=0; j<p3d.size(); j++)
    {
        gtsam::Unit3 uv = pc.backprojectPointAtInfinity(p2d1[j]);
        featureVectors[j] = uv.unitVector();
    }
    
    std::vector<gtsam::Pose3> poses;
    if(P3P::computePoses(featureVectors, p3d, poses) < 0)
        return gtsam::Pose3::identity();
    
    return disambiguatePoses(poses, p3d, p2d1);
}

std::vector<double> LocalizePose::RANSAC_P3P(gtsam::Pose3& p1guess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers){
    //EM approach to finding the best pose.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
    gtsam::Pose3 best_pose = p1guess;
    int kdisambiguation = 1; // pick k random points to use for disambiguation, which are different from the set used to compute the localiation
    std::vector<int> rset(3+kdisambiguation);
    std::vector<gtsam::Point3> subp3d(rset.size());
    std::vector<gtsam::Point2> subp2d1(rset.size());
    std::vector<double> subinliers(rset.size(), ACCEPTABLE_TRI_RERROR);
    std::vector<double> best_posevals(4, 0.0);
    int iters=0;
    int last_save_iter = 0;
    double err = ACCEPTABLE_TRI_RERROR;
    
    for(; iters<MAX_RANSAC_ITERS; ++iters) {
        GenerateRandomSet(rset.size(), rset);
        for(int j=0; j<rset.size(); j++)
        {
            subp3d[j] = p3d[rset[j]];
            subp2d1[j] = p2d1[rset[j]];
        }
        
        gtsam::Pose3 estp = RunP3P(subp3d, subp2d1);
        
        std::vector<double> posevals = Maximization(estp, p3d, p2d1, inliers, err);
        if(posevals[1]>best_posevals[1]){
            swap(best_posevals, posevals);
            best_pose = estp;
            last_save_iter = iters;
        }
        
        //makes RANSAC faster by 10x (1ms to 10ms), but it's less consistent
        //if(best_posevals[1]/p3d.size()>RANSAC_PERC_DC || iters-last_save_iter>=RANSAC_IMPROV_ITERS) break;
    }
    
    if(debug) {
        printf("ransac iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int)iters, (int)best_posevals[0], best_posevals[2], best_posevals[3], (int)best_posevals[1], (int)p3d.size());
    }
    
    //have to reset the inliers.
    std::vector<double> posevals = Maximization(best_pose, p3d, p2d1, inliers, err);
    
    p1guess = best_pose;
    best_posevals[0] = iters;
    return best_posevals;
}

std::vector<std::vector<double> > LocalizePose::RobustDualBA(std::vector<double> p0, std::vector<double> p1,
                                                            std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1, std::vector<double>& rerrorp,
                                                            std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerrorb){
    gtsam::Pose3 gp0 = GTSAMInterface::VectorToPose(p0);
    gtsam::Pose3 gp1 = GTSAMInterface::VectorToPose(p1);
    gtsam::Pose3 p1frame0(gp1.rotation(), gp0.translation());
    struct timespec start, runir, end;
    
    if(p3d.size()<MIN_CORRESPONDENCES || b3d.size()<MIN_CORRESPONDENCES)  {
        std::cout << "Localization failed due to sizes: " <<p3d.size() << ", " << b3d.size() << std::endl;
        return {};
    }
    
    clock_gettime(CLOCK_MONOTONIC, &start);
    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p1frame0.
    std::vector<double> posevals1f0 = RANSAC_P3P(p1frame0, p3d, p2d1, rerrorp);
    
    if(posevals1f0[1]<0.000001){
        std::cout << "Localization failed due to RANSAC failure on set p"<<std::endl;
        return {};
    }
    
    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p0frame1.
    //gtsam::Pose3 p0frame1 = gp1.compose(p1frame0.between(gp0)); //this also looks wrong.
    gtsam::Pose3 p0frame1 = gp1.compose(gp1.between(p1frame0)*p1frame0.between(gp0)*p1frame0.between(gp1));
    std::vector<double> posevals0f1 = RANSAC_P3P(p0frame1, b3d, b2d0, rerrorb);
    
    clock_gettime(CLOCK_MONOTONIC, &runir);
    if(posevals0f1[1]<0.000001) {
        std::cout << "Localization failed due to RANSAC failure on set b"<<std::endl;
        return {};
    }

    //iterative localization (like EM), started with the good initial estimates found using RANSAC.
    std::vector<std::vector<double> > results = DualIterativeBA(gp0, gp1, p1frame0, p0frame1, p3d, p2d1, rerrorp, b3d, b2d0, rerrorb);
    clock_gettime(CLOCK_MONOTONIC, &end);
    std::string irtime = std::to_string((runir.tv_sec - start.tv_sec) + (runir.tv_nsec - start.tv_nsec)/1000000000.0);
    std::string altime = std::to_string((end.tv_sec - runir.tv_sec) + (end.tv_nsec - runir.tv_nsec)/1000000000.0);
    std::string tottime = std::to_string((end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec)/1000000000.0);
    if(debug) std::cout << "runtimes (s) "<<irtime << " + " << altime << " ~= " << tottime << std::endl;
    
    return results;
}

void
LocalizePose::testP3P()
{
    gtsam::Pose3 p;
    gtsam::Pose3 pT(gtsam::Rot3::Ypr(0.1, 0.15, 0.0), gtsam::Point3(0.0, 0.5, 0));
    
    std::vector<gtsam::Point3> pT3d;
    std::vector<gtsam::Point2> pT2d;
    gtsam::Cal3_S2::shared_ptr gtcam = _cam.GetGTSAMCam();
    gtsam::PinholeCamera<gtsam::Cal3_S2> pc(p, *gtcam);
    std::uniform_real_distribution<double> unifx(0,704);
    std::uniform_real_distribution<double> unify(0,480);
    std::uniform_real_distribution<double> dunif(10,30);
    std::default_random_engine re;
    for(int i=0; i<15; ++i)
    {
        double x = unifx(re);
        double y = unify(re);
        gtsam::Point2 p2d(x,y);
        double depth = dunif(re);
        
        gtsam::Point3 p3d = pc.backproject(p2d, depth);
        gtsam::Point3 p3d_tf = pT.transform_to(p3d);
        gtsam::Point2 onimage = _cam.ProjectToImage(p3d_tf);
        if(not _cam.InsideImage(onimage.x(), onimage.y()))
        {
            --i;
//            std::cout <<" " << p2d << " -> " << depth << ": "<<p3d << "; " << p3d_tf << " -> " << onimage << std::endl;
            continue;
        }
        pT3d.push_back(p3d);
        pT2d.push_back(onimage);
    }
    
    std::vector<double> inliers(15, 1.0);
    std::vector<double> correct = GTSAMInterface::PoseToVector(pT);
    std::vector<double> vec(6,0);
    std::uniform_real_distribution<double> uniferr(0,0.25);
    for(int i=0; i<6; ++i)
    {
        double err = uniferr(re);
        vec[i] = correct[i]+err-0.125;
    }
    
    debug = true;
    
    std::vector<std::vector<double>> res = UseBAIterative(vec, pT3d, pT2d, inliers);
    
    std::cout << "\n result: ";
    for(int i=0; i<6; ++i) std::cout << res[0][i] << ", ";
    std::cout << "\n correct: ";
    for(int i=0; i<6; ++i) std::cout << correct[i] << ", ";
    std::cout << "\n estimate: ";
    for(int i=0; i<6; ++i) std::cout << vec[i] << ", ";
}

void LocalizePose::testP3PStatic()
{
    std::vector<gtsam::Point3> p3d;
    p3d.push_back(gtsam::Point3(9.4018, -1.5499,   23.0600));
    p3d.push_back(gtsam::Point3(-2.5338,    3.2552,   21.2078));
    p3d.push_back(gtsam::Point3(7.8374,   -0.2290,   29.9017));
    
    std::vector<gtsam::Vector3> fv;
    fv.push_back(gtsam::Vector3(0.2242,   -0.1192,    0.9672));
    fv.push_back(gtsam::Vector3(-0.2503,    0.1390,    0.9581));
    fv.push_back(gtsam::Vector3(0.1025,   -0.0488,    0.9935));
    
    std::vector<gtsam::Pose3> poses;
    P3P::computePoses(fv, p3d, poses);
}






















