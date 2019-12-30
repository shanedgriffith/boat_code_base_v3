#include "LocalizePose6D.h"

#include "P3P.hpp"
#include "Optimization/MultiSession/LocalizationFactor.h"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>

LocalizePose6D::
LocalizePose6D(const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d)
: p3d_(p3d)
, p2d_(p2d)
, inliers_(p3d.size(), 0.0)
{
    std::vector<double> flexible_sigmas = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5};
    gtsam::Vector6 v6p = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&flexible_sigmas), 6, 1);
    flexible_ = gtsam::noiseModel::Diagonal::Sigmas(v6p);
    
    measurement_noise_outlier_free_ = gtsam::noiseModel::Isotropic::Sigma(2, 3.0);
    measurement_noise_robust_iter_0_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(ACCEPTABLE_TRI_RERROR), measurement_noise_outlier_free_);
    measurement_noise_robust_iter_1ton_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(ACCEPTABLE_TRI_RERROR), measurement_noise_outlier_free_);
    
    if(p3d_.size() < MIN_CORRESPONDENCES)
    {
        throw std::runtime_error("LocalizePose6D::LocalizePose6D() Error. Need at least " + std::to_string(MIN_CORRESPONDENCES) + " correspondences");
    }
}

void
LocalizePose6D::
AddPose(gtsam::Symbol symb, const gtsam::Pose3& pguess)
{
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(symb, pguess, flexible_));
    initial_estimate_.insert(symb, pguess);
}

gtsam::Pose3
LocalizePose6D::
UseBA(const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset, const std::vector<double>& inliers, int iter)
{
    gtsam::Symbol symb('x', 0);
    AddPose(symb, pguess);
    AddLocalizationFactors(symb, p3d_subset, p2d_subset, inliers, iter);
    gtsam::Values result = RunBA();
    if(result.size()==0) return gtsam::Pose3::identity();
    else return result.at<gtsam::Pose3>(symb);
}

void
LocalizePose6D::
AddLocalizationFactors(gtsam::Symbol symb, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset, const std::vector<double>& inliers, int iter)
{
    gtsam::Cal3_S2::shared_ptr gt_camera = cam_.GetGTSAMCam();
    gtsam::noiseModel::Base::shared_ptr measurement_noise = measurement_noise_outlier_free_;
    if(robust_loss_)
    {
        if(iter==0) measurement_noise = measurement_noise_robust_iter_0_;
        else measurement_noise = measurement_noise_robust_iter_1ton_;
    }
    
    for(int i=0; i<p2d_subset.size(); ++i)
    {
        if(not robust_loss_ and inliers[i]<0.0)
        {
            continue;
        }
        
        graph_.add(LocalizationFactor<gtsam::Pose3, gtsam::Cal3_S2>(p2d_subset[i], p3d_subset[i], measurement_noise, symb, gt_camera));
    }
}

bool
LocalizePose6D::
EmptyPose(const gtsam::Pose3& p)
{
    gtsam::Pose3 zeros = gtsam::Pose3::identity();
    return zeros.equals(p);
}

double
LocalizePose6D::
MeasureReprojectionError() //const gtsam::Pose3& gtp, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d, const std::vector<unsigned char>& inliers
{
    double sumall=0;
    double sumin=0;
    int cin=0;
    
    int count=0;
    for(int i=0; i<p3d_.size(); i++)
    {
        gtsam::Point3 tfp = gtp.transform_to(p3d_[i]);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        if(!cam_.InsideImage(res)) continue;
        double dist = res.distance(p2d_[i]);
        sumall+=dist;
        count++;
        if(inliers[i])
        {
            sumin+=dist;
            cin++;
        }
    }
    if(debug_)
    {
        std::cout<<"num points: " << count << ", reprojection error: "<<sumall/count<<", inliers only: "<<sumin/cin<< ", "<< cin << " of " << count << " are inliers." << std::endl;
    }
    
    return sumall/count;
}

std::vector<double>
LocalizePose6D::
Maximization(double err) //const gtsam::Pose3& gtp, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers,
{
    int nchanges=0;
    int ninliers = 0;
    double sumall=0;
    double sumin=0;
    for(int i=0; i<p3d_.size(); i++){
        gtsam::Point3 tfp = gtp.transform_to(p3d_[i]);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        double dist = res.distance(p2d_[i]);
        if(dist > err || !cam_.InsideImage(res))
        {
            if(inliers_[i]>=0.0) nchanges++;
            inliers_[i] = -1;
            if(!cam_.InsideImage(res)) continue;
        }
        else
        {
            if(inliers_[i]<0) nchanges++;
            inliers_[i] = err;//dist; //I seem to get more reliable estimates using err, rather than dist, here.
            sumin+=dist;
            ninliers++;
        }
        sumall+=dist;
    }
    return {(double)nchanges, (double)ninliers, sumall/p3d_.size(), sumin/ninliers};
}

std::tuple<gtsam::Pose3, std::vector<double>>
LocalizePose6D::
RANSAC_BA() // const gtsam::Pose3& p1guess, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers
{
    //EM approach to finding the best pose.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
    gtsam::Pose3 best_pose = p1guess;
    const int SAMPLE_SIZE = 4;
    std::vector<gtsam::Point3> subp3d(SAMPLE_SIZE);
    std::vector<gtsam::Point2> subp2d1(SAMPLE_SIZE);
    std::vector<double> subinliers(SAMPLE_SIZE, ACCEPTABLE_TRI_RERROR);
    std::vector<double> best_posevals(4, 0.0);
    int iters=0;
    int last_save_iter = 0;
    double err = ACCEPTABLE_TRI_RERROR;
    
    int n_iters = MAX_RANSAC_ITERS;
    
    for(; iters<n_iters; iters++)
    {
        std::vector<size_t> rset = GenerateRandomSet(p3d_.size(), SAMPLE_SIZE);
        for(int j=0; j<rset.size(); j++){
            subp3d[j] = p3d_[rset[j]];
            subp2d1[j] = p2d_[rset[j]];
        }
        gtsam::Pose3 estp = UseBA(best_pose, subp3d, subp2d1, subinliers);
        
        if(EmptyPose(estp)) continue;
        std::vector<double> posevals = Maximization(estp, p3d, p2d1, inliers, err);
        if(posevals[1]>best_posevals[1] or
           (posevals[1]==best_posevals[1] and posevals[2] < best_posevals[2]))
        {
            swap(best_posevals, posevals);
            best_pose = estp;
            last_save_iter = iters;
        }
        
        int n_total_iters = ceil(NumRequiredRANSACIterations(best_posevals[1], p3d.size(), SAMPLE_SIZE, 0.99));
        
        if(n_total_iters < 0) continue;
        
        n_iters = std::min(n_iters, n_total_iters);
        
        //makes RANSAC faster by 10x (1ms to 10ms), but it's less consistent
        //if(best_posevals[1]/p3d.size()>RANSAC_PERC_DC || iters-last_save_iter>=RANSAC_IMPROV_ITERS) break;
    }
    
    if(debug)
    {
        printf("ransac iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int)iters, (int)best_posevals[0], best_posevals[2], best_posevals[3], (int)best_posevals[1], (int)p3d.size());
    }
    
    //have to reset the inliers.
    std::vector<double> posevals = Maximization(best_pose, p3d, p2d1, inliers, err);
    
    best_posevals[0] = iters;
    return std::make_tuple(best_pose, best_posevals);
}

std::tuple<gtsam::Pose3, std::vector<double>>
LocalizePose6D::
RANSAC_P3P() // const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers
{
    //EM approach to finding the best pose.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
    gtsam::Pose3 best_pose;
//    int kdisambiguation = 1; // pick k random points to use for disambiguation, which are different from the set used to compute the localiation
//    std::vector<int> rset(3+kdisambiguation);
    int SAMPLE_SIZE = 4;
    std::vector<gtsam::Point3> subp3d(SAMPLE_SIZE);
    std::vector<gtsam::Point2> subp2d1(SAMPLE_SIZE);
    std::vector<double> best_posevals(4, 0.0);
    int iters=0;
    int last_save_iter = 0;
    double err = ACCEPTABLE_TRI_RERROR;
    int n_iters = MAX_RANSAC_ITERS;
    
    for(; iters<n_iters; ++iters)
    {
//        GenerateRandomSet(rset.size(), rset);
        std::vector<size_t> rset = GenerateRandomSet(p3d_.size(), SAMPLE_SIZE);
        for(int j=0; j<SAMPLE_SIZE; j++)
        {
            subp3d[j] = p3d_[rset[j]];
            subp2d1[j] = p2d_[rset[j]];
        }
        
        gtsam::Pose3 estp = RunP3P(subp3d, subp2d1);
        
        std::vector<double> posevals = Maximization(estp, p3d, p2d1, inliers, err);
        if(posevals[1]>best_posevals[1] or
           (posevals[1]==best_posevals[1] and posevals[2] < best_posevals[2]))
        {
            swap(best_posevals, posevals);
            best_pose = estp;
            last_save_iter = iters;
        }
        
        int n_total_iters = ceil(NumRequiredRANSACIterations(best_posevals[1], p3d.size(), 3, 0.99));//the fourth sample needs to be an inlier as well..., but if it's slightly off, the the correct result can still be selected.
        
        if(n_total_iters < 0) continue;
        
        n_iters = std::min(n_iters, n_total_iters);
        //        std::cout << "computed: " << n_total_iters << " iterations from " << posevals[1] << " inliers " << std::endl;
        //makes RANSAC faster by 10x (1ms to 10ms), but it's less consistent
        //if(best_posevals[1]/p3d.size()>RANSAC_PERC_DC || iters-last_save_iter>=RANSAC_IMPROV_ITERS) break;
    }
    
    if(debug)
    {
        printf("ransac iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int)iters, (int)best_posevals[0], best_posevals[2], best_posevals[3], (int)best_posevals[1], (int)p3d.size());
    }
    
    //have to reset the inliers.
    std::vector<double> posevals = Maximization(best_pose, p3d, p2d1, inliers, err);
    
    best_posevals[0] = iters;
    return std::make_tuple(best_pose, best_posevals);
}

gtsam::Pose3
LocalizePose6D::
disambiguatePoses(const std::vector<gtsam::Pose3>& poses, const gtsam::Point3& point3d, const gtsam::Point2& point2d)
{
    int bestpidx = -1;
    double bestdist = std::numeric_limits<double>::max();
    for(int i=0; i<poses.size(); ++i)
    {
        gtsam::Point3 tfp = poses[i].transform_to(point3d);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        double dist = res.distance(point2d);
        
        if(dist < bestdist)
        {
            bestdist = sumd;
            bestpidx = i;
        }
    }
    
    return poses[bestpidx];
}

gtsam::Pose3
LocalizePose6D::
RunP3P(const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset)
{
    /*should have 4 points. the first three are used for the estimation. the fourth for disambiguation.*/
    std::vector<gtsam::Vector3> feature_vectors(p3d_subset.size());
    static gtsam::PinholeCamera<gtsam::Cal3_S2> pc(gtsam::Pose3::identity(), *(cam_.GetGTSAMCam()));
    for(int j=0; j<p3d_subset.size(); j++)
    {
        gtsam::Unit3 uv = pc.backprojectPointAtInfinity(p2d_subset[j]);
        feature_vectors[j] = uv.unitVector();
    }
    
    if(not P3P::suitableSet(p3d_subset))
        return gtsam::Pose3::identity();
    
    std::vector<gtsam::Pose3> poses = P3P::computePoses(feature_vectors, p3d_subset);
    
    //NOTE: only the fourth point is used for disambiguation
    gtsam::Pose3 res = disambiguatePoses(poses, p3d_subset[3], p2d_subset[3]);
    
    return res;
}

std::tuple<gtsam::Pose3, std::vector<double>>
LocalizePose6D::
UseBAIterative()
{
    //TODO: make this function consistent with the DualIterativeBA(). (and update RANSAC_BA() as well)
    //EM approach to finding the best pose.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
    
    
    gtsam::Pose3 best_pose;
    std::vector<double> best_posevals;
    double best_score;
    int minpiter = -1;
    gtsam::Pose3 estp;
    
    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p1frame0.
    std::vector<double> posevalsransac;
    switch(RANSAC_MODEL)
    {
        case 0:
            std::tie(estp, posevalsransac) = RANSAC_P3P(p3d, p2d, inliers);
            break;
        case 1:
            std::tie(estp, posevalsransac) = RANSAC_BA(pguess, p3d, p2d, inliers);
            break;
        default:
            std::cout << "set the ransac model" << std::endl;
            exit(-1);
    }
    if(posevalsransac[1]<0.000001) return {};
    
    //measure rerror with the previous set of inliers, if it's good, update the set of inliers.
    int iters = 0;
    int nchanges=1;
    double err = ACCEPTABLE_TRI_RERROR;
    //while(err>6.0 ||(nchanges > 0 && iters < MAX_ITERS)){
    for(int i=0; nchanges > 0 and i < MAX_ITERS; i++)
    {
        estp = UseBA(estp, p3d, p2d, inliers, i);
        if(EmptyPose(estp)) break;
        std::vector<double> posevals = Maximization(estp, p3d, p2d, inliers, err);
        
        if(iters==0 or posevals[1]>best_score) //using the reprojection error of the inlier set, rather than the number of inliers.
        {
            best_score = posevals[1];
            best_pose = estp;
            best_posevals = posevals;
            minpiter = iters;
        }
        iters++;
        nchanges = posevals[0];
        
        if(debug)
        {
            printf("bai iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
                   (int)iters, (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)p3d.size());
        }
        if(robust_loss_ and iters > 1)
            break;
    }
    if(minpiter != iters-1) Maximization(best_pose, p3d, p2d, inliers, err); //reset the inliers.
    if(best_posevals.size()==0)return {};
    best_posevals[0] = iters;
    return std::make_tuple(best_pose, best_posevals);
}


