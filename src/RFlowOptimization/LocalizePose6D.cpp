#include "LocalizePose6D.h"

#include "P3P.h"
#include "PNP.h"
#include "Optimization/MultiSession/LocalizationFactor.h"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>

LocalizePose6D::
LocalizePose6D(const Camera& cam, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d)
: Localization(cam)
, p3d_(p3d)
, p2d_(p2d)
, inliers_(p3d.size(), 0.0)
{
    //TODO.
    if(p3d_.size() < MIN_CORRESPONDENCES)
    {
        throw std::runtime_error("LocalizePose6D::LocalizePose6D() Error. Need at least " + std::to_string(MIN_CORRESPONDENCES) + " correspondences");
    }
}

void
LocalizePose6D::
setInitialEstimate(const gtsam::Pose3& guess)
{
    pguess_ = guess;
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
        gtsam::Point3 tfp = pguess_.transform_to(p3d_[i]);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        if(!cam_.InsideImage(res)) continue;
        double dist = res.distance(p2d_[i]);
        sumall+=dist;
        count++;
        if(inliers_[i])
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
Maximization()
{
    int nchanges=0;
    int ninliers = 0;
    double sumall=0;
    double sumin=0;
    for(int i=0; i<p3d_.size(); i++){
        gtsam::Point3 tfp = pguess_.transform_to(p3d_[i]);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        double dist = res.distance(p2d_[i]);
        if(dist > ACCEPTABLE_TRI_RERROR || !cam_.InsideImage(res))
        {
            if(inliers_[i]>=0.0) nchanges++;
            inliers_[i] = -1;
            if(!cam_.InsideImage(res)) continue;
        }
        else
        {
            if(inliers_[i]<0) nchanges++;
            inliers_[i] = ACCEPTABLE_TRI_RERROR;//dist; //I seem to get more reliable estimates using err, rather than dist, here.
            sumin+=dist;
            ninliers++;
        }
        sumall+=dist;
    }
    return {(double)nchanges, (double)ninliers, sumall/p3d_.size(), sumin/ninliers};
}

std::tuple<gtsam::Pose3, std::vector<double>>
LocalizePose6D::
RANSAC()
{
    //EM approach to finding the best pose.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
    gtsam::Pose3 best_pose = pguess_;
    const int SAMPLE_SIZE = 4;
    std::vector<gtsam::Point3> subp3d(SAMPLE_SIZE);
    std::vector<gtsam::Point2> subp2d1(SAMPLE_SIZE);
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
        
        bool success;
        std::tie(success, pguess_) = runMethod(pguess_, subp3d, subp2d1);
        if(not success) continue;
        
        std::vector<double> posevals = Maximization();
        if(posevals[1]>best_posevals[1] or
           (posevals[1]==best_posevals[1] and posevals[2] < best_posevals[2]))
        {
            swap(best_posevals, posevals);
            best_pose = pguess_;
            last_save_iter = iters;
        }
        
        int n_total_iters = ceil(NumRequiredRANSACIterations(best_posevals[1], p3d_.size(), SAMPLE_SIZE, 0.99));
        
        if(n_total_iters < 0) continue;
        
        n_iters = std::min(n_iters, n_total_iters);
        
        //makes RANSAC faster by 10x (1ms to 10ms), but it's less consistent
        //if(best_posevals[1]/p3d.size()>RANSAC_PERC_DC || iters-last_save_iter>=RANSAC_IMPROV_ITERS) break;
    }
    
    if(debug_)
    {
        printf("ransac iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int)iters, (int)best_posevals[0], best_posevals[2], best_posevals[3], (int)best_posevals[1], (int)p3d_.size());
    }
    
    //have to reset the inliers.
    std::vector<double> posevals = Maximization(best_pose, p3d_, p2d_, inliers_, err);
    
    best_posevals[0] = iters;
    return std::make_tuple(best_pose, best_posevals);
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
    
    //first RANSAC to find the best estimate of p1frame0.
    std::vector<double> posevalsransac;
    std::tie(pguess_, posevalsransac) = RANSAC();
    
    if(posevalsransac[1]<0.000001) return {};
    
    //measure rerror with the previous set of inliers, if it's good, update the set of inliers.
    int iters = 0;
    int nchanges=1;
    double err = ACCEPTABLE_TRI_RERROR;
    for(int i=0; nchanges > 0 and i < MAX_ITERS; i++)
    {
        PNP localizer(cam_, pguess_, p3d_, p2d_, inliers_);
        bool success;
        std::tie(success, pguess_) = localizer.run();
        if(not success) continue;
        
        std::vector<double> posevals = Maximization();
        
        if(iters==0 or posevals[1]>best_score) //using the reprojection error of the inlier set, rather than the number of inliers.
        {
            best_score = posevals[1];
            best_pose = pguess_;
            best_posevals = posevals;
            minpiter = iters;
        }
        iters++;
        nchanges = posevals[0];
        
        if(debug_)
        {
            printf("bai iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
                   (int) iters, (int) posevals[0], posevals[2], posevals[3], (int) posevals[1], (int) p3d_.size());
        }
        if(robust_loss_ and iters > 1)
            break;
    }
    
    pguess_ = best_pose;
    if(minpiter != iters-1) Maximization(); //reset the inliers.
    if(best_posevals.size()==0) return std::make_tuple(best_pose, {});
    best_posevals[0] = iters;
    return std::make_tuple(best_pose, best_posevals);
}


