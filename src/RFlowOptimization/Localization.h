#pragma once

#include <vector>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "DataTypes/Camera.hpp"


class Localization
{
protected:
    const int MAX_RANSAC_ITERS = 100;
    double ACCEPTABLE_TRI_RERROR = 6.0;
    int MAX_ITERS = 15;
    int RANSAC_IMPROV_ITERS = 100;
    double RANSAC_PERC_DC = 0.5;
    int MIN_CORRESPONDENCES = 4;
    
    double
    NumRequiredRANSACIterations(int ninliers, int setsize, int nsamples_per_iteration, double probability_all_inliers);
    
    void
    GenerateRandomSet(int n, std::vector<int>& rset);
    
    std::vector<size_t>
    GenerateRandomSet(int n, int k);
    
    gtsam::Values RunBA();
    
    std::vector<double>
    Maximization(double err) = 0; //const gtsam::Pose3& gtp, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers,
    
    
    bool debug_ = false;
    bool robust_loss_ = false;
    int ransac_model_ = 0;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    const Camera& cam_;
    
public:
    
    Localization(const Camera& cam);
    
    void setRobustLoss();
    
    void setRANSACModel(int model);
    
    void setErrorThreshold(double e);
    
    void setDebug();
};



/*
Localization (abstract)
 >RunBA.
 >

LocalizePose6D
 >gtsam::Pose3& estimate
 >gtsam::Pose3& best_estimate
 >p3d
 >p2d1
 >>IterativeBA

LocalizePose6DDual: LocalizePose6D
 >b3d
 >p2d0
 //OR, implement two different LocalizePose6D, and extend Localization
>robustdualBA
 
 
LocalizePose5D
 >gtsam::EM& estimate;
 >gtsam::EM& best_estimate;
 >p2d0, p2d1
 
 
 
 */
