#pragma once

#include <vector>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "DataTypes/Camera.hpp"


class Localization
{
public:
    
    enum METHOD {P3P=0, PNP}; //class template better?
    
protected:
    const int MAX_RANSAC_ITERS = 100;
    double ACCEPTABLE_TRI_RERROR = 6.0;
    int MAX_ITERS = 15;
    int RANSAC_IMPROV_ITERS = 100;
    double RANSAC_PERC_DC = 0.5;
    int MIN_CORRESPONDENCES = 4;
    
    double
    NumRequiredRANSACIterations(size_t ninliers, size_t setsize, size_t nsamples_per_iteration, double probability_all_inliers);
    
    void
    GenerateRandomSet(int n, std::vector<int>& rset);
    
    std::vector<size_t>
    GenerateRandomSet(int n, int k);
    
    virtual std::vector<double>
    Maximization() = 0; //const gtsam::Pose3& gtp, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers,
    
    std::tuple<bool, gtsam::Pose3>
    runMethod(const gtsam::Pose3& guess, const std::vector<gtsam::Point3>& subp3d, const std::vector<gtsam::Point2>& subp2d1);
    
    bool debug_;
    Localization::METHOD ransac_method_;
    bool robust_loss_;
    
    const Camera& cam_;
    
public:
    
    Localization(const Camera& cam);
    
    void setRobustLoss();
    
    void setRANSACMethod(Localization::METHOD method);
    
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
