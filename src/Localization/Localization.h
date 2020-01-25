#pragma once

#include <vector>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "DataTypes/Camera.hpp"

//this class is simply a RANSAC interface. anything that implements it does RANSAC.
class Localization
{
protected:
    const int MAX_OPTIMIZATION_ITERS = 15;
    const int MAX_RANSAC_ITERS = 100;
    
    double
    NumRequiredRANSACIterations(size_t ninliers, size_t setsize, size_t nsamples_per_iteration, double probability_all_inliers);
    
    void
    GenerateRandomSet(int n, std::vector<int>& rset);
    
    std::vector<size_t>
    GenerateRandomSet(size_t n, size_t k);
    
    virtual void
    updateSubsets(const std::vector<size_t>& rset = {}) = 0;
    
    virtual std::vector<double>
    Maximization() = 0;
    
    virtual void
    updateResult() = 0;
    
    virtual void
    updateOptimizationMethod() = 0;
    
    virtual bool
    runMethod(bool use_robust_loss, bool use_inliers) = 0;
    
    virtual void
    updateGuess() = 0;
    
    std::vector<double>
    RANSAC();
    
    std::vector<double>
    iterativeBA();
    
    virtual size_t
    setSize() = 0;
    
    virtual size_t
    sampleSize() = 0;
    
    bool debug_;
    bool robust_loss_;
    size_t iter;
    
    const Camera& cam_;
    
public:
    
    Localization(const Camera& cam);
    
    void setRobustLoss();
    
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
