#pragma once

#include "Localization.h"

//#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/EssentialMatrix.h>

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/NoiseModel.h>


class LocalizePose5D: public Localization
{
public:
    
    enum METHOD {_NISTER=0, _PNP};
    
protected:
    const int SAMPLE_SIZE = 8;
    double ACCEPTABLE_TRI_RERROR = 0.002;//0.05;
    
    size_t
    setSize();
    
    size_t
    sampleSize();
    
    void
    updateSubsets(const std::vector<size_t>& rset = {});
    
    void
    updateOptimizationMethod();
    
    double
    MeasureReprojectionError();
    
    std::vector<double>
    Maximization();
    
    void
    updateResult();
    
    void
    updateGuess();
    
    bool
    runMethod(bool use_robust_loss, bool use_inliers);
    
    
    gtsam::EssentialMatrix pguess_;
    gtsam::EssentialMatrix best_guess_;
    const std::vector<gtsam::Point2>& p2d0_;
    const std::vector<gtsam::Point2>& p2d1_;
    std::shared_ptr<std::vector<double>> inliers_;
    LocalizePose5D::METHOD ransac_method_;
    std::vector<gtsam::Point2> p2d0_subset_;
    std::vector<gtsam::Point2> p2d1_subset_;
    
    
public:
    
    LocalizePose5D(const Camera& cam, const std::vector<gtsam::Point2>& p2d0, const std::vector<gtsam::Point2>& p2d1);
    
    std::tuple<bool, gtsam::EssentialMatrix, std::vector<double>>
    UseRANSAC();
    
    std::tuple<bool, gtsam::EssentialMatrix, std::vector<double>>
    UseBAIterative();
    
    std::shared_ptr<std::vector<double>>
    getInliers();
    
    void
    setInitialEstimate(const gtsam::EssentialMatrix& guess);
    
    void
    setRANSACMethod(LocalizePose5D::METHOD method);
    
    void
    setErrorThreshold(double e);
};
