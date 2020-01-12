#pragma once


#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include "DataTypes/Camera.hpp"

class PNP
{
public:
    
    enum NM{OUTLIER_FREE=0, HUBER, GEMAN_MCCLURE};
    
protected:
    
    gtsam::noiseModel::Base::shared_ptr flexible_;
    gtsam::noiseModel::Base::shared_ptr measurement_noise_;
    
    void
    addPose(gtsam::Symbol symb);
    
    void
    addLocalizationFactors(gtsam::Symbol symb);
    
    gtsam::Symbol
    constructGraph();
    
    std::tuple<bool, gtsam::Values>
    optimize();
    
    bool debug_;
    bool explicit_filter_;
    size_t nlocalization_factors;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    bool optimization_succeeded_;
    
    const Camera& cam_;
    const gtsam::Pose3& pguess_;
    const std::vector<gtsam::Point3>& p3d_subset_;
    const std::vector<gtsam::Point2>& p2d_subset_;
    std::shared_ptr<std::vector<double>> inliers_;
    
public:
    
    
    PNP(const Camera& cam, const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset);
    
    void
    setNoiseModel(double acceptable_rerror, PNP::NM noise_model);
    
    void
    setInliers(std::shared_ptr<std::vector<double>> inliers);
    
    std::tuple<bool, gtsam::Pose3>
    run();
    
    void
    setDebug();
};
