#pragma once


#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "DataTypes/Camera.hpp"

class PNP
{
protected:
    
    gtsam::noiseModel::Base::shared_ptr flexible_;
    gtsam::noiseModel::Base::shared_ptr measurement_noise_;
    
    void
    AddPose(gtsam::Symbol symb, const gtsam::Pose3& pguess);
    
    void
    AddLocalizationFactors(gtsam::Symbol symb, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset, const std::vector<double>& inliers);
    
    gtsam::Values RunBA();
    
    bool debug_;
    bool explicit_filter_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    const Camera& cam_;
    
public:
    
    enum NM{OUTLIER_FREE=0, HUBER, GEMAN_MCCLURE};
    
    PNP(const Camera& cam, double acceptable_rerror, PNP::NM noise_model);
    
    gtsam::Pose3
    UseBA(const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset, const std::vector<double>& inliers);
    
    void
    setExplicitFilter();
    
    bool
    EmptyPose(const gtsam::Pose3& p);
    
};
