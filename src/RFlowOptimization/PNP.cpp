#include "PNP.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

PNP::
PNP(const Camera& cam, double acceptable_rerror, PNP::NM noise_model)
: debug_(false)
, explicit_filter_(false)
, cam_(cam)
{
    std::vector<double> flexible_sigmas = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5};
    gtsam::Vector6 v6p = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&flexible_sigmas), 6, 1);
    flexible_ = gtsam::noiseModel::Diagonal::Sigmas(v6p);
    
    gtsam::noiseModel::Base::shared_ptr measurement_noise_outlier_free_ = gtsam::noiseModel::Isotropic::Sigma(2, acceptable_rerror/2.0);
    switch(noise_model)
    {
        case PNP::OUTLIER_FREE:
            measurement_noise_ = measurement_noise_outlier_free_;
            break;
        case PNP::HUBER:
            measurement_noise_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(acceptable_rerror), measurement_noise_outlier_free_);
            break;
        case PNP::GEMAN_MCCLURE:
        default:
            measurement_noise_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(acceptable_rerror), measurement_noise_outlier_free_);
            break;
    }
}

void
PNP::
setExplicitFilter()
{
    explicit_filter_ = true;
}

void
PNP::
AddPose(gtsam::Symbol symb, const gtsam::Pose3& pguess)
{
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(symb, pguess, flexible_));
    initial_estimate_.insert(symb, pguess);
}

void
PNP::
AddLocalizationFactors(gtsam::Symbol symb, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset, const std::vector<double>& inliers)
{
    gtsam::Cal3_S2::shared_ptr gt_camera = cam_.GetGTSAMCam();

    for(int i=0; i<p2d_subset.size(); ++i)
    {
        if(explicit_filter_ and inliers[i]<0.0) //unnecessary if using GM. TODO: test to see which parameter values for GM + Huber produce the best results.
        {
            continue;
        }
        
        graph_.add(LocalizationFactor<gtsam::Pose3, gtsam::Cal3_S2>(p2d_subset[i], p3d_subset[i], measurement_noise_, symb, gt_camera));
    }
}

gtsam::Values
PNP::
RunBA()
{
    gtsam::Values result;
    try
    {
        result = gtsam::DoglegOptimizer(graph_, initial_estimate_).optimize();
    }
    catch(const std::exception& ex)
    {
        if(debug_)
        {
            std::cout<<"PNP::RunBA() error: optimization failed. \n " << std::endl;
            ex.what();
        }
    }
    graph_.resize(0);
    initial_estimate_.clear();
    return result;
}

gtsam::Pose3
PNP::
UseBA(const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset, const std::vector<double>& inliers)
{
    gtsam::Symbol symb('x', 0);
    AddPose(symb, pguess);
    AddLocalizationFactors(symb, p3d_subset, p2d_subset, inliers);
    gtsam::Values result = RunBA();
    if(result.size()==0) return gtsam::Pose3::identity();
    else return result.at<gtsam::Pose3>(symb);
}
