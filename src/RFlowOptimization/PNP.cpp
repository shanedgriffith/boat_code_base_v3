#include "PNP.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

#include "Optimization/MultiSession/LocalizationFactor.h"

PNP::
PNP(const Camera& cam, const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset)
: debug_(false)
, explicit_filter_(false)
, cam_(cam)
, pguess_(pguess)
, p3d_subset_(p3d_subset)
, p2d_subset_(p2d_subset)
, inliers_({})
{
    //assume some sigmas for the pose.
    std::vector<double> flexible_sigmas = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5};
    gtsam::Vector6 v6p = Eigen::Map<Eigen::Matrix<double, 6, 1> >((double*)(&flexible_sigmas), 6, 1);
    flexible_ = gtsam::noiseModel::Diagonal::Sigmas(v6p);
    
    // default acceptable error of 6 pixels.
    measurement_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 6.0/2.0);
}

PNP::
PNP(const Camera& cam, const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset, const std::vector<double>& inliers)
: PNP(cam, pguess, p3d_subset, p2d_subset)
, explicit_filter_(true)
, inliers_(inliers)
{}

void
PNP::
setNoiseModel(double acceptable_rerror, PNP::NM noise_model)
{
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
addPose(gtsam::Symbol symb)
{
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(symb, pguess_, flexible_));
    initial_estimate_.insert(symb, pguess_);
}

void
PNP::
addLocalizationFactors(gtsam::Symbol symb, )
{
    gtsam::Cal3_S2::shared_ptr gt_camera = cam_.GetGTSAMCam();

    for(int i=0; i<p2d_subset_.size(); ++i)
    {
        if(explicit_filter_ and inliers[i]<0.0) //unnecessary if using GM. TODO: test to see which parameter values for GM + Huber produce the best results.
        {
            continue;
        }
        
        graph_.add(LocalizationFactor<gtsam::Pose3, gtsam::Cal3_S2>(p2d_subset[i], p3d_subset[i], measurement_noise_, symb, gt_camera));
    }
}

gtsam::Symbol
PNP::
constructGraph()
{
    gtsam::Symbol symb('x', 0);
    AddPose(symb, pguess);
    AddLocalizationFactors(symb, p3d_subset, p2d_subset, inliers);
    return symb;
}

gtsam::Values
PNP::
optimize()
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

std::tuple<bool, gtsam::Pose3>
PNP::
run()
{
    gtsam::Symbol symb = constructGraph();
    gtsam::Values result = optimize();

    if(result.size()==0)
    {
        return std::make_tuple(false, gtsam::Pose3::identity());
    }
    else
    {
        return std::make_tuple(true, result.at<gtsam::Pose3>(symb));
    }
}
