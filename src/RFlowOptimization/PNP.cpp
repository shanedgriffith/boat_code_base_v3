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
, nlocalization_factors(0)
, cam_(cam)
, pguess_(pguess)
, p3d_subset_(p3d_subset)
, p2d_subset_(p2d_subset)
{
    //assume some sigmas for the pose.
    gtsam::Vector6 v6p;
    v6p = (gtsam::Vector(6) << 5.0, 5.0, 5.0, 0.5, 0.5, 0.5).finished();
    flexible_ = gtsam::noiseModel::Diagonal::Sigmas(v6p);
}

void
PNP::
setDebug()
{
    debug_ = true;
}

void
PNP::
setInliers(std::shared_ptr<std::vector<double>> inliers)
{
    explicit_filter_ = inliers->size() > 0;
    inliers_ = inliers;
}

void
PNP::
setNoiseModel(double acceptable_rerror, PNP::NM noise_model)
{
    gtsam::noiseModel::Base::shared_ptr measurement_noise_outlier_free_ = gtsam::noiseModel::Isotropic::Sigma(2, acceptable_rerror);
    switch(noise_model)
    {
        case PNP::OUTLIER_FREE:
            measurement_noise_ = measurement_noise_outlier_free_;
            break;
        case PNP::HUBER:
            measurement_noise_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(acceptable_rerror), measurement_noise_outlier_free_);
            break;
        case PNP::GEMAN_MCCLURE:
            measurement_noise_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(acceptable_rerror), measurement_noise_outlier_free_); //acceptable_rerror
            break;
        default:
            throw std::runtime_error("PNP::setNoiseModel() error. unknown loss.");
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
addLocalizationFactors(gtsam::Symbol symb)
{
    gtsam::Cal3_S2::shared_ptr gt_camera = cam_.GetGTSAMCam();

    for(int i=0; i<p2d_subset_.size(); ++i)
    {
        if(explicit_filter_ and (*inliers_)[i] < 0.0) //unnecessary if using GM. TODO: test to see which parameter values for GM + Huber produce the best results.
        {
            continue;
        }
        ++nlocalization_factors;
        graph_.add(LocalizationFactor<gtsam::Pose3, gtsam::Cal3_S2>(p2d_subset_[i], p3d_subset_[i], measurement_noise_, symb, gt_camera));
    }
    
//    int c2 = 0;
//    for(size_t i=0; i<p2d_subset_.size(); ++i)
//    {
//        if(explicit_filter_ and (*inliers_)[i] == 0) ++c2;
//    }
//    
//    std::cout << "localization with " << nlocalization_factors << " of " << p2d_subset_.size() << ". unset inliers: " << c2 << std::endl;
}

gtsam::Symbol
PNP::
constructGraph()
{
    gtsam::Symbol symb('x', 0);
    addPose(symb);
    addLocalizationFactors(symb);
    return symb;
}

std::tuple<bool, gtsam::Values>
PNP::
optimize()
{
    gtsam::Values result;
    bool suc = 0;
    
    if(debug_)
    {
        std::cout << "-------------BundleAdjustment-------------" << std::endl;
        graph_.print();
        initial_estimate_.print();
    }
    
    if(nlocalization_factors == 0)
    {
        if(debug_)
        {
            std::cout<<"PNP::RunBA() error: no localization factors. \n " << std::endl;
        }
        return std::make_tuple(suc, result);
    }
    
    try
    {
        gtsam::DoglegOptimizer optimizer(graph_, initial_estimate_);
        double initial_error = optimizer.error();
        result = optimizer.optimize();
        double result_error = optimizer.error();
        suc = result_error < initial_error;
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
    return std::make_tuple(suc, result);
}

std::tuple<bool, gtsam::Pose3>
PNP::
run()
{
    gtsam::Symbol symb = constructGraph();
    gtsam::Values result;
    bool suc;
    std::tie(suc, result) = optimize();

    if(result.size()==0)
    {
        return std::make_tuple(false, gtsam::Pose3::identity());
    }
    else
    {
        return std::make_tuple(suc, result.at<gtsam::Pose3>(symb));
    }
}
