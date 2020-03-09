#include "PNP.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/slam/EssentialMatrixFactor.h>

#include "LocalizationFactor.h"

template <class T, class P>
PNP<T,P>::
PNP(const Camera& cam, const T& pguess, const std::vector<P>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset)
: debug_(false)
, explicit_filter_(false)
, nlocalization_factors(0)
, cam_(cam)
, pguess_(pguess)
, p3d_subset_(p3d_subset)
, p2d_subset_(p2d_subset)
{
    initializeCorrespondenceNoiseModel();
}

template <>
void
PNP<gtsam::Pose3, gtsam::Point3>::
initializeCorrespondenceNoiseModel()
{
    //assume some sigmas for the pose.
    gtsam::Vector6 v6p;
    v6p = (gtsam::Vector(6) << 5.0, 5.0, 5.0, 0.5, 0.5, 0.5).finished();
//    v6p = (gtsam::Vector(6) << 5.0, 5.0, 5.0, 0.5, 0.5, 0.5).finished();
    flexible_ = gtsam::noiseModel::Diagonal::Sigmas(v6p);
}

template <>
void
PNP<gtsam::EssentialMatrix, gtsam::Point2>::
initializeCorrespondenceNoiseModel()
{
    //assume some sigmas for E.
    gtsam::Vector5 v5p;
    v5p = (gtsam::Vector(5) << 1.0, 1.0, 0.5, 0.5, 0.5).finished(); //TODO: correct?
    flexible_ = gtsam::noiseModel::Diagonal::Sigmas(v5p);
}

template <class T, class P>
void
PNP<T,P>::
setDebug()
{
    debug_ = true;
}

template <class T, class P>
void
PNP<T,P>::
setInliers(std::shared_ptr<std::vector<double>> inliers)
{
    explicit_filter_ = inliers->size() > 0;
    inliers_ = inliers;
}

template <class T, class P>
void
PNP<T,P>::
setNoiseModel(double acceptable_rerror, PNP<T,P>::NM noise_model, const int& N)
{
    gtsam::noiseModel::Base::shared_ptr measurement_noise_outlier_free_ = gtsam::noiseModel::Isotropic::Sigma(N, acceptable_rerror);
    switch(noise_model)
    {
        case PNP<T,P>::OUTLIER_FREE:
            measurement_noise_ = measurement_noise_outlier_free_;
            break;
        case PNP<T,P>::HUBER:
            measurement_noise_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(acceptable_rerror), measurement_noise_outlier_free_);
            break;
        case PNP<T,P>::GEMAN_MCCLURE:
            measurement_noise_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(acceptable_rerror), measurement_noise_outlier_free_);
            break;
        default:
            throw std::runtime_error("PNP::setNoiseModel() error. unknown loss.");
    }
}

template <class T, class P>
void
PNP<T,P>::
addPose(gtsam::Symbol symb)
{
    graph_.add(gtsam::PriorFactor<T>(symb, pguess_, flexible_));
    initial_estimate_.insert(symb, pguess_);
}

template <>
void
PNP<gtsam::Pose3, gtsam::Point3>::
addLocalizationFactor(gtsam::Symbol symb, size_t i)
{
    static gtsam::Cal3_S2::shared_ptr gt_camera = cam_.GetGTSAMCam();
    graph_.add(LocalizationFactor<gtsam::Pose3, gtsam::Cal3_S2>(p2d_subset_[i], p3d_subset_[i], measurement_noise_, symb, gt_camera));
}

template <>
void
PNP<gtsam::EssentialMatrix, gtsam::Point2>::
addLocalizationFactor(gtsam::Symbol symb, size_t i)
{
    static gtsam::Cal3_S2::shared_ptr gt_camera = cam_.GetGTSAMCam();
    
    graph_.add(CustomEFactor(symb, p3d_subset_[i], p2d_subset_[i], measurement_noise_));
//    graph_.add(gtsam::EssentialMatrixFactor(symb, p3d_subset_[i], p2d_subset_[i], measurement_noise_, gt_camera));
}

template <class T, class P>
void
PNP<T,P>::
addLocalizationFactors(gtsam::Symbol symb)
{
    for(int i=0; i<p2d_subset_.size(); ++i)
    {
        if(explicit_filter_ and (*inliers_)[i] < 0.0) //unnecessary if using GM. TODO: test to see which parameter values for GM + Huber produce the best results.
        {
            continue;
        }
        ++nlocalization_factors;
        addLocalizationFactor(symb, i);
    }
}

template <class T, class P>
gtsam::Symbol
PNP<T,P>::
constructGraph()
{
    gtsam::Symbol symb('x', 0);
    addPose(symb);
    addLocalizationFactors(symb);
    return symb;
}

template <class T, class P>
std::tuple<bool, gtsam::Values>
PNP<T,P>::
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
            exit(1);
        }
    }
    graph_.resize(0);
    initial_estimate_.clear();
    return std::make_tuple(suc, result);
}

template <class T, class P>
std::tuple<bool, T>
PNP<T,P>::
run()
{
    gtsam::Symbol symb = constructGraph();
    gtsam::Values result;
    bool suc;
    std::tie(suc, result) = optimize();

    if(result.size()==0)
    {
        return std::make_tuple(false, T());
    }
    else
    {
        return std::make_tuple(suc, result.at<T>(symb));
    }
}

//explicit template specialization for a class template (see https://stackoverflow.com/a/13952386/6834155)
//this *must* go at the bottom
template class PNP<gtsam::EssentialMatrix, gtsam::Point2>;
template class PNP<gtsam::Pose3, gtsam::Point3>;
