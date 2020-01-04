#include "LocalizePose6D.h"

#include "P3P.h"
#include "PNP.h"
#include "Optimization/MultiSession/LocalizationFactor.h"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>

LocalizePose6D::
LocalizePose6D(const Camera& cam, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d)
: Localization(cam)
, p3d_(p3d)
, p2d_(p2d)
, inliers_(p3d.size(), 0.0)
, ransac_method_(LocalizePose6D::METHOD::PNP)
, p3d_set_(p3d_subset_)
, p2d_set_(p2d_subset_)
{
    //TODO.
    if(p3d_.size() < SAMPLE_SIZE)
    {
        throw std::runtime_error("LocalizePose6D::LocalizePose6D() Error. Need at least " + std::to_string(SAMPLE_SIZE) + " correspondences");
    }
}

void
LocalizePose6D::
setRANSACMethod(LocalizePose6D::METHOD method)
{
    ransac_method_ = method;
}

void
LocalizePose6D::
setInitialEstimate(const gtsam::Pose3& guess)
{
    pguess_ = guess;
    best_guess_ = guess;
}

void
LocalizePose6D::
updateResult()
{
    best_guess_ = pguess_;
}

size_t
LocalizePose6D::
setSize()
{
    return p3d_.size();
}

size_t
LocalizePose6D::
sampleSize()
{
    return SAMPLE_SIZE;
}

std::vector<double>
LocalizePose6D::
getInliers()
{
    return inliers_;
}


void
LocalizePose6D::
setErrorThreshold(double e)
{
    ACCEPTABLE_TRI_RERROR = e;
}

double
LocalizePose6D::
MeasureReprojectionError() //const gtsam::Pose3& gtp, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d, const std::vector<unsigned char>& inliers
{
    double sumall=0;
    double sumin=0;
    int cin=0;
    
    int count=0;
    for(int i=0; i<p3d_.size(); i++)
    {
        gtsam::Point3 tfp = pguess_.transform_to(p3d_[i]);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        if(!cam_.InsideImage(res)) continue;
        double dist = res.distance(p2d_[i]);
        sumall+=dist;
        count++;
        if(inliers_[i])
        {
            sumin+=dist;
            cin++;
        }
    }
    if(debug_)
    {
        std::cout<<"num points: " << count << ", reprojection error: "<<sumall/count<<", inliers only: "<<sumin/cin<< ", "<< cin << " of " << count << " are inliers." << std::endl;
    }
    
    return sumall/count;
}

std::vector<double>
LocalizePose6D::
Maximization()
{
    int nchanges=0;
    int ninliers = 0;
    double sumall=0;
    double sumin=0;
    for(int i=0; i<p3d_.size(); i++){
        gtsam::Point3 tfp = pguess_.transform_to(p3d_[i]);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        double dist = res.distance(p2d_[i]);
        if(dist > ACCEPTABLE_TRI_RERROR || !cam_.InsideImage(res))
        {
            if(inliers_[i]>=0.0) nchanges++;
            inliers_[i] = -1;
            if(!cam_.InsideImage(res)) continue;
        }
        else
        {
            if(inliers_[i]<0) nchanges++;
            inliers_[i] = ACCEPTABLE_TRI_RERROR;//dist; //I seem to get more reliable estimates using err, rather than dist, here.
            sumin+=dist;
            ninliers++;
        }
        sumall+=dist;
    }
    return {(double) nchanges, (double) ninliers, sumall/p3d_.size(), sumin/ninliers};
}

void
LocalizePose6D::
updateSubsets(const std::vector<size_t>& rset)
{
    if(rset.size() == 0)
    {
        p3d_set_ = p3d_;
        p2d_set_ = p2d_;
        return;
    }
    else
    {
        p3d_set_ = p3d_subset_;
        p2d_set_ = p2d_subset_;
    }
    p3d_subset_.resize(rset.size());
    p2d_subset_.resize(rset.size());
    for(int j=0; j<rset.size(); j++)
    {
        p3d_subset_[j] = p3d_[rset[j]];
        p2d_subset_[j] = p2d_[rset[j]];
    }
}

void
LocalizePose6D::
updateOptimizationMethod()
{
    ransac_method_ = LocalizePose6D::METHOD::PNP;
}

bool
LocalizePose6D::
runMethod()
{
    bool success;
    switch(ransac_method_)
    {
        case LocalizePose6D::METHOD::P3P:
        {
            P3P localizer(cam_, p3d_set_, p2d_set_);
            std::tie(success, pguess_) = localizer.run();
            break;
        }
        case LocalizePose6D::METHOD::PNP:
        {
            PNP localizer(cam_, best_guess_, p3d_set_, p2d_set_);
            std::tie(success, pguess_) = localizer.run();
            break;
        }
        default:
            std::cout << "Localization::METHOD not recognized." << std::endl;
            exit(-1);
            break;
    }
    
    return success;
}

std::tuple<bool, gtsam::Pose3, std::vector<double>>
LocalizePose6D::
UseBAIterative()
{
    std::vector<double> posevals = RANSAC();
    bool suc = posevals[1] < 0.000001;
    if(suc)
    {
        posevals = iterativeBA();
    }
    
    return std::make_tuple(suc, best_guess_, posevals);
}

