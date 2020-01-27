#include "LocalizePose5D.h"

#include "Nister5point.h" //see, e.g., https://github.com/opencv/opencv/blob/master/modules/calib3d/src/five-point.cpp
#include "PNP.h"

LocalizePose5D::
LocalizePose5D(const Camera& cam, const std::vector<gtsam::Point2>& p2d0, const std::vector<gtsam::Point2>& p2d1)
: Localization(cam)
, p2d0_(p2d0)
, p2d1_(p2d1)
, ransac_method_(LocalizePose5D::METHOD::_PNP)
{
    if(p2d0_.size() < SAMPLE_SIZE)
    {
        throw std::runtime_error("LocalizePose5D::LocalizePose5D() Error. Need at least " + std::to_string(SAMPLE_SIZE) + " correspondences");
    }
    inliers_ = std::make_shared<std::vector<double>>(p2d0.size(), 0);
}

void
LocalizePose5D::
setRANSACMethod(LocalizePose5D::METHOD method)
{
    ransac_method_ = method;
}

void
LocalizePose5D::
setInitialEstimate(const gtsam::EssentialMatrix& guess)
{
    pguess_ = guess;
    best_guess_ = guess;
}

void
LocalizePose5D::
updateGuess()
{
    pguess_ = best_guess_;
}

void
LocalizePose5D::
updateResult()
{
    best_guess_ = pguess_;
}

size_t
LocalizePose5D::
setSize()
{
    return p2d0_.size();
}

size_t
LocalizePose5D::
sampleSize()
{
    return SAMPLE_SIZE;
}

std::shared_ptr<std::vector<double>>
LocalizePose5D::
getInliers()
{
    return inliers_;
}

void
LocalizePose5D::
setErrorThreshold(double e)
{
    ACCEPTABLE_TRI_RERROR = e;
}

double
LocalizePose5D::
MeasureReprojectionError()
{
    double sumall=0;
    double sumin=0;
    int cin=0;
    
    int count=0;
//    std::cout << "dist: ";
    for(int i=0; i<p2d0_.size(); i++)
    {
        gtsam::Vector3 va = gtsam::EssentialMatrix::Homogeneous(p2d0_[i]).normalized();
        gtsam::Vector3 vb = gtsam::EssentialMatrix::Homogeneous(p2d1_[i]).normalized();
        double dp = va.dot( (pguess_.matrix() * vb)) ;
        dp = fabs(dp);
//        std::cout << ", " << dp ;
        sumall += dp;
        count++;
        if((*inliers_)[i] >= 0.0)
        {
            sumin += dp;
            cin++;
        }
    }
//    std::cout << std::endl;
    if(debug_)
    {
        std::cout<<"num points: " << count << ", reprojection error: "<<sumall/count<<", inliers only: "<<sumin/cin<< ", "<< cin << " of " << count << " are inliers." << std::endl;
    }
    
    return sumall/count;
}

std::vector<double>
LocalizePose5D::
Maximization()
{
    int nchanges=0;
    int ninliers = 0;
    double sumall=0;
    double sumin=0;
//    std::cout << "dist: ";
    for(size_t i=0; i<p2d0_.size(); ++i)
    {
        gtsam::Vector3 va = gtsam::EssentialMatrix::Homogeneous(p2d0_[i]).normalized();
        gtsam::Vector3 vb = gtsam::EssentialMatrix::Homogeneous(p2d1_[i]).normalized();
        double dp = va.dot( (pguess_.matrix() * vb));
        dp = fabs(dp);
        if(dp > ACCEPTABLE_TRI_RERROR)
        {
            if((*inliers_)[i]>=0.0) nchanges++;
            (*inliers_)[i] = -1;
        }
        else
        {
            if((*inliers_)[i]<0) nchanges++;
            (*inliers_)[i] = ACCEPTABLE_TRI_RERROR; //I seem to get more reliable estimates using err, rather than dist, here.
            sumin+=dp;
            ninliers++;
        }
        sumall+=dp;
    }
//    std::cout << std::endl;
    
    return {(double) nchanges, (double) ninliers, sumall/p2d0_.size(), sumin/ninliers};
}

void
LocalizePose5D::
updateSubsets(const std::vector<size_t>& rset)
{
    if(rset.size() == 0)
    {
        //the method will run on the original, full set
        p2d0_subset_.resize(p2d0_.size());
        p2d1_subset_.resize(p2d1_.size());
        for(int j=0; j<p2d0_.size(); ++j)
        {
            p2d0_subset_[j] = p2d0_[j];
            p2d1_subset_[j] = p2d1_[j];
        }
    }
    else
    {
        //the method will run on the subset
        p2d0_subset_.resize(rset.size());
        p2d1_subset_.resize(rset.size());
        for(int j=0; j<rset.size(); ++j)
        {
            p2d0_subset_[j] = p2d0_[rset[j]];
            p2d1_subset_[j] = p2d1_[rset[j]];
        }
    }
}

void
LocalizePose5D::
updateOptimizationMethod()
{
    ransac_method_ = LocalizePose5D::METHOD::_PNP;
}

bool
LocalizePose5D::
runMethod(bool use_robust_loss, bool use_inliers)
{
    bool success;
    switch(ransac_method_)
    {
        case LocalizePose5D::METHOD::_NISTER:
        {
            Nister5Point n5p(p2d0_subset_, p2d1_subset_);
            std::tie(success, pguess_) = n5p.run();
            break;
        }
        case LocalizePose5D::METHOD::_PNP:
        {
            PNP<gtsam::EssentialMatrix, gtsam::Point2> localizer(cam_, best_guess_, p2d0_subset_, p2d1_subset_);
            localizer.setDebug();
            PNP<gtsam::EssentialMatrix, gtsam::Point2>::NM noise_model = PNP<gtsam::EssentialMatrix, gtsam::Point2>::NM::OUTLIER_FREE;
            if(use_robust_loss)
            {
                if(iter == 0) noise_model = PNP<gtsam::EssentialMatrix, gtsam::Point2>::NM::HUBER;
                else noise_model = PNP<gtsam::EssentialMatrix, gtsam::Point2>::NM::GEMAN_MCCLURE;
            }
            else if(use_inliers) localizer.setInliers(inliers_);
            localizer.setNoiseModel<1>(ACCEPTABLE_TRI_RERROR/2.0, noise_model);
            std::tie(success, pguess_) = localizer.run();
            std::cout << "pnp? " << success << std::endl;
            if(not success)
                exit(1);
            break;
        }
        default:
            std::cout << "Localization::METHOD not recognized." << std::endl;
            exit(-1);
            break;
    }
    
    return success;
}

std::tuple<bool, gtsam::EssentialMatrix, std::vector<double>>
LocalizePose5D::
UseRANSAC()
{
    std::vector<double> posevals = RANSAC();
    bool suc = posevals[1] > 0.000001;
    return std::make_tuple(suc, best_guess_, posevals);
}

std::tuple<bool, gtsam::EssentialMatrix, std::vector<double>>
LocalizePose5D::
UseBAIterative()
{
    std::vector<double> posevals = RANSAC();
    bool suc = posevals[1] > 0.000001;
    if(suc)
    {
        posevals = iterativeBA();
    }
    
    return std::make_tuple(suc, best_guess_, posevals);
}

