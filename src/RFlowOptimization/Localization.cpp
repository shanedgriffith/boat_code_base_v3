#include "Localization.h"

#include <random>

Localization::
Localization(const Camera& cam)
: cam_(cam)
, debug_(false)
, ransac_method_(Localization::METHOD::PNP)
, robust_loss_(false)
{}

void
Localization::
setRobustLoss()
{
    robust_loss_ = true;
}

void
Localization::
setRANSACMethod(Localization::METHOD method)
{
    ransac_method_ = method;
}

void
Localization::
setErrorThreshold(double e)
{
    ACCEPTABLE_TRI_RERROR = e;
}

void
Localization::
setDebug()
{
    debug_ = true;
}

double
Localization::
NumRequiredRANSACIterations(size_t ninliers, size_t setsize, size_t nsamples_per_iteration, double probability_all_inliers)
{
    double w = 1.0 * ninliers / setsize;
    return log(1-probability_all_inliers) / log(1.0-pow(w, nsamples_per_iteration));
}

//n choose k in O(k)
std::vector<size_t>
Localization::
GenerateRandomSet(int n, int k)
{   //meant to be called with a value for n that doesn't change
    static std::vector<int> setindices;
    
    if(setindices.size() != n)
    {
        setindices = std::vector<int>(n);
        for(int i=0; i<n; ++i)
            setindices[i] = i;
    }
    
    std::random_device r;
    std::default_random_engine e1(r());
    
    //random set of entries between 0 and n without repeats.
    std::vector<size_t> rset(k);
    for(size_t nadded = 0; nadded<rset.size(); ++nadded)
    {
        std::uniform_int_distribution<size_t> uniform_dist(nadded, n-1);
        size_t rnum = uniform_dist(e1);
        rset[nadded] = setindices[rnum];
    }
    return rset;
}

std::tuple<bool, gtsam::Pose3>
Localization::
runMethod(const gtsam::Pose3& guess, const std::vector<gtsam::Point3>& subp3d, const std::vector<gtsam::Point2>& subp2d1)
{
    switch(ransac_method_)
    {
        case Localization::METHOD::P3P:
            P3P localizer(cam_, subp3d, subp2d1);
            return localizer.run();
            break;
        case Localization::METHOD::PNP:
            PNP localizer(cam_, guess, subp3d, subp2d1);
            return localizer.run();
            break;
        default:
    }
    std::cout << "Localization::METHOD not recognized." << std::endl;
    exit(-1);
}


















