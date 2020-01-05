#include "Localization.h"

#include "P3P.h"
#include "PNP.h"

#include <random>

Localization::
Localization(const Camera& cam)
: cam_(cam)
, debug_(false)
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
GenerateRandomSet(size_t n, size_t k)
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

std::vector<double>
Localization::
RANSAC()
{
    //the standard RANSAC loop.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers
    std::vector<double> best_posevals(4, 0.0);
    size_t SET_SIZE = setSize();
    size_t SAMPLE_SIZE = sampleSize();
    
    size_t n_iters = MAX_RANSAC_ITERS;
    int nchanges = 0;
    for(size_t iters = 0; iters < n_iters; ++iters)
    {
        std::vector<size_t> rset = GenerateRandomSet(SET_SIZE, SAMPLE_SIZE);
        
        updateSubsets(rset);
        
        bool success = runMethod();
        if(not success) continue;
        
        std::vector<double> posevals = Maximization();
        if(posevals[1]>best_posevals[1] or
           (posevals[1]==best_posevals[1] and posevals[2] < best_posevals[2]))
        {
            swap(best_posevals, posevals);
            updateResult(); //or set initial estimate?
            nchanges = best_posevals[0];
            best_posevals[0] = iters;
        }
        
        int n_total_iters = ceil(NumRequiredRANSACIterations(best_posevals[1], SET_SIZE, SAMPLE_SIZE, 0.99));
        
        if(n_total_iters < 0) continue;
        
        n_iters = std::min(static_cast<int>(n_iters), n_total_iters);
    }
    
    if(debug_)
    {
        printf("ransac iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %zu\n",
               (int) best_posevals[0], (int) nchanges, best_posevals[2], best_posevals[3], (int) best_posevals[1], SET_SIZE);
    }
    
    //reset the inliers. (don't keep the returned result)
    Maximization();
    
    return best_posevals;
}

std::vector<double>
Localization::
iterativeBA()
{
    //TODO: make this function consistent with the DualIterativeBA(). (and update RANSAC_BA() as well)
    //EM approach to finding the best pose.
    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
    
    //assumes that the initial estimate of the pose is good; e.g., obtained with RANSAC.
    std::vector<double> best_posevals(4,0);
    
    updateOptimizationMethod();
    
    updateSubsets();
    
    int nchanges=1;
    for(size_t iters = 0; nchanges > 0 and iters < MAX_OPTIMIZATION_ITERS; ++iters)
    {
        bool success = runMethod();
        if(not success) continue;
        
        std::vector<double> posevals = Maximization();
        
        if(iters==0 or posevals[1]>best_posevals[1]) //using the reprojection error of the inlier set, rather than the number of inliers.
        {
            std::swap(best_posevals, posevals);
            updateResult();
            best_posevals[0] = iters;
        }
        
        nchanges = posevals[0];
        
        if(debug_)
        {
            printf("bai iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %zu\n",
                   (int) iters, (int) posevals[0], posevals[2], posevals[3], (int) posevals[1], setSize());
        }
        if(robust_loss_ and iters > 1)
            break;
    }
    
    //reset the inliers. (don't keep the returned result)
    Maximization();
    
    return best_posevals;
}














