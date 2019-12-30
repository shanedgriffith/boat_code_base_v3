#include "Localization.cpp"



Localization::
Localization(const Camera& cam)
: cam_(cam)
{}

void
Localization::
setRobustLoss()
{
    robust_loss_ = true;
}

void
Localization::
setRANSACModel(int model)
{
    ransac_model_ = model;
}

void
Localization::
setErrorThreshold(double e)
{
    ACCEPTABLE_TRI_RERROR = e;
}

void setDebug()
{
    debug_ = true;
}

double
Localization::
NumRequiredRANSACIterations(int ninliers, int setsize, int nsamples_per_iteration, double probability_all_inliers)
{
    double w = 1.0 * ninliers / setsize;
    return log(1-probability_all_inliers) / log(1.0-pow(w, nsamples_per_iteration));
}

//n choose k in O(k)
std::vector<size_t>
Localization::
GenerateRandomSet(int n, int k)
{   //meant to be called with a value for n that doesn't change
    static std::vector setindices;
    
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





















