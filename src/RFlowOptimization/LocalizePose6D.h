#pragma once

#include "Localization.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/NoiseModel.h>


/*Code to localize a pose using points from an existing map. That is known 3d points to 2d correspondences. This method works
 * far better than OpenCVs SolvePnP() methods. Here, EM is used with an iterative approach to identify good correspondences
 * and then use those to localize the pose. The method stops when the number of inliers stops changing or MAX_ITERS is reached,
 * whichever comes first.
 *
 * The two camera poses at which the original correspondences were obtained are necessary in these functions. The first pose is
 * static. The second pose is an initial estimate. Although the second pose can be triangulated without this info, it's more
 * stable with it, since the original point correspondences are also provided for the first static pose.
 * */
class LocalizePose6D: public Localization
{
public:
    
    enum METHOD {P3P=0, PNP}; //class template better?
    
protected:
    const int SAMPLE_SIZE = 4;
    double ACCEPTABLE_TRI_RERROR = 6.0;
    
    size_t
    setSize();
    
    size_t
    sampleSize();
    
    void
    updateSubsets(const std::vector<size_t>& rset = {});
    
    void
    updateOptimizationMethod();
    
    double
    MeasureReprojectionError();
    
    std::vector<double>
    Maximization();
    
    void
    updateResult();
    
    bool
    runMethod();
    
    
    gtsam::Pose3 pguess_;
    gtsam::Pose3 best_guess_;
    const std::vector<gtsam::Point3>& p3d_;
    const std::vector<gtsam::Point2>& p2d_;
    std::vector<double> inliers_;
    LocalizePose6D::METHOD ransac_method_;
    std::vector<gtsam::Point3> p3d_subset_;
    std::vector<gtsam::Point2> p2d_subset_;
    std::vector<gtsam::Point3>& p3d_set_; //extra references to avoid a copy.
    std::vector<gtsam::Point2>& p2d_set_;
    
    
public:
    
    LocalizePose6D(const Camera& cam, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d);
    
    std::tuple<bool, gtsam::Pose3, std::vector<double>>
    UseBAIterative();
    
    std::vector<double>
    getInliers();
    
    void
    setInitialEstimate(const gtsam::Pose3& guess);
    
    void setRANSACMethod(LocalizePose6D::METHOD method);
    
    void
    setErrorThreshold(double e);
};
