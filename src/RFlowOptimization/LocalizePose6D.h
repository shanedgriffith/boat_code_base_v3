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
protected:
    const std::vector<gtsam::Point3>& p3d_;
    const std::vector<gtsam::Point2>& p2d_;
    std::vector<double> inliers_;
    
    gtsam::Pose3
    UseBA(const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d, const std::vector<double>& inliers, int iter = 0);
    
    void
    AddLocalizationFactors(gtsam::Symbol symb, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d, const std::vector<double>& inliers, int iter = 0);
    
    void
    AddPose(gtsam::Symbol symb, const gtsam::Pose3& pguess);
    
    std::tuple<gtsam::Pose3, std::vector<double>>
    RANSAC_BA(const gtsam::Pose3& p1guess, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers);
    
    std::tuple<gtsam::Pose3, std::vector<double>>
    RANSAC_P3P(const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers);
    
    double MeasureReprojectionError();
    
    
public:
    
    LocalizePose6D(const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d);
    
    //for Pose3
    std::tuple<gtsam::Pose3, std::vector<double>>
    UseBAIterative();
    
    std::tuple<gtsam::Pose3, std::vector<double>>
    combinedLocalizationMethod(const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers);
    
    
};
