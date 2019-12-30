#pragma once


#include <gtsam/geometry/EssentialMatrix.h>


//class LocalizePose5D: public Localization
//{
//protected:
//    const gtsam::EssentialMatrix& estp_;
//    const gtsam::EssentialMatrix& best_estimate_;
//    const std::vector<gtsam::Point2>& p2d0_;
//    const std::vector<gtsam::Point2>& p2d1_;
//    
//    std::tuple<gtsam::EssentialMatrix, std::vector<double>>
//    RANSAC_BA();
//    
//    std::tuple<gtsam::EssentialMatrix, std::vector<double>>
//    RANSAC_Nister();
//    
//public:
//    
//    
//    std::tuple<gtsam::EssentialMatrix, std::vector<double>>
//    UseBAIterative(const std::vector<gtsam::Point2>& p2d0, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers);
//    
//};
