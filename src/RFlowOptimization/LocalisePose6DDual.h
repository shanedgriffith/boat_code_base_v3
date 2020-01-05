#pragma once

#include "Localization.h"
#include "LocalizePose6D.h"


//TODO. implementation that fits with the new Localization interface.

//class LocalisePose6DDual : public LocalizePose6D {
//protected:
//    
//    const std::vector<gtsam::Point3>& b3d;
//    const std::vector<gtsam::Point2>& b2d0;
//    
//    bool DualBA(double val,
//                gtsam::Pose3 p0, gtsam::Pose3& p1frame0, std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
//                gtsam::Pose3 p1, gtsam::Pose3& p0frame1, std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1);
//    
//    double
//    GetBestValueForInterposeVar(gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 p1frame0, gtsam::Pose3 p0frame1,
//                                std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
//                                std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1);
//    
//    std::tuple<gtsam::Pose3, gtsam::Pose3, std::vector<double>>
//    DualIterativeBA(const gtsam::Pose3& p0, const gtsam::Pose3& p1, const gtsam::Pose3& p1frame0, const gtsam::Pose3& p0frame1,
//                    const std::vector<gtsam::Point3>& f3d, const std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
//                    const std::vector<gtsam::Point3>& b3d, const std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1);
//    
//    gtsam::Pose3 p1frame0_guess_;
//    gtsam::Pose3 p0frame1_guess_;
//    
//public:
//    
//    LocalisePose6DDual(const Camera& cam, const gtsam::Pose3& p0, const gtsam::Pose3& p1,
//                       const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& rerrorp,
//                       const std::vector<gtsam::Point3>& b3d, const std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerrorb);
//    
//    
//    std::tuple<bool, gtsam::Pose3, std::vector<double>>
//    RobustDualBA();
//    
//    
//};
