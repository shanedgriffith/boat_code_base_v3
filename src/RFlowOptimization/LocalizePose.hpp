/*
 * LocalizePose.hpp
 *
 *  Created on: Jan 17, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_LOCALIZEPOSE_HPP_
#define SRC_RFLOWOPTIMIZATION_LOCALIZEPOSE_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

#include <DataTypes/Camera.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>

/*Code to localize a pose using points from an existing map. That is known 3d points to 2d correspondences. This method works
 * far better than OpenCVs SolvePnP() methods. Here, EM is used with an iterative approach to identify good correspondences
 * and then use those to localize the pose. The method stops when the number of inliers stops changing or MAX_ITERS is reached,
 * whichever comes first.
 *
 * The two camera poses at which the original correspondences were obtained are necessary in these functions. The first pose is
 * static. The second pose is an initial estimate. Although the second pose can be triangulated without this info, it's more
 * stable with it, since the original point correspondences are also provided for the first static pose.
 * */
class LocalizePose{
protected:
    double ACCEPTABLE_TRI_RERROR = 6.0;
//    double ACCEPTABLE_INTERPOSE_VAR = 0.05;//0.005 works for 140926, but is too tight for 140625 //now this parameter is adaptive.
    int MAX_ITERS = 15;
    int MAX_RANSAC_ITERS = 100;
    int RANSAC_IMPROV_ITERS = 100;
    double RANSAC_PERC_DC = 0.5;
    int MIN_CORRESPONDENCES = 4;
    int RANSAC_MODEL = 0;

    std::vector<double> Maximization(gtsam::Pose3& gtp, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers, double err);
    void UseBA(gtsam::Pose3& pguess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers);
    void AddLocalizationFactors(gtsam::Symbol symb, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers);
    void AddPose(gtsam::Symbol symb, gtsam::Pose3 pguess, std::vector<double> noise);
    gtsam::Values RunBA();

    bool EmptyPose(gtsam::Pose3& p);

    bool DualBA(double val,
                gtsam::Pose3 p0, gtsam::Pose3& p1frame0, std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
                gtsam::Pose3 p1, gtsam::Pose3& p0frame1, std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1);

    double GetBestValueForInterposeVar(gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 p1frame0, gtsam::Pose3 p0frame1,
                                                     std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
                                                     std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1);
    
    std::vector<std::vector<double> > DualIterativeBA(gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 p1frame0, gtsam::Pose3 p0frame1,
                                                     std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
                                                     std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1);
    
    void GenerateRandomSet(int n, std::vector<int>& rset);
    double NumRequiredRANSACIterations(int ninliers, int setsize, int nsamples_per_iteration, double probability_all_inliers);
    std::vector<double> RANSAC_BA(gtsam::Pose3& p1guess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers);
    
    std::vector<double> RANSAC_P3P(gtsam::Pose3& p1guess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers);
    
    gtsam::Pose3 disambiguatePoses(const std::vector<gtsam::Pose3>& poses, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d);
    
    void removeZeroPoints(std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1);
    
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initEst;
    const Camera& _cam;
public:
    bool debug = false;
    LocalizePose(const Camera& cam):_cam(cam){}

    void SetErrorThreshold(double e) {ACCEPTABLE_TRI_RERROR = e;}
    
    double MeasureReprojectionError(std::vector<double>& pnppose, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<unsigned char>& inliers);

    std::vector<std::vector<double> > UseBAIterative(std::vector<double> pguess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers);

    void PrintVec(std::vector<double> p);
    
    std::vector<std::vector<double> > RobustDualBA(std::vector<double> p0, std::vector<double> p1,
                                                                std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1, std::vector<double>& rerrorp,
                                                                std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerrorb);
    
    gtsam::Pose3 RunP3P(std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1);
    
    std::vector<std::vector<double>> combinedLocalizationMethod(std::vector<double> pguess, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers);
    
    void testP3P();
    
    void testP3PStatic();
    void testLocalizePoses();
    
    void setRANSACModel(int model) {RANSAC_MODEL = model;}
    
//    void test();
};



#endif /* SRC_RFLOWOPTIMIZATION_LOCALIZEPOSE_HPP_ */
