#include "LocalisePose6DDual.h"



LocalisePose6DDual(const gtsam::Pose3& p0, const gtsam::Pose3& p1,
                   const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& rerrorp,
                   const std::vector<gtsam::Point3>& b3d, const std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerrorb)
//:
{
    
}

double
LocalisePose6DDual::
GetBestValueForInterposeVar(const gtsam::Pose3& p1frame0, const gtsam::Pose3& p0frame1)
/*
 gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 p1frame0, gtsam::Pose3 p0frame1,
 std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
 std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1
 */
{
    std::vector<double> posevals(4, 0.0);
    std::vector<double> inliers0(rerror0.size(), ACCEPTABLE_TRI_RERROR);
    std::vector<double> inliers1(rerror1.size(), ACCEPTABLE_TRI_RERROR);
    double bestval = 0.005;
    int maxinliers = -1;
    double err = ACCEPTABLE_TRI_RERROR;
    //0.25, 0.125, 0.0625, 0.03125, 0.0156, 0.078
    for(double val=0.005; val <= 0.4; val*=2)
    {
        bool success = DualBA(val,
                              p0, p1frame0, f3d, f2d1, rerror0,
                              p1, p0frame1, b3d, b2d0, rerror1);
        if(!success) continue;
        
        std::vector<double> pv0 = Maximization(p1frame0, f3d, f2d1, inliers0, err);
        std::vector<double> pv1 = Maximization(p0frame1, b3d, b2d0, inliers1, err);
        
        posevals[0] = pv0[0]+pv1[0];
        posevals[1] = pv0[1]+pv1[1];
        posevals[2] = (pv0[2]*f3d.size()+pv1[2]*b3d.size())/(f3d.size()+b3d.size());
        posevals[3] = (pv0[3]*pv0[1] + pv1[3]*pv1[1])/(pv0[1]+pv1[1]);
        
        if(posevals[1] > maxinliers)
        {
            maxinliers = posevals[1];
            bestval = val;
        }
    }
    
    return bestval;
}

std::tuple<gtsam::Pose3, gtsam::Pose3, std::vector<double>>
LocalisePose6DDual::
DualIterativeBA(const gtsam::Pose3& p1frame0, const gtsam::Pose3& p0frame1)
/*
 const gtsam::Pose3& p0, const gtsam::Pose3& p1, const gtsam::Pose3& p1frame0, const gtsam::Pose3& p0frame1,
 const std::vector<gtsam::Point3>& f3d, const std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
 const std::vector<gtsam::Point3>& b3d, const std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1
 */
{
    std::vector<double> posevals(4, 0.0);
    int minpiter = -1;
    double best_score;
    
    bool success = false;
    int nchanges=1;
    int i=0;
    double err = ACCEPTABLE_TRI_RERROR;
    double val = GetBestValueForInterposeVar(p0, p1, p1frame0, p0frame1, f3d, f2d1, rerror0, b3d, b2d0, rerror1);
    if(debug) std::cout << "Best Value For Interpose Var: " << val << std::endl;
    for(; nchanges > 0 && i < MAX_ITERS; i++)
    {
        success = DualBA(val,
                         p0, p1frame0, f3d, f2d1, rerror0,
                         p1, p0frame1, b3d, b2d0, rerror1);
        if(!success) break;
        std::vector<double> pv0 = Maximization(p1frame0, f3d, f2d1, rerror0, err);
        std::vector<double> pv1 = Maximization(p0frame1, b3d, b2d0, rerror1, err);
        
        posevals[0] = pv0[0]+pv1[0];
        posevals[1] = pv0[1]+pv1[1];
        posevals[2] = (pv0[2]*f3d.size()+pv1[2]*b3d.size())/(f3d.size()+b3d.size());
        posevals[3] = (pv0[3]*pv0[1] + pv1[3]*pv1[1])/(pv0[1]+pv1[1]);
        
        nchanges = posevals[0];
        if(debug)
        {
            printf("iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
                   (int)i, (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)(f3d.size()+b3d.size()));
        }
    }
    
    if(i==0) return {};
    
    if(debug && success)
    {
        printf("iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
               (int) (i-1), (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)(f3d.size()+b3d.size()));
    }
    
    posevals[0] = i;
    return std::make_tuple(p1frame0, p0frame1, posevals);
}

bool
LocalisePose6DDual::
DualBA(double val, gtsam::Pose3& p1frame0, gtsam::Pose3& p0frame1)
/*
 double val,
 gtsam::Pose3 p0, gtsam::Pose3& p1frame0, std::vector<gtsam::Point3>& f3d, std::vector<gtsam::Point2>& f2d1, std::vector<double>& rerror0,
 gtsam::Pose3 p1, gtsam::Pose3& p0frame1, std::vector<gtsam::Point3>& b3d, std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerror1
 */
{
    
    gtsam::Symbol symb1('x', 0);
    gtsam::Symbol symb3('x', 1);
    
    std::vector<double> flexible = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5};//0.1, 0.1, 0.25};
    
    AddLocalizationFactors(symb1, f3d, f2d1, rerror0);
    AddLocalizationFactors(symb3, b3d, b2d0, rerror1);
    AddPose(symb1, p1frame0, flexible);
    AddPose(symb3, p0frame1, flexible);
    
    //The method doesn't converge unless the poses are already close to their final values.
    //An alternative is to start this constraint loose and then tighten it after a few iterations, but
    //that more frequently caused exceptions during optimization.
    gtsam::Vector6 v6;
    v6.setConstant(val);
    gtsam::noiseModel::Diagonal::shared_ptr btwnnoise = gtsam::noiseModel::Diagonal::Sigmas(v6);
    graph.add(VirtualBetweenFactor(symb1, symb3, p0, p1, btwnnoise));
    
    gtsam::Values result = RunBA();
    if(result.size()==0) return false;
    
    p1frame0 = result.at<gtsam::Pose3>(symb1);
    p0frame1 = result.at<gtsam::Pose3>(symb3);
    return true;
}

std::tuple<gtsam::Pose3, std::vector<double>>
LocalisePose6DDual::
RobustDualBA()
/*const gtsam::Pose3& gp0, const gtsam::Pose3& gp1,
 const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& rerrorp,
 const std::vector<gtsam::Point3>& b3d, const std::vector<gtsam::Point2>& b2d0, std::vector<double>& rerrorb*/
{
    gtsam::Pose3 p1frame0(gp1.rotation(), gp0.translation());
    struct timespec start, runir, end;
    
    if(p3d.size()<MIN_CORRESPONDENCES || b3d.size()<MIN_CORRESPONDENCES)
    {
        std::cout << "Localization failed due to sizes: " <<p3d.size() << ", " << b3d.size() << std::endl;
        return {}; //TODO: valid?
    }
    
    clock_gettime(CLOCK_MONOTONIC, &start);
    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p1frame0.
    std::vector<double> posevals1f0;
    std::tie(p1frame0, posevals1f0) = RANSAC_P3P(p3d, p2d1, rerrorp);
    
    if(posevals1f0[1]<0.000001)
    {
        std::cout << "Localization failed due to RANSAC failure on set p"<<std::endl;
        return {}; //TODO: valid?
    }
    
    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p0frame1.
    //gtsam::Pose3 p0frame1 = gp1.compose(p1frame0.between(gp0)); //this also looks wrong.
    gtsam::Pose3 p0frame1 = gp1.compose(gp1.between(p1frame0)*p1frame0.between(gp0)*p1frame0.between(gp1));
    std::vector<double> posevals0f1;
    std::tie(p0frame1, posevals0f1) = RANSAC_P3P(b3d, b2d0, rerrorb);
    
    clock_gettime(CLOCK_MONOTONIC, &runir);
    if(posevals0f1[1]<0.000001)
    {
        std::cout << "Localization failed due to RANSAC failure on set b"<<std::endl;
        return {}; //TODO: valid?
    }
    
    //iterative localization (like EM), started with the good initial estimates found using RANSAC.
    gtsam::Pose3 estimated;
    std::vector<double> stats;
    std::tie(estimated, stats) = DualIterativeBA(gp0, gp1, p1frame0, p0frame1, p3d, p2d1, rerrorp, b3d, b2d0, rerrorb);
    
    if(debug)
    {
        clock_gettime(CLOCK_MONOTONIC, &end);
        std::string irtime = std::to_string((runir.tv_sec - start.tv_sec) + (runir.tv_nsec - start.tv_nsec)/1000000000.0);
        std::string altime = std::to_string((end.tv_sec - runir.tv_sec) + (end.tv_nsec - runir.tv_nsec)/1000000000.0);
        std::string tottime = std::to_string((end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec)/1000000000.0);
        std::cout << "runtimes (s) "<<irtime << " + " << altime << " ~= " << tottime << std::endl;
    }
    
    return std::make_tuple(estimated, stats); //TODO: can these values be forwarded instead of untied/retied?
}
