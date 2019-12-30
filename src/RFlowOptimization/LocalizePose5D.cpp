//#include "LocalizePose5D.h"
//
//#include <gtsam/slam/EssentialMatrixFactor.h>
//
//std::tuple<gtsam::EssentialMatrix, std::vector<double>>
//LocalizePose5D::
//RANSAC_Nister()
//{
//    
//}
//
//std::tuple<gtsam::EssentialMatrix, std::vector<double>>
//LocalizePose5D::
//RANSAC_BA() //const gtsam::EssentialMatrix& emguess, const std::vector<gtsam::Point2>& p2d0, std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers
//{
//    //EM approach to finding the best pose.
//    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
//    gtsam::EssentialMatrix best_pose = emguess; // TODO: get this from somewhere. (probably run Nister.)
//    const int SAMPLE_SIZE = 4;
//    std::vector<int> rset(SAMPLE_SIZE); //15 because it's using BA to solve for the pose. MIN_CORRESPONDENCES);
//    std::vector<gtsam::Point2> subp2d0(rset.size());
//    std::vector<gtsam::Point2> subp2d1(rset.size());
//    std::vector<double> subinliers(rset.size(), ACCEPTABLE_TRI_RERROR);
//    std::vector<double> best_posevals(4, 0.0);
//    int iters=0;
//    int last_save_iter = 0;
//    double err = ACCEPTABLE_TRI_RERROR;
//    
//    int n_iters = MAX_RANSAC_ITERS;
//    
//    for(; iters<n_iters; iters++){
//        GenerateRandomSet(p2d0.size(), rset);
//        for(int j=0; j<rset.size(); j++)
//        {
//            subp2d0[j] = p2d0[rset[j]];
//            subp2d1[j] = p2d1[rset[j]];
//        }
//        gtsam::EssentialMatrix estp = UseBA(best_pose, subp2d0, subp2d1, subinliers);
//        
//        if(EmptyPose(estp)) continue;
//        std::vector<double> posevals = Maximization(estp, p2d0, p2d1, inliers, err);
//        if(posevals[1]>best_posevals[1] or
//           (posevals[1]==best_posevals[1] and posevals[2] < best_posevals[2]))
//        {
//            swap(best_posevals, posevals);
//            best_pose = estp;
//            last_save_iter = iters;
//        }
//        
//        int n_total_iters = ceil(NumRequiredRANSACIterations(best_posevals[1], p2d0.size(), SAMPLE_SIZE, 0.99));
//        
//        if(n_total_iters < 0) continue;
//        
//        n_iters = std::min(n_iters, n_total_iters);
//        
//        //makes RANSAC faster by 10x (1ms to 10ms), but it's less consistent
//        //if(best_posevals[1]/p3d.size()>RANSAC_PERC_DC || iters-last_save_iter>=RANSAC_IMPROV_ITERS) break;
//    }
//    
//    if(debug)
//    {
//        printf("ransac iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
//               (int)iters, (int)best_posevals[0], best_posevals[2], best_posevals[3], (int)best_posevals[1], (int)p2d0.size());
//    }
//    
//    //have to reset the inliers.
//    std::vector<double> posevals = Maximization(best_pose, p2d0, p2d1, inliers, err);
//    
//    best_posevals[0] = iters;
//    return std::make_tuple(best_pose, best_posevals);
//}
//
//std::tuple<gtsam::EssentialMatrix, std::vector<double>>
//LocalizePose5D::
//UseBAIterative(std::vector<gtsam::Point2>& p2d0, std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers)
//{
//    //TODO: make this function consistent with the DualIterativeBA(). (and update RANSAC_BA() as well)
//    //EM approach to finding the best pose.
//    //returns: {best pose, info}, where info is {# of iterations, # of inliers, average reprojection error, average reprojection error of inliers}
//    
//    if(p2d0.size() < MIN_CORRESPONDENCES)
//    {
//        return {};
//    }
//    gtsam::EssentialMatrix best_pose;
//    std::vector<double> best_posevals;
//    double best_score;
//    int minpiter = -1;
//    gtsam::EssentialMatrix estp;
//    
//    //use RANSAC (with EM of sorts; uses the updated best pose) to find the best estimate of p1frame0.
//    std::vector<double> posevalsransac;
//    switch(RANSAC_MODEL)
//    {
//        case 0:
//            std::tie(estp, posevalsransac) = RANSAC_NISTER(p2d0, p2d1, inliers);
//            break;
//        case 1:
//            gtsam::EssentialMatrix first_guess; //TODO.
//            std::tie(estp, posevalsransac) = RANSAC_BA(first_guess, p2d0, p2d1, inliers);
//            break;
//        default:
//            std::cout << "set the ransac model" << std::endl;
//            exit(-1);
//    }
//    
//    if(posevalsransac[1]<0.000001) return {};
//    
//    //measure rerror with the previous set of inliers, if it's good, update the set of inliers.
//    int iters = 0;
//    int nchanges=1;
//    double err = ACCEPTABLE_TRI_RERROR;
//    //while(err>6.0 ||(nchanges > 0 && iters < MAX_ITERS)){
//    for(int i=0; nchanges > 0 and i < MAX_ITERS; i++)
//    {
//        UseBA(estp, p2d0, p2d1, inliers, i);
//        if(EmptyPose(estp)) break;
//        std::vector<double> posevals = Maximization(estp, p2d0, p2d1, inliers, err);
//        
//        if(iters==0 or posevals[1]>best_score) //using the reprojection error of the inlier set, rather than the number of inliers.
//        {
//            best_score = posevals[1];
//            best_pose = estp;
//            best_posevals = posevals;
//            minpiter = iters;
//        }
//        iters++;
//        nchanges = posevals[0];
//        
//        if(debug)
//        {
//            printf("bai iter[%d]: %d changes; reprojection error: %lf (all), %lf (inliers); number of inliers %d of %d\n",
//                   (int)iters, (int)posevals[0], posevals[2], posevals[3], (int)posevals[1], (int)p2d0.size());
//        }
//        if(robust_loss_ and iters > 1)
//            break;
//    }
//    if(minpiter != iters-1) Maximization(best_pose, p2d0, p2d1, inliers, err); //reset the inliers.
//    if(best_posevals.size()==0)
//        return {};
//    best_posevals[0] = iters;
//    return std::make_tuple(best_pose, best_posevals);
//}
//
