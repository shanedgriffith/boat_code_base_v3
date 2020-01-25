/*
 * LocalizePose.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: shane
 */



//
//#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
//
//#include <random>
//#include <unordered_map>
//#include <cmath>
//
//#include "Optimization/SingleSession/GTSAMInterface.h"
//#include "Optimization/MultiSession/LocalizationFactor.h"
//#include "Optimization/MultiSession/VirtualBetweenFactor.h"
//#include "LocalizePose.hpp"
//
//#include "FileParsing/ParseOptimizationResults.h"
//#include "FileParsing/ParseFeatureTrackFile.h"
//
//
//
//void
//LocalizePose::testP3P()
//{
////    gtsam::Pose3 p;
////    gtsam::Pose3 pT(gtsam::Rot3::Ypr(0.1, 0.15, 0.0), gtsam::Point3(0.0, 0.5, 0));
////    
////    std::vector<gtsam::Point3> pT3d;
////    std::vector<gtsam::Point2> pT2d;
////    gtsam::Cal3_S2::shared_ptr gtcam = _cam.GetGTSAMCam();
////    gtsam::PinholeCamera<gtsam::Cal3_S2> pc(p, *gtcam);
////    std::uniform_real_distribution<double> unifx(0,704);
////    std::uniform_real_distribution<double> unify(0,480);
////    std::uniform_real_distribution<double> dunif(10,30);
////    std::default_random_engine re;
////    for(int i=0; i<15; ++i)
////    {
////        double x = unifx(re);
////        double y = unify(re);
////        gtsam::Point2 p2d(x,y);
////        double depth = dunif(re);
////        
////        gtsam::Point3 p3d = pc.backproject(p2d, depth);
////        gtsam::Point3 p3d_tf = pT.transform_to(p3d);
////        gtsam::Point2 onimage = _cam.ProjectToImage(p3d_tf);
////        if(not _cam.InsideImage(onimage.x(), onimage.y()))
////        {
////            --i;
//////            std::cout <<" " << p2d << " -> " << depth << ": "<<p3d << "; " << p3d_tf << " -> " << onimage << std::endl;
////            continue;
////        }
////        pT3d.push_back(p3d);
////        pT2d.push_back(onimage);
////    }
////    
////    std::vector<double> inliers(15, 1.0);
////    std::vector<double> correct = GTSAMInterface::PoseToVector(pT);
////    std::vector<double> vec(6,0);
////    std::uniform_real_distribution<double> uniferr(0,0.25);
////    for(int i=0; i<6; ++i)
////    {
////        double err = uniferr(re);
////        vec[i] = correct[i]+err-0.125;
////    }
////    
////    debug = true;
////    
////    std::vector<std::vector<double>> res = UseBAIterative(vec, pT3d, pT2d, inliers);
////    
////    std::cout << "\n result: ";
////    for(int i=0; i<6; ++i) std::cout << res[0][i] << ", ";
////    std::cout << "\n correct: ";
////    for(int i=0; i<6; ++i) std::cout << correct[i] << ", ";
////    std::cout << "\n estimate: ";
////    for(int i=0; i<6; ++i) std::cout << vec[i] << ", ";
//}
//
//void LocalizePose::testP3PStatic()
//{
////    std::vector<gtsam::Point3> p3d;
////    p3d.push_back(gtsam::Point3(9.4018, -1.5499,   23.0600));
////    p3d.push_back(gtsam::Point3(-2.5338,    3.2552,   21.2078));
////    p3d.push_back(gtsam::Point3(7.8374,   -0.2290,   29.9017));
////    
////    std::vector<gtsam::Vector3> fv;
////    fv.push_back(gtsam::Vector3(0.2242,   -0.1192,    0.9672));
////    fv.push_back(gtsam::Vector3(-0.2503,    0.1390,    0.9581));
////    fv.push_back(gtsam::Vector3(0.1025,   -0.0488,    0.9935));
//    
////    std::vector<gtsam::Point3> p3d;
////    p3d.push_back(gtsam::Point3(7.8374239625126618, -0.22899924803678293, 29.901690966140507));
////    p3d.push_back(gtsam::Point3(9.4018271719739328, -1.5498887495034495, 23.059974538213531));
////    p3d.push_back(gtsam::Point3(6.399907865448605, 3.7013520677203462, 15.88052298403249));
////
////    std::vector<gtsam::Vector3> fv;
////    fv.push_back(gtsam::Vector3(0.10253084628648672, -0.04876381770836747, 0.99353385228802438));
////    fv.push_back(gtsam::Vector3(0.22417194757265813, -0.11919172894844883, 0.96723330674236041));
////    fv.push_back(gtsam::Vector3(0.24338070939382267, 0.14619270947184998, 0.95885010402681758));
//    
//    std::vector<gtsam::Point3> p3d;
//    p3d.push_back(gtsam::Point3(6.399907865448605, 3.7013520677203462, 15.88052298403249));
//    p3d.push_back(gtsam::Point3(7.8374239625126618, -0.22899924803678293, 29.901690966140507));
//    p3d.push_back(gtsam::Point3(-2.533779947900519, 3.2552056641485154, 21.207798563040122));
//    
//    std::vector<gtsam::Vector3> fv;
//    fv.push_back(gtsam::Vector3(0.24338070939382267, 0.14619270947184998, 0.95885010402681758));
//    fv.push_back(gtsam::Vector3(0.10253084628648672, -0.04876381770836747, 0.99353385228802438));
//    fv.push_back(gtsam::Vector3(-0.25028760971257907,0.13904407122646398, 0.9581350941705109));
//    
//    std::vector<gtsam::Pose3> poses;
//    P3P::computePoses(fv, p3d, poses);
//}
//
//using namespace gtsam;
//
//
//std::vector<gtsam::Point3> createPoints() {
//    
//    // Create the set of ground-truth landmarks
//    std::vector<gtsam::Point3> points;
//    points.push_back(gtsam::Point3(10.0,10.0,10.0));
//    points.push_back(gtsam::Point3(-10.0,10.0,10.0));
//    points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
//    points.push_back(gtsam::Point3(10.0,-10.0,10.0));
//    points.push_back(gtsam::Point3(10.0,10.0,-10.0));
//    points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
//    points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
//    points.push_back(gtsam::Point3(10.0,-10.0,-10.0));
//    
//    return points;
//}
//
//
////std::vector<gtsam::Pose3> createPoses(
////                                      const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,-M_PI/2), gtsam::Point3(30, 0, 0)),
////                                      const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0,-M_PI/4,0), gtsam::Point3(sin(M_PI/4)*30, 0, 30*(1-sin(M_PI/4)))),
////                                      int steps = 8) {
////    
////    // Create the set of ground-truth poses
////    // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
////    std::vector<gtsam::Pose3> poses;
////    int i = 1;
////    poses.push_back(init);
////    for(; i < steps; ++i) {
////        poses.push_back(poses[i-1].compose(delta));
////    }
////    
////    return poses;
////}
////
////void LocalizePose::test()
////{
////    // Define the camera calibration parameters
////    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
////    
////    // Define the camera observation noise model
////    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
////    
////    // Create the set of ground-truth landmarks
////    std::vector<Point3> points = createPoints();
////    
////    // Create the set of ground-truth poses
////    std::vector<Pose3> poses = createPoses();
////    
////    // Create a factor graph
////    NonlinearFactorGraph graph;
////    
////    // Add a prior on pose x1. This indirectly specifies where the origin is.
////    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
////    graph.add(gtsam::PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise)); // add directly to graph
////    
////    // Simulated measurements from each camera pose, adding them to the factor graph
////    for (size_t i = 0; i < poses.size(); ++i) {
////        SimpleCamera camera(poses[i], *K);
////        for (size_t j = 0; j < points.size(); ++j) {
////            Point2 measurement = camera.project(points[j]);
////            graph.add(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K));
////        }
////    }
////    
////    // Because the structure-from-motion problem has a scale ambiguity, the problem is still under-constrained
////    // Here we add a prior on the position of the first landmark. This fixes the scale by indicating the distance
////    // between the first camera and the first landmark. All other landmark positions are interpreted using this scale.
////    noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
////    graph.add(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph
////    graph.print("Factor Graph:\n");
////    
////    // Create the data structure to hold the initial estimate to the solution
////    // Intentionally initialize the variables off from the ground truth
////    Values initialEstimate;
////    for (size_t i = 0; i < poses.size(); ++i)
////        initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
////    for (size_t j = 0; j < points.size(); ++j)
////        initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));
////    initialEstimate.print("Initial Estimates:\n");
////    
////    /* Optimize the graph and print results */
////    Values result = DoglegOptimizer(graph, initialEstimate).optimize();
////    result.print("Final results:\n");
////    std::cout << "initial error = " << graph.error(initialEstimate) << std::endl;
////    std::cout << "final error = " << graph.error(result) << std::endl;
////}
//
//void LocalizePose::removeZeroPoints(std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d1)
//{
//    static const gtsam::Point3 zero(0,0,0);
//    std::vector<gtsam::Point3> p3d_;
//    std::vector<gtsam::Point2> p2d_;
//    for(int j =0; j<p3d.size(); ++j)
//    {
//        if(p3d[j].distance(zero) < 0.001) continue;
//        p3d_.push_back(p3d[j]);
//        p2d_.push_back(p2d1[j]);
//    }
//    p3d = p3d_;
//    p2d1 = p2d_;
//}
//
//std::vector<std::vector<double>>
//LocalizePose::
//combinedLocalizationMethod(const gtsam::Pose3& pguess, const std::vector<gtsam::Point3>& p3d, const std::vector<gtsam::Point2>& p2d1, std::vector<double>& inliers)
//{
////    RANSAC_MODEL = 0;
////    std::vector<std::vector<double>> posevals = UseBAIterative(pguess, p3d, p2d1, inliers);
////    
////    if(posevals.size() > 0)
////    {
//////        if(posevals[1][1] > 0.5 * p3d.size())
//////        {
//////            return posevals;
//////        }
//////        
//////        if(posevals[1][1] < 0.1 * p3d.size())
//////        {
//////            posevals[0] = pguess;
//////        }
////    
////        RANSAC_MODEL = 1;
////        posevals = UseBAIterative(posevals[0], p3d, p2d1, inliers);
////    
////        if(posevals.size() > 0 and (posevals[1][1] > 0.5 * p3d.size() or posevals[1][1] > 15))
////        {
////            return posevals;
////        }
////    }
//    
//    RANSAC_MODEL = 0;
//    gtsam::Pose3 pose;
//    std::vector<double> posevals;
//    std::tie(pose, posevals) = UseBAIterative(pguess, p3d, p2d1, inliers);
//    
//    RANSAC_MODEL = 1;
//    if(posevals.size() > 0)
//    {
//        std::tie(pose, posevals) = UseBAIterative(pose, p3d, p2d1, inliers);
//    }
//    else
//    {
//        std::tie(pose, posevals) = UseBAIterative(pguess, p3d, p2d1, inliers);
//    }
//    
//    if(posevals.size() > 0 and (posevals[1] > 0.5 * p3d.size() or posevals[1] > 15))
//    {
//        return std::make_tuple(pose, posevals);
//    }
//    
//    return {};
//}
//
//void LocalizePose::testLocalizePoses()
//{
//    ParseOptimizationResults POR("/Volumes/Untitled/data/SingleSessionSLAM/", "140106");
//    std::default_random_engine re;
//    debug = true;
//    
////    int nsuc = 0;
//    std::vector<double> last;
//    for(int i=0; i<POR.boat.size(); ++i)
//    {
//        int ftfno = POR.ftfilenos[i];
//        ParseFeatureTrackFile PFTF(_cam, "/Volumes/Untitled/data/Lakeshore_KLT/140106", ftfno);
//        std::vector<gtsam::Point3> p3d = POR.GetSubsetOf3DPoints(PFTF.ids);
//        std::vector<gtsam::Point2>& p2d1 = PFTF.imagecoord;
//        
//        removeZeroPoints(p3d, p2d1);
//        std::vector<double> inliers(p3d.size(), 1.0);
//        
//        std::vector<double> estp(6);
//        std::vector<double>& optimized = POR.boat[i];
//        std::uniform_real_distribution<double> uniferr(0,0.5);
//        for(int j=0; j<6; ++j)
//        {
//            double err = uniferr(re);
//            estp[j] = optimized[j]+err;
//        }
//        
//        std::vector<std::vector<double>> posevals = combinedLocalizationMethod(estp, p3d, p2d1, inliers);
//        if(posevals.size() == 0)
//        {
//            std::cout << i << "] no result? " << " number of points: " << p3d.size() << std::endl;
//        }
//        else
//        {
//            gtsam::Pose3 resultpose = GTSAMInterface::VectorToPose(posevals[0]);
//            gtsam::Pose3 diffpose = resultpose.between(POR.CameraPose(i));
//            std::vector<double> p = GTSAMInterface::PoseToVector(diffpose);
//            
//            if(posevals[1][1] > 0.5 * p3d.size() or posevals[1][1] > 15)
//            {
//                std::cout << i << "] good localization : difference from expected ("<<p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<p[3]<<", "<<p[4]<<", "<<p[5]<<") " << std::endl;
//            }
//        }
//        
//        /*
//        RANSAC_MODEL = 1;
//        std::vector<double> inliers(p3d.size(), 1.0);
//        std::vector<std::vector<double>> posevals = UseBAIterative(estp, p3d, p2d1, inliers);
//
//        if(posevals.size() > 0)
//        {
//            gtsam::Pose3 resultpose = GTSAMInterface::VectorToPose(posevals[0]);
//            gtsam::Pose3 diffpose = resultpose.between(POR.CameraPose(i));
//            std::vector<double> p = GTSAMInterface::PoseToVector(diffpose);
//            
//            if(posevals[1][1] < 0.1 * p3d.size())
//            {
//                //std::cout << i << "] bad P3P localization : difference from expected ("<<p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<p[3]<<", "<<p[4]<<", "<<p[5]<<") " << std::endl;
//                posevals[0] = last;
//            }
//            
//            if(posevals[1][1] > 0.5 * p3d.size())
//            {
////                std::cout << i << "] good P3P localization : difference from expected ("<<p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<p[3]<<", "<<p[4]<<", "<<p[5]<<") " << std::endl;
//                nsuc++;
//                last = posevals[0];
////                return posevals[0];
//            }
//            else
//            {
////                std::cout << i << "] bad P3P localization : difference from expected ("<<p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<p[3]<<", "<<p[4]<<", "<<p[5]<<") " << std::endl;
//                
//                RANSAC_MODEL = 1;
//                inliers = std::vector<double>(p3d.size(), 1.0);
//                //std::cout << "sizes: " << posevals.size() << ", " << p3d.size() <<", " << p2d1.size() << std::endl;
//                posevals = UseBAIterative(posevals[0], p3d, p2d1, inliers);
//                
//                if(posevals.size() > 0)
//                {
//                    gtsam::Pose3 resultpose = GTSAMInterface::VectorToPose(posevals[0]);
//                    gtsam::Pose3 diffpose = resultpose.between(POR.CameraPose(i));
//                    std::vector<double> p = GTSAMInterface::PoseToVector(diffpose);
//                    
//                    if(posevals[1][1] > 0.5 * p3d.size() or posevals[1][1] > 15)
//                    {
//                        std::cout << i << "] good BA localization : difference from expected ("<<p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<p[3]<<", "<<p[4]<<", "<<p[5]<<") " << std::endl;
//                        nsuc++;
//                        last = posevals[0];
////                        return posevals[0];
//                    }
//                    else
//                    {
//                        std::cout << i << "] bad BA localization : difference from expected ("<<p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<p[3]<<", "<<p[4]<<", "<<p[5]<<") " << std::endl;
//                        //return {}
//                    }
//                }
//                else
//                {
//                    std::cout << i << "] no result? " << " number of points: " << p3d.size() <<  std::endl;
//                    //return {}
//                }
//            }
//        }
//        else
//        {
//            std::cout << i << "] no result? " << " number of points: " << p3d.size() << std::endl;
//            //return {}
//        }
//        */
//    }
//    
////    std::cout << "% success: " << 1.0 * nsuc / POR.boat.size() << ", " << nsuc << " of " << POR.boat.size() << std::endl;
//}

















