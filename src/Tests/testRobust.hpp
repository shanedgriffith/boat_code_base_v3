#pragma once


#include <stdio.h>
#include <list>
#include <iostream>
#include <vector>
#include <string>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>    //calibration and performs projections
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h> //for calculating the marginal covariance of the desired variables.
#include <gtsam/nonlinear/Values.h>//the initial guess of each thing (be it a pose3, point3, point3, whatever).
#include <gtsam/nonlinear/Symbol.h>//using symbols to identify factors
#include <gtsam/base/debug.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>  //used for visual slam. this is one nonlinear optimizer.
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>

class testRobust{
private:
    
    typedef std::pair<double, gtsam::Pose2> TimedOdometry;
    
    std::list<TimedOdometry>
    readOdometry();
    
    
    typedef boost::tuple<double, size_t, double> RangeTriple;
    
    std::vector<RangeTriple>
    readTriples();
    
    std::vector<std::tuple<int, gtsam::Point2, int, gtsam::Point2>>
    defineMatches();
    
public:
    
    testRobust();
    
    void range();
    
    void image();
};

