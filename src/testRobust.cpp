#include "testRobust.hpp"

//#include <gtsam/slam/dataset.h>

#include <fstream>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

#include <gtsam/slam/expressions.h>

using namespace std;
using namespace gtsam;
namespace NM = gtsam::noiseModel;

// data available at http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/
// Datafile format (from http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/log.html)

// load the odometry
// DR: Odometry Input (delta distance traveled and delta heading change)
//    Time (sec)  Delta Dist. Trav. (m) Delta Heading (rad)


list<testRobust::TimedOdometry>
testRobust::
readOdometry()
{
    list<TimedOdometry> odometryList;
    string data_file = "/users/shane/git_repos/gtsam-4.0.2/examples/Data/Plaza2_DR.txt";
    ifstream is(data_file.c_str());
    
    while (is) {
        double t, distance_traveled, delta_heading;
        is >> t >> distance_traveled >> delta_heading;
        odometryList.push_back(TimedOdometry(t, Pose2(distance_traveled, 0, delta_heading)));
    }
    is.clear(); /* clears the end-of-file and error flags */
    return odometryList;
}

// load the ranges from TD
//    Time (sec)  Sender / Antenna ID Receiver Node ID  Range (m)



vector<testRobust::RangeTriple>
testRobust::
readTriples()
{
    vector<RangeTriple> triples;
    string data_file = "/users/shane/git_repos/gtsam-4.0.2/examples/Data/Plaza2_TD.txt";
    ifstream is(data_file.c_str());
    
    while (is) {
        double t, sender, range;
        size_t receiver;
        is >> t >> sender >> receiver >> range;
        triples.push_back(RangeTriple(t, receiver, range));
    }
    is.clear(); /* clears the end-of-file and error flags */
    return triples;
}

vector<tuple<int, Point2, int, Point2>>
testRobust::
defineMatches()
{
    vector<tuple<int, Point2, int, Point2>> res =
    {
        {0, gtsam::Point2(19,332), 0, gtsam::Point2(118,602)},
        {0, gtsam::Point2(85,340), 0, gtsam::Point2(176,642)},
        {0, gtsam::Point2(158,394), 0, gtsam::Point2(436,650)},
        {0, gtsam::Point2(198,126), 0, gtsam::Point2(367,475)},
        {0, gtsam::Point2(260,178), 0, gtsam::Point2(406,553)},
        {0, gtsam::Point2(228,76), 0, gtsam::Point2(416,438)},
        {0, gtsam::Point2(258,113), 0, gtsam::Point2(429,488)},
        {0, gtsam::Point2(309,140), 0, gtsam::Point2(449,693)},
        {0, gtsam::Point2(350,279), 0, gtsam::Point2(454,690)},
        {0, gtsam::Point2(349,235), 0, gtsam::Point2(471,647)},
        {0, gtsam::Point2(376,202), 0, gtsam::Point2(442,510)},
    };
    
    return res;
    //tuple<int, Point2, int, Point2>
}

void
testRobust::
image()
{
    /*IMPORTANT: Robust loss functions need an initialization with many points inside the convergence basin, otherwise all the points are outside that and optimization stops without doing anything.
     */
    
    // This is the value we wish to estimate
    gtsam::Pose2_ pose_expr(0);
    
    // Set up initial values, and Factor Graph
    gtsam::Values initial;
    gtsam::ExpressionFactorGraph graph;
    
    // provide an initial estimate which is pretty much random
    initial.insert(0, Pose2(1, 1, 0.01));
    
    // We assume the same noise model for all points (since it is the same camera)
    auto measurementNoise = noiseModel::Isotropic::Sigma(2, 10.0);
    
    /********* First change *********/
    // We define our robust error model here, providing the default parameter value for the estimator.
    auto huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(10), measurementNoise);
    
    std::vector<std::tuple<int, Point2, int, Point2>> matches = defineMatches();
    
    // Now we add in the factors for the measurement matches.
    // Matches is a vector of 4 tuples (index1, keypoint1, index2, keypoint2)
    int index_i, index_j;
    Point2 p, measurement;
    for (vector<tuple<int, Point2, int, Point2>>::iterator it = matches.begin(); it != matches.end(); ++it) {
        
        std::tie(index_i, measurement, index_j, p) = *it;
        
        gtsam::Point2_ predicted = transformTo(pose_expr, p);
        
        // Add the Point2 expression variable, an initial estimate, and the measurement noise.
        // The graph takes in factors with the robust error model.
        /********* Second change *********/
        graph.addExpressionFactor<gtsam::Point2>(predicted, measurement, huber);
    }
    
    // Optimize and print basic result
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final Result:\n");
    
    auto geman = noiseModel::Robust::Create(noiseModel::mEstimator::GemanMcClure::Create(3), measurementNoise);
    graph.resize(0);
    
    for (vector<tuple<int, Point2, int, Point2>>::iterator it = matches.begin(); it != matches.end(); ++it)
    {
        std::tie(index_i, measurement, index_j, p) = *it;
        gtsam::Point2_ predicted = transformTo(pose_expr, p);
        graph.addExpressionFactor<gtsam::Point2>(predicted, measurement, geman);
    }
    
    result = gtsam::LevenbergMarquardtOptimizer(graph, result).optimize();
    result.print("Final Result:\n");
    
    
    geman = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), measurementNoise);
    graph.resize(0);
    
    for (vector<tuple<int, Point2, int, Point2>>::iterator it = matches.begin(); it != matches.end(); ++it)
    {
        std::tie(index_i, measurement, index_j, p) = *it;
        gtsam::Point2_ predicted = transformTo(pose_expr, p);
        graph.addExpressionFactor<gtsam::Point2>(predicted, measurement, geman);
    }
    
    result = gtsam::LevenbergMarquardtOptimizer(graph, result).optimize();
    result.print("Final Result:\n");
}

testRobust::
testRobust(){}

void
testRobust::
range()
{
    
    // load Plaza2 data
    list<TimedOdometry> odometry = readOdometry();
    //  size_t M = odometry.size();
    
    vector<RangeTriple> triples = readTriples();
    size_t K = triples.size();
    
    // parameters
    size_t minK = 150; // minimum number of range measurements to process initially
    size_t incK = 25; // minimum number of range measurements to process after
    bool groundTruth = false;
    bool robust = true;
    
    // Set Noise parameters
    Vector priorSigmas = Vector3(1,1,M_PI);
    Vector odoSigmas = Vector3(0.05, 0.01, 0.1);
    double sigmaR = 100; // range standard deviation
    const NM::Base::shared_ptr // all same type
    priorNoise = NM::Diagonal::Sigmas(priorSigmas), //prior
    odoNoise = NM::Diagonal::Sigmas(odoSigmas), // odometry
    gaussian = NM::Isotropic::Sigma(1, sigmaR), // non-robust
    tukey = NM::Robust::Create(NM::mEstimator::Tukey::Create(15), gaussian), //robust
    rangeNoise = robust ? tukey : gaussian;
    
    // Initialize iSAM
    ISAM2 isam;
    
    // Add prior on first pose
    Pose2 pose0 = Pose2(-34.2086489999201, 45.3007639991120,
                        M_PI - 2.02108900000000);
    NonlinearFactorGraph newFactors;
    newFactors.push_back(PriorFactor<Pose2>(0, pose0, priorNoise));
    Values initial;
    initial.insert(0, pose0);
    
    //  initialize points
    if (groundTruth) { // from TL file
        initial.insert(symbol('L', 1), Point2(-68.9265, 18.3778));
        initial.insert(symbol('L', 6), Point2(-37.5805, 69.2278));
        initial.insert(symbol('L', 0), Point2(-33.6205, 26.9678));
        initial.insert(symbol('L', 5), Point2(1.7095, -5.8122));
    } else { // drawn from sigma=1 Gaussian in matlab version
        initial.insert(symbol('L', 1), Point2(3.5784, 2.76944));
        initial.insert(symbol('L', 6), Point2(-1.34989, 3.03492));
        initial.insert(symbol('L', 0), Point2(0.725404, -0.0630549));
        initial.insert(symbol('L', 5), Point2(0.714743, -0.204966));
    }
    
    // set some loop variables
    size_t i = 1; // step counter
    size_t k = 0; // range measurement counter
    bool initialized = false;
    Pose2 lastPose = pose0;
    size_t countK = 0;
    
    
    
    // Loop over odometry
    gttic_(iSAM);
    for(const TimedOdometry& timedOdometry: odometry) {
        //--------------------------------- odometry loop -----------------------------------------
        double t;
        Pose2 odometry;
        boost::tie(t, odometry) = timedOdometry;
        
        // add odometry factor
        newFactors.push_back(BetweenFactor<Pose2>(i-1, i, odometry, odoNoise));
        
        // predict pose and add as initial estimate
        Pose2 predictedPose = lastPose.compose(odometry);
        lastPose = predictedPose;
        initial.insert(i, predictedPose);
        
        // Check if there are range factors to be added
        while (k < K && t >= boost::get<0>(triples[k])) {
            size_t j = boost::get<1>(triples[k]);
            double range = boost::get<2>(triples[k]);
            k = k + 1;
            if(j>= 2 or j <=4) continue;
            newFactors.push_back(RangeFactor<Pose2, Point2>(i, symbol('L', j), range,rangeNoise));
            countK = countK + 1;
        }
        
//        newFactors.print();
//        initial.print();
        
        // Check whether to update iSAM 2
        if ((k > minK) && (countK > incK)) {
            if (!initialized) { // Do a full optimize for first minK ranges
                gttic_(batchInitialization);
                LevenbergMarquardtOptimizer batchOptimizer(newFactors, initial);
                initial = batchOptimizer.optimize();
                gttoc_(batchInitialization);
                initialized = true;
            }
            gttic_(update);
            isam.update(newFactors, initial);
            gttoc_(update);
            gttic_(calculateEstimate);
            Values result = isam.calculateEstimate();
            gttoc_(calculateEstimate);
            lastPose = result.at<Pose2>(i);
            newFactors = NonlinearFactorGraph();
            initial = Values();
            countK = 0;
        }
        i += 1;
        //--------------------------------- odometry loop -----------------------------------------
    } // end for
    gttoc_(iSAM);
    
    // Print timings
    tictoc_print_();
    
    exit(0);
}
