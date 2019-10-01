#pragma once

#include "ParseDroneRun.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <DataTypes/Camera.hpp>
#include <DataTypes/LandmarkTrack.h>

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

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#ifdef GTSAM4

class IMUFactorOptimizationTest
{
private:
    gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model;
    gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model;
    gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model;
    gtsam::noiseModel::Diagonal::shared_ptr bias_between_noise;
    gtsam::noiseModel::Isotropic::shared_ptr pixelNoise;
    
    gtsam::ISAM2 i2;
    
    std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> > landmark_factors; //used by the GTSAMInterface // GTSAM 4.0

    std::vector<int> landmark_keys; //used by the GTSAMInterface
    
    gtsam::NonlinearFactorGraph graph;
    
    gtsam::Values initialEstimate;
    gtsam::Values result;
    
    gtsam::NavState state;
    gtsam::imuBias::ConstantBias bias;
    std::shared_ptr<gtsam::PreintegrationType> imu_preintegrated_;
    
    void optimize();
    void setup();
    void updateIMU(int keyframe);
    void addIMUFactor(int keyframe);
    
    void printPose(int i, double time, gtsam::Pose3& pred);
    
    void AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, LandmarkTrack& landmark);
    void AddLandmarkTracks(std::vector<LandmarkTrack>& landmarks);
    
//    void reprojectionError();
    
    void printResults(int keyframe, int start, int mc, std::vector<bool>& moved);
    
    void saveResults(std::vector<double>& timestamps, std::vector<gtsam::Pose3>& traj);
    
    Camera cam;
    
    ParseDroneRun& _pdr;
    
public:
    
    IMUFactorOptimizationTest(ParseDroneRun& pdr):
    cam(ParseDroneRun::GetCamera()), _pdr(pdr)
    {
        pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.001, 0.001, 0.001, 1.0, 1.0, 1.0).finished());
//        pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
        velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1); // m/s
        bias_noise_model = gtsam::noiseModel::Isotropic::Sigmas((gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished());
//        bias_noise_model = gtsam::noiseModel::Isotropic::Sigmas((gtsam::Vector(6) << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001).finished());
        bias_between_noise = gtsam::noiseModel::Isotropic::Sigmas((gtsam::Vector(6) << 0.001, 0.001, 0.001, 0.00001, 0.00001, 0.00001).finished());
        pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2.0, 3.0);
        
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 1.0;
        parameters.relinearizeSkip = 1;
        parameters.cacheLinearizedFactors = true;
        parameters.enableDetailedResults = false;
        i2 = gtsam::ISAM2(parameters);
    }
    
    void testIMUFactor(std::vector<gtsam::Vector3> lin_acc, std::vector<gtsam::Vector3> ang_vel, std::vector<double> timestamps, boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p);
    
    void optimizeDroneRun();
    
    void solveForIMU();
    
};

#endif











































































































