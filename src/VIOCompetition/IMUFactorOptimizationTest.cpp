#include "IMUFactorOptimizationTest.hpp"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>


#ifdef GTSAM4

#include "GravityFactor.h"

using namespace std;
using namespace gtsam;


using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::A;
using symbol_shorthand::R;
using symbol_shorthand::S;

void
IMUFactorOptimizationTest::optimize()
{
    //std::cout << "size before optimization: ("<<initialEstimate.size() << ", " << result.size() <<")"<<std::endl;
    
    
    try
    {
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
        result = optimizer.optimize();
        initialEstimate = result;
    }
    catch(const std::exception& ex)
    {
        printf("There was an exception while attempting to solve the factor graph.");
        printf("Known causes of the exception:\n");
        printf(" >The camera pose used to create landmark observations was not initialized\n");
        printf(" >A prior was not specified for the first camera pose.\n");
        printf(" >landmarks need to be observed at least twice.");
        std::cout << ex.what() << std::endl;
        
        printf("\nThe factor graph.\n");
        graph.print();
        printf("\n\n\nThe initial estimates.\n");
        initialEstimate.print();
        printf("Terminated with an exception. ");
        std::cout << ex.what()<<std::endl;
        exit(-1);
    }
    
    
    
//     //Runs much slower than batch optimization on all the data. Why?
//    for(int i=0; i<1; i++) 
//     {
//        if(i==0) 
//     {
//            i2.update(graph, initialEstimate);
//        } 
//     else
//     {
//            i2.update();
//        }
//    }
//    
//    //isam2 holds its own copy of those values and the graph structure.
//    initialEstimate.clear();
//    graph.resize(0);
//    result = i2.calculateEstimate();
    
}

void
IMUFactorOptimizationTest::setup()
{
//    gtsam::Pose3 prior_pose(gtsam::Rot3::Ry(M_PI_2) * gtsam::Rot3::identity(), gtsam::Point3());//
    gtsam::Pose3 prior_pose = gtsam::Pose3(gtsam::Rot3(0.113215734831919, -0.109361391408676, 0.705885554328104, -0.690614123549854),
                                           gtsam::Point3(-1.487742644847689, -5.505546379457251, -0.772862460266841));
    
    gtsam::Vector3 prior_velocity = gtsam::Vector3::Identity();
    imuBias::ConstantBias prior_imu_bias(_pdr.acc_bias, _pdr.ang_bias);
    
    //identity prior is OK if the drone starts on the ground.
    // Add all prior factors (pose, velocity, bias) to the graph.
    graph.add(PriorFactor<Pose3>(X(0), prior_pose, pose_noise_model));
    graph.add(PriorFactor<Vector3>(V(0), prior_velocity, velocity_noise_model));
    graph.add(PriorFactor<imuBias::ConstantBias>(B(0), prior_imu_bias, bias_noise_model));
    
    initialEstimate.insert(X(0), prior_pose);
    initialEstimate.insert(V(0), prior_velocity);
    initialEstimate.insert(B(0), prior_imu_bias);
    
    // Store previous state for the imu integration and the latest predicted outcome.
    state = NavState(prior_pose, prior_velocity);
    bias = prior_imu_bias;
    
    imu_preintegrated_ = std::make_shared<PreintegratedImuMeasurements>(_pdr.getIMUNoiseModel(), prior_imu_bias);
}

void
IMUFactorOptimizationTest::updateIMU(int keyframe)
{
    // Overwrite the beginning of the preintegration for the next step.
    state = NavState(result.at<Pose3>(X(keyframe)),
                     result.at<Vector3>(V(keyframe)));
    bias = result.at<imuBias::ConstantBias>(B(0));
    
    // Reset the preintegration object.
//    imu_preintegrated_->resetIntegrationAndSetBias(bias);
    imu_preintegrated_->resetIntegration();
}


void
IMUFactorOptimizationTest::addIMUFactor(int keyframe)
{
    std::shared_ptr<PreintegratedImuMeasurements> preint_imu = std::dynamic_pointer_cast<PreintegratedImuMeasurements>(imu_preintegrated_);
    
    ImuFactor imu_factor(X(keyframe-1), V(keyframe-1),
                         X(keyframe  ), V(keyframe  ),
                         B(0),
                         *preint_imu);
    
    graph.add(imu_factor);
    
    // Now optimize and compare results.
    state = imu_preintegrated_->predict(state, bias);
    initialEstimate.insert(X(keyframe), state.pose());
    initialEstimate.insert(V(keyframe), state.v());
}

void
IMUFactorOptimizationTest::AddLandmarkTrack(gtsam::Cal3_S2::shared_ptr k, LandmarkTrack& landmark)
{
    /*Add the landmark track to the graph.*/
    
    const int ldist = 100; //this threshold specifies the distance between the camera and the landmark.
    const int onoise = 10; //the threshold specifies at what point factors are discarded due to reprojection error.
    
    //GTSAM 4.0
    //landmarkDistanceThreshold - if the landmark is triangulated at a distance larger than that the factor is considered degenerate
    //dynamicOutlierRejectionThreshold - if this is nonnegative the factor will check if the average reprojection error is smaller than this threshold after triangulation,
    //  and the factor is disregarded if the error is large
    gtsam::SmartProjectionParams params;
    params.setLandmarkDistanceThreshold(ldist);
    params.setDynamicOutlierRejectionThreshold(onoise);
    
    gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> sppf(pixelNoise, k, boost::none, params);
    
    for(int i=0; i<landmark.points.size(); i++)
    {
        sppf.add(landmark.points[i], landmark.camera_keys[i]); //GTSAM 4.0
    }
    
    landmark_factors.push_back(sppf);
    landmark_keys.push_back(landmark.key);
    if(landmark.used)
    {
        graph.add(sppf);
    }
}

void
IMUFactorOptimizationTest::AddLandmarkTracks(std::vector<LandmarkTrack>& landmarks)
{
    for(int i=0; i<landmarks.size(); i++)
    {
        AddLandmarkTrack(cam.GetGTSAMCam(), landmarks[i]);
    }
}

void
IMUFactorOptimizationTest::printPose(int i, double time, gtsam::Pose3& pred)
{
    printf("%d %lf %lf %lf %lf %lf %lf %lf %lf\n", i, time, pred.translation().x(), pred.translation().y(), pred.translation().z(), pred.rotation().ypr().x(), pred.rotation().ypr().y(), pred.rotation().ypr().z());
}

void
IMUFactorOptimizationTest::optimizeDroneRun()
{
    std::vector<bool> moved = _pdr.identifyMovingFrames();
    gtsam::Pose3 zero_pose = gtsam::Pose3::identity();
    gtsam::Vector3 prior_velocity = gtsam::Vector3::Identity();
    double tight = 0.00001;
    gtsam::noiseModel::Diagonal::shared_ptr tight_vector_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << tight, tight, tight).finished());
    std::vector<LandmarkTrack> active;
    
    setup();
    
//    gtsam::Pose3 initial = _pdr.getInitialPose(0);
//    gtsam::Pose3 offset = _pdr.tfIMUtoCam();
//    printPose(0, 0, initial);
//    printPose(0, 0, offset);
    
    gtsam::Rot3 roffset = _pdr.tfIMUtoCam().rotation();
    gtsam::Vector3 g0 = _pdr.lin_acc[0];//(_pdr.groundtruth[0].rotation() * roffset * _pdr.lin_acc[0]);
    
    int keyframe = 1;
    int start = 0;
    
    while(_pdr.IMAGEtimestamps[++start] < 1540113195.794294595718);
    int t = 0;
    
    gtsam::Vector3 gz(0,0,-9.81); //gravity specified in the camera frame first, then rotated into the IMU frame.
//    gtsam::Vector3 gb(0,-9.81,0);
//    gtsam::Vector3 gc(-9.81,0,0);
//    gtsam::Vector3 gd(0,0,9.81);
//    gtsam::Vector3 ge(0,9.81,0);
//    gtsam::Vector3 gf(9.81,0,0);
    
    bool stationary = true;
    int mc = 0;
//    std::cout << "Added image ";
    for(int i=start; i<_pdr.IMUtimestamps.size(); i++)
    {
        if(_pdr.IMUtimestamps[i] < 1540113195.794294595718)
            continue;
        
        double dt = ( i > 0 ) ? _pdr.IMUtimestamps[i] - _pdr.IMUtimestamps[i-1] : 0.001;
        
        NavState curstate = imu_preintegrated_->predict(state, bias);
        gtsam::Pose3 pred = curstate.pose();
        printPose(i, _pdr.IMUtimestamps[i], pred);
        
        while(_pdr.GTtimestamps[++t] < _pdr.IMUtimestamps[i]);
        printPose(t, _pdr.GTtimestamps[t], _pdr.groundtruth[t]);
        
        //TODO: update this so it doesn't use the groundtruth.
        
        gtsam::Vector3 res =  (gtsam::Rot3::Rx(M_PI_2) * _pdr.groundtruth[t].rotation() * roffset) * (_pdr.lin_acc[i] - g0);// // _pdr.groundtruth[t].rotation() * _pdr.gravity; //should this be the current gravity vector?
//        std::cout << "vec1: " << (_pdr.lin_acc[i] - _pdr.groundtruth[t].rotation() * _pdr.gravity).transpose() << std::endl;
//        std::cout << "vec2: " << (_pdr.lin_acc[i] - _pdr.groundtruth[t].rotation().transpose() * _pdr.gravity).transpose() << std::endl;
//        std::cout << "original: " << (_pdr.lin_acc[i]).transpose() << std::endl;
//        std::cout << "gravity: " << (_pdr.gravity).transpose() << std::endl;
//        std::cout << "rotated: " << (_pdr.groundtruth[t].rotation() * ga).transpose() << std::endl;
//        std::cout << "rotated: " << (_pdr.groundtruth[t].rotation() * gb).transpose() << std::endl;
//        std::cout << "rotated: " << (_pdr.groundtruth[t].rotation() * gc).transpose() << std::endl;
//        std::cout << "rotated: " << (_pdr.groundtruth[t].rotation() * gd).transpose() << std::endl;
//        std::cout << "rotated: " << (_pdr.groundtruth[t].rotation() * ge).transpose() << std::endl;
//        std::cout << "rotated: " << (_pdr.groundtruth[t].rotation() * gf).transpose() << std::endl;
//        std::cout << "rotated: " << (_pdr.groundtruth[t].rotation() * roffset * _pdr.lin_acc[i]).transpose() << std::endl;
//        std::cout << "subtracted: " << ((_pdr.groundtruth[t].rotation() * roffset) * (_pdr.lin_acc[i] - g0)).transpose() << std::endl;
        gtsam::Pose3 test;
        imu_preintegrated_->integrateMeasurement(res, _pdr.ang_vel[i], dt);
        
//        std::cout << "vec["<<i<<"]: " << res.transpose() << std::endl;
        
        if(_pdr.IMUtimestamps[i] > _pdr.IMAGEtimestamps[keyframe + start] + _pdr.timeshift_cam_imu)
        {
            if(keyframe%10==0)
                std::cout << start + keyframe << " ";
//            ParseFeatureTrackFile PFT = _pdr.LoadVisualFeatureTracks(cam, keyframe, false);
//            
//            std::vector<LandmarkTrack> inactive = PFT.ProcessNewPoints('x', keyframe, active, 100);
//            AddLandmarkTracks(inactive);
            
            addIMUFactor(keyframe);
            
            if(not moved[keyframe-1+start])
            {
                //acceleration = 0.
                graph.add(PriorFactor<gtsam::Vector3>(V(keyframe), prior_velocity, tight_vector_noise));
                graph.add(BetweenFactor<gtsam::Pose3>(X(keyframe), X(keyframe-1), zero_pose, bias_between_noise));
            }
            else mc++;
            
            optimize();
            
            std::cout << "predicted state  " << state.pose().translation() << "  " << "  " << state.pose().rotation().ypr().transpose() << std::endl;
            
            updateIMU(keyframe);
            
            if(++keyframe >= _pdr.IMAGEtimestamps.size())
            {
                break;
            }
        }
        
        if(_pdr.IMUtimestamps[i] > 1540113197.381905555725)
            break;
    }
    std::cout << std::endl;
    
    AddLandmarkTracks(active);
    
    optimize();
    
    printResults(keyframe, start, mc, moved);
}

void
IMUFactorOptimizationTest::printResults(int keyframe, int start, int mc, std::vector<bool>& moved)
{
    std::cout << std::fixed;
    std::cout << std::setprecision(7);
    
    imuBias::ConstantBias imubias = result.at<imuBias::ConstantBias>(B(0));
    std::vector<double> timestamps;
    std::vector<gtsam::Pose3> traj;
    for(int i=0; i<keyframe-1; i++)
    {
        gtsam::Pose3 posei = _pdr.tfIMUtoCam() * result.at<gtsam::Pose3>(X(i));
//        posei = gtsam::Pose3(gtsam::Rot3::Rx(M_PI_2) * posei.rotation(), posei.translation());
        std::cout << start + i << "  " << moved[i+start] << "  " << _pdr.IMAGEtimestamps[i+start] + _pdr.timeshift_cam_imu << "  " << posei.translation() << "  " << "  " << posei.rotation().ypr().transpose() << std::endl;
        traj.push_back(posei);
        timestamps.push_back(_pdr.IMAGEtimestamps[i+start] + _pdr.timeshift_cam_imu);
    }
    std::cout << "bias  " << imubias.accelerometer().transpose() << "  " << imubias.gyroscope().transpose() << std::endl;
    std::cout << "moving keyframes " << mc << std::endl;
    
    saveResults(timestamps, traj);
}

void
IMUFactorOptimizationTest::saveResults(std::vector<double>& timestamps, std::vector<gtsam::Pose3>& traj)
{
    std::string filename = "/Users/shane/Documents/projects/VIO/experiments/traj.txt";
    FILE * fp = FileParsing::OpenFile(filename, "w");
    fprintf(fp, "# id timestamp tx ty tz qx qy qz qw\n");
    for(int i=0; i<traj.size(); i++)
    {
        gtsam::Vector q = traj[i].rotation().quaternion();
        
        std::string line =
        to_string(traj[i].translation().x()) + " " +
        to_string(traj[i].translation().y()) + " " +
        to_string(traj[i].translation().z()) + " " +
        to_string(q(1)) + " " +
        to_string(q(2)) + " " +
        to_string(q(3)) + " " +
        to_string(q(0));
        fprintf(fp, "%d %lf %s\n", i, timestamps[i], line.c_str());
    }
    fclose(fp);
}


void
IMUFactorOptimizationTest::solveForIMU()
{
    /*std::vector<bool> moved = _pdr.identifyMovingFrames();
    gtsam::Vector3 initial_acc(0,0,0);
    gtsam::Vector3 initial_bias_acc = pdr.acc_bias;
    gtsam::Rot3 initial_orientation;
    gtsam::Rot3 initial_bias_orient = gtsam::Rot3::Ypr(pdr.ang_bias.z(), pdr.ang_bias.y(), pdr.ang_bias.x());
    
    double tight = 0.00001;
    gtsam::noiseModel::Diagonal::shared_ptr tight_vector_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << tight, tight, tight).finished());
    double loose = 1.0;
    gtsam::noiseModel::Diagonal::shared_ptr loose_vector_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << loose, loose, loose).finished());
    
    initialEstimate.insert(A(0), initial_acc);          //acc
    initialEstimate.insert(B(0), initial_bias_acc);     //bias_acc
    initialEstimate.insert(R(0), initial_orientation);  //orientation
    
    graph.add(PriorFactor<gtsam::Vector3>(A(0), initial_acc, tight_vector_noise));
    graph.add(PriorFactor<gtsam::Vector3>(B(0), initial_bias_acc, loose_vector_noise));
    graph.add(PriorFactor<gtsam::Rot3>(R(0), initial_orientation, loose_vector_noise));
    
    gtsam::Rot3 cur_orientation;
    gtsam::Vector3 cur_acceleration;
    int keyframe = 1;
    for(int i=0; i<pdr.IMUtimestamps.size(); i++)
    {
        if(keyframe < pdr.IMAGEtimestamps.size() and
           pdr.IMUtimestamps[i] > pdr.IMAGEtimestamps[keyframe] + pdr.timeshift_cam_imu)
        {
            ++keyframe;
        }
        
        double dt = ( i > 0 ) ? pdr.IMUtimestamps[i] - pdr.IMUtimestamps[i-1] : 0.001;
        imu_preintegrated_->integrateMeasurement(lin_acc[i], ang_vel[i], dt);
        
        graph.add(GravityFactor(A(i), B(0), R(i), pdr.lin_acc[i], tight_vector_noise));
        if(i>0)
        {
            gtsam::Vector3 eulerangles = cur_orientation.rpy();
            eulerangles += pdr.ang_vel[i] * dt;
            gtsam::Rot3 curorient = gtsam::Rot3::RzRyRx(eulerangles);
            
            graph.add(PriorFactor<gtsam::Rot3>(R(i), curorient, tight_vector_noise));
            initialEstimate.insert(R(i), curorient);
            
            initialEstimate.insert(A(i), cur_acceleration);
            
            if(not moved[keyframe-1])
            {
                //acceleration = 0.
                graph.add(PriorFactor<gtsam::Vector3>(A(i), initial_acc, tight_vector_noise));
            }
        }
        
        optimize();
        
        cur_orientation = result.at<gtsam::Rot3>(R(i));
        cur_acceleration = result.at<gtsam::Vector3>(A(i));
    }
    
    gtsam::Vector3 bias = result.at<gtsam::Vector3>(B(0));
    std::cout << "bias: " << bias << std::endl;
    
//    for(int i=0; i<pdr.IMUtimestamps.size(); i++)
//    {
//        
//    }
     */
}

void
IMUFactorOptimizationTest::testIMUFactor(std::vector<gtsam::Vector3> lin_acc, std::vector<gtsam::Vector3> ang_vel, std::vector<double> timestamps, boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p)
{
    gtsam::Pose3 prior_pose = gtsam::Pose3::identity();
    gtsam::Vector3 prior_velocity = gtsam::Vector3::Identity();
    gtsam::Vector3 prior_acceleration = gtsam::Vector3::Identity();
    imuBias::ConstantBias prior_imu_bias;
    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    
    //identity prior is OK if the drone starts on the ground.
    // Add all prior factors (pose, velocity, bias) to the graph.
    graph.add(PriorFactor<Pose3>(X(0), prior_pose, pose_noise_model));
    graph.add(PriorFactor<Vector3>(V(0), prior_velocity, velocity_noise_model));
    graph.add(PriorFactor<imuBias::ConstantBias>(B(0), prior_imu_bias, bias_noise_model));
    
    initialEstimate.insert(X(0), prior_pose);
    initialEstimate.insert(V(0), prior_velocity);
    initialEstimate.insert(B(0), prior_imu_bias);
    initialEstimate.insert(A(0), prior_acceleration);
    
    // Store previous state for the imu integration and the latest predicted outcome.
    NavState prev_state(prior_pose, prior_velocity);
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = prior_imu_bias;
    
    std::shared_ptr<PreintegrationType> imu_preintegrated_ = std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);
    
    int correction_count = 0;
    double pose_interval_time = 0.05;
    double time0 = timestamps[0];
    double last_added_time = time0 - pose_interval_time;
    Vector3 last_position;
    
    std::cout << std::fixed;
    std::cout << std::setprecision(7);
    
    for(int i=0; i<timestamps.size(); i++)
    {
        double dt = ( i > 0 ) ? timestamps[i] - timestamps[i-1] : 0.001;
        imu_preintegrated_->integrateMeasurement(lin_acc[i], ang_vel[i], dt);
        
        if(timestamps[i] - last_added_time >= pose_interval_time)
        {
            correction_count++;
            
            std::shared_ptr<PreintegratedImuMeasurements> preint_imu = std::dynamic_pointer_cast<PreintegratedImuMeasurements>(imu_preintegrated_);
            
            ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                                 X(correction_count  ), V(correction_count  ),
                                 B(correction_count-1),
                                 *preint_imu);
            
            graph.add(imu_factor);
            graph.add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1),
                                                           B(correction_count  ),
                                                           zero_bias, bias_between_noise));
            
            // Now optimize and compare results.
            prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
            initialEstimate.insert(X(correction_count), prop_state.pose());
            initialEstimate.insert(V(correction_count), prop_state.v());
            initialEstimate.insert(B(correction_count), prev_bias);
            
            optimize();
            
            // Overwrite the beginning of the preintegration for the next step.
            prev_state = NavState(result.at<Pose3>(X(correction_count)),
                                  result.at<Vector3>(V(correction_count)));
            prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));
            
            // Reset the preintegration object.
            imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
            
            
            // Get the position and the orientation estimates.
            Vector3 gtsam_position = prev_state.pose().translation();
            Vector4 gtsam_quat = prev_state.pose().rotation().quaternion();
            
            std::cout << "[" << timestamps[i] << "] " << gtsam_position.transpose() << " " << gtsam_quat.transpose() << " speed: " << (gtsam_position-last_position).norm() / ( timestamps[i] - last_added_time ) << std::endl;
            
            last_position = gtsam_position;
            last_added_time = timestamps[i];
        }
        
        //        if(timestamps[i] - timestamps[0] > 30)
        //        {
        //            break;
        //        }
    }
}

#endif
