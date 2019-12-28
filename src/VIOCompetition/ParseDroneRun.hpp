#pragma once


#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/base/Vector.h>
#include <FileParsing/ParseSurvey.h>
#include <DataTypes/Camera.hpp>
#include "PreprocessDroneRun.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>


class ParseDroneRun : public ParseSurvey
{
private:

    std::vector<std::string> ParseLine(char * line);
    
    gtsam::Vector3 computeBias(const std::vector<gtsam::Vector3>& reading, int s, int e);
    
//    void computeGravity();
    
    void loadGroundTruth();
    
protected:
    
    void ProcessLineEntries(int type, std::vector<std::string>& lp);
    void ReadDelimitedFile(std::string file, int type);
    
public:
    
    //IMU
    std::vector<gtsam::Vector3> ang_vel;
    std::vector<gtsam::Vector3> lin_acc;
    std::vector<double> IMUtimestamps;
    std::vector<double> IMAGEtimestamps;
    gtsam::Vector3 acc_bias;
    gtsam::Vector3 ang_bias;
    
    std::vector<gtsam::Pose3> groundtruth;
    std::vector<double> GTtimestamps;
    
    const double timeshift_cam_imu = 0.005808240975061038;
    
    gtsam::Vector3 gravity;
    gtsam::Vector3 lin_acc_error;
    
    ParseDroneRun(std::string database, std::string name):
    ParseSurvey(database, database, name)
    {
        std::string imu_file = _base + name + "/imu.txt";
        ReadDelimitedFile(imu_file, 0);
        PreprocessDroneRun pdr(database, name);
        IMAGEtimestamps = pdr.timestamps;
        loadGroundTruth();
        
        //gravity vector in IMU coords? (when the camera is in the forward-facing orientation?)
        gravity = gtsam::Vector3(1.1842827,  -9.4668998,  -2.26797387); //gtsam::Rot3::Ry(-M_PI_2) *
        
        //transform IMU to target frame: gtsam::Rot3::Ry(M_PI_2)
        
        acc_bias = computeBias(lin_acc, 0, 200); // TODO.
        ang_bias = computeBias(ang_vel, 0, 200);
        
        acc_bias = acc_bias - gravity; //this produces a bias that's nearly what's solved for
        std::cout << "acc bias: " << acc_bias.transpose() << " norm: " << acc_bias.norm() << std::endl;
        std::cout << "ang bias: " << ang_bias.transpose() << " norm: " << ang_bias.norm() << std::endl;
        
//        acc_bias = test; //gtsam::Vector3(1.2765936, -0.0148882, 0.2851399);
    }
    
    static Camera GetCamera();
    
//    double movementStartTime();
    int GetImageNumber(int auxidx);
    int GetIndexOfImage(int image);
    double GetAvgAngularVelocity(int sidx, int eidx);
    std::vector<double> GetDrawScale();
    std::vector<bool> identifyMovingFrames();
    
    gtsam::Pose3 getInitialPose(int start);
    
    gtsam::Pose3 tfIMUtoCam();
    
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> getIMUNoiseModel();
};


































