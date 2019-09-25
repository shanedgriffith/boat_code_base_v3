#include "ParseDroneRun.hpp"

#include <iostream>
#include <cmath>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

using namespace std;
using namespace gtsam;

void ParseDroneRun::ProcessLineEntries(int type, vector<string>& lp)
{
    if(lp[0].compare("#") == 0) return;
    ang_vel.push_back(gtsam::Vector3(stod(lp[2]), stod(lp[3]), stod(lp[4])));
    lin_acc.push_back(gtsam::Vector3(stod(lp[5]), stod(lp[6]), stod(lp[7]))); //  
    IMUtimestamps.push_back(stod(lp[1]));
}

std::vector<std::string> ParseDroneRun::ParseLine(char * line)
{
    return ParseLineAdv(line, " ");
}

void ParseDroneRun::ReadDelimitedFile(string file, int type)
{
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

void ParseDroneRun::loadGroundTruth()
{
    std::string file = _base + _date + "/groundtruth.txt";
    FILE * fp = OpenFile(file,"r",false);
    if(!fp)
    {
        std::cout << "No groundtruth file at: " << file << std::endl;
        return;
    }
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp))
    {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        if(lp[0].compare("#") == 0) continue;
        gtsam::Point3 T(stod(lp[2]), stod(lp[3]), stod(lp[4]));
        gtsam::Rot3 R(stod(lp[8]), stod(lp[5]), stod(lp[6]), stod(lp[7]));
        groundtruth.push_back(gtsam::Pose3(R, T));
        GTtimestamps.push_back(stod(lp[1]));
    }
    fclose(fp);
}

Camera ParseDroneRun::GetCamera()
{
    //TODO: what's the camera's calibration? Or calibrate it separately.
    Camera davis(2*174.23979032083346, 2*174.11105443010973, 163.91078563399876, 140.9726709818771, 346, 260);
    davis.SetDistortion(-0.03560363132286886, 0.001974723646350411, 0.0, 0.0, -0.0045671620060236855);
    return davis;
}

int ParseDroneRun::GetImageNumber(int auxidx)
{
    return auxidx;
}

int ParseDroneRun::GetIndexOfImage(int image)
{
    return image;
}

double ParseDroneRun::GetAvgAngularVelocity(int sidx, int eidx)
{
    return ang_vel[sidx].x();
}

std::vector<double> ParseDroneRun::GetDrawScale()
{
    return {-300,300,-300,300};
}

gtsam::Vector3 ParseDroneRun::computeBias(const std::vector<gtsam::Vector3>& reading, int s, int e)
{
    gtsam::Vector3 bias(0,0,0);
    for(int i=s; i<e; i++)
    {
        bias += reading[i];
    }
    bias /= (e-s);
    
    return bias;
    
    /*
    gtsam::Vector3 mean(0,0,0);
    int count=0;
    for(; IMUtimestamps[count] < movementStartTime(); count++)
    {
        mean += lin_acc[count];
    }
    
    mean /= count;
    
    gtsam::Vector3 stddev(0,0,0);
    for(int i=0; IMUtimestamps[i] < movementStartTime(); i++)
    {
        gtsam::Vector3 val = lin_acc[i] - mean;
        gtsam::Vector3 squared(val.x() * val.x(), val.y()*val.y(), val.z()*val.z());
        stddev += squared;
    }
    
    gtsam::Vector3 val = 1.0/(count-1) * stddev;
    stddev = gtsam::Vector3(pow(val.x(), 0.5), pow(val.y(), 0.5), pow(val.z(), 0.5));
    
    std::cout << "mean: " << mean.transpose() << ". std: " << stddev.transpose() << std::endl;
    std::cout << "gravity magnitude: " << mean.norm() << std::endl;
    
    exit(1);
    
    gravity = mean;
    lin_acc_error = stddev;
     */
}

boost::shared_ptr<PreintegratedCombinedMeasurements::Params>
ParseDroneRun::getIMUNoiseModel()
{
    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = 0.1;
    double gyro_noise_sigma = 0.05;
    double accel_bias_randomwwalk_sigma = 0.002;
    double gyro_bias_randomwwalk_sigma = 0.00004;
    Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    Matrix33 integration_error_cov = Matrix33::Identity(3,3) * 1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_randomwwalk_sigma,2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_randomwwalk_sigma,2);
    Matrix66 bias_acc_omega_int = Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration
    
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = boost::make_shared<PreintegratedCombinedMeasurements::Params>(Vector3(0, 0, 0));
//    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = boost::make_shared<PreintegratedCombinedMeasurements::Params>(-9.81); //TODO (?)
//    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.80655);
    // PreintegrationBase params:
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;
    
    return p;
}

gtsam::Pose3
ParseDroneRun::tfIMUtoCam()
{
//    gtsam::Vector3 trans(-0.0015224098391112568, -0.006621897399791399, -0.023154837302635834);
    gtsam::Vector3 trans(0, 0, 0);
    gtsam::Matrix3 rot;
    rot << 0.9998829655327196, 0.005335413966337045, -0.014338360969823338,
            -0.005432624310654592, 0.9999624656424586, -0.006749362884958196,
            0.014301812143655866, 0.00682646790524808, 0.9998744208676132;
    gtsam::Rot3 r(rot);
    
    return gtsam::Pose3(r, trans);
}

/*
 angle between two 3D vectors:
 https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
 */
gtsam::Pose3
ParseDroneRun::getInitialPose(int start)
{
    gtsam::Vector3 default_gravity(1.1842827,  -9.4668998,  -2.26797387);
    gtsam::Vector3 initial_reading = lin_acc[0];
    
    default_gravity.normalize();
    initial_reading.normalize();
    
    gtsam::Vector3 v = default_gravity.cross(initial_reading); //isn't ||v|| = 1?
    double angle = std::acos(default_gravity.dot(initial_reading) / (default_gravity.norm() * initial_reading.norm()));
    double c = default_gravity.dot(initial_reading) * cos(angle);
    
    gtsam::Matrix3 vx;
    vx << 0,   -v.z(),  v.y(),
          v.z(), 0,    -v.x(),
          -v.y(), v.x(),    0;
    
    gtsam::Matrix3 m2 =  vx * vx * 1.0/(1.0+c);
    gtsam::Matrix3 m1 = vx;
    gtsam::Matrix3 m0 = gtsam::Rot3::identity().matrix();
    gtsam::Matrix3 rot_mat = m0 + m1 + m2;
    
    gtsam::Rot3 res(rot_mat);
    
    std::cout << "result rotation: \n " << res << "\n as rpy: " << res.rpy().transpose() << std::endl;
    return gtsam::Pose3(res, gtsam::Point3::identity());
}

std::vector<bool>
ParseDroneRun::identifyMovingFrames()
{
    Camera davis = GetCamera();
    int zero = 0;
    ParseFeatureTrackFile last = LoadVisualFeatureTracks(davis, zero, false);
    std::vector<bool> moved(IMAGEtimestamps.size(), false);
    for(int i=1; i<IMAGEtimestamps.size(); i++)
    {
        ParseFeatureTrackFile next = LoadVisualFeatureTracks(davis, i, false);
        double mean_displacement = last.GetDisplacementFrom(next);
        if(mean_displacement < 0 or
           mean_displacement > 0.3)
        {
            moved[i] = true;
        }
        
        //std::cout << "displacement["<<i<<"] " << moved[i] << "   " << mean_displacement << std::endl;
        
        last = next;
    }
    return moved;
}








