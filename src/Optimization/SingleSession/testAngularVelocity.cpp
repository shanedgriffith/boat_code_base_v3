#include "testAngularVelocity.h"

#include "FileParsing/ParseOptimizationResults.h"

#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>


void
testAngularVelocity::
compareAngularVelocityToTraj()
{
    std::string survey = "140106_better_z_values_plus_loop_closure (closer to ground truth)";
    ParseOptimizationResults POR("/Volumes/Untitled/data/iSAM/", survey);
    ParseBoatSurvey PS("/Volumes/Untitled/data/VBags/", "/Volumes/Untitled/data/Lakeshore_KLT/", "140106");
    
    gtsam::Pose3 b_tm1 = POR.CameraPose(0);
    double t_m1 = POR.timestamp(0);
    double sum_ratio = 0;
    int increment = 5;
    for(int i=increment; i<POR.GetNumberOfPoses(); i=i+increment)
    {
        gtsam::Pose3 b_t = POR.CameraPose(i);
        gtsam::Pose3 diff = b_tm1.between(b_t);
        double t = POR.timestamp(i);
        
        int idx_t = PS.timestampToIndex(t);
        int idx_m1 = PS.timestampToIndex(t_m1);
        double average = PS.GetAvgAngularVelocity(idx_m1, idx_t);
        double change_in_yaw = PS.changeInYaw(t_m1, t); //should this be averaged? it's 10 measurements... Is there a bias?
        double yaw = diff.rotation().yaw();
        //did cedric output the yaw estimate from the IMU as omega? rather than the angular velocity?
        //the average angular velocity x the change in time (which is 1 second) seems to match. NOPE.
        //I'm not integrating right?
        //
        
        sum_ratio += change_in_yaw/yaw;
        std::cout << "change in yaw: optimized : " << yaw << "; measured: " << change_in_yaw << ", average ang_vel: " << average << std::endl;
        //yaw is in rad/s, because of a.between(b), right?
        //change_in_yaw is in rad, because of rad/s * delta t, right?
        t_m1 = t;
        b_tm1 = b_t;
    }
    std::cout << "ratio : " << sum_ratio / (POR.GetNumberOfPoses()/increment-1) << std::endl;
}

//difference between 0.002 and 0.02 in pixels? (but why is it consistently that far off? is it because the scene is consistently 10m away?)
//if the pixel resolution is too low, the gradient for project() will be unable to differentiate between angles that small, and between() will prefer lower change in yaw.
void
testAngularVelocity::
testReprojectionWithYawDifference()
{
    Camera _cam = ParseBoatSurvey::GetCamera();
    gtsam::Pose3 I = gtsam::Pose3::identity();
    gtsam::Rot3 yaw_optimized = gtsam::Rot3::yaw(0.002);
    gtsam::Rot3 yaw_measured =  gtsam::Rot3::yaw(0.02);
    
    gtsam::Pose3 pose_optimized(I.rotation() * yaw_optimized, I.translation());
    gtsam::Pose3 pose_measured(I.rotation() * yaw_measured, I.translation());
    
    gtsam::Cal3_S2::shared_ptr cal = _cam.GetGTSAMCam();
    gtsam::Point2 mid = gtsam::Point2(_cam.w()/2, _cam.h()/2);
    gtsam::PinholePose<gtsam::Cal3_S2> pca(I, cal);
    gtsam::Point3 P = pca.backproject(mid, 100);
    
    gtsam::PinholePose<gtsam::Cal3_S2> pco(pose_optimized, cal);
    gtsam::PinholePose<gtsam::Cal3_S2> pcm(pose_measured, cal);
    
    gtsam::Point2 opt2d = pco.project(P);
    gtsam::Point2 mes2d = pcm.project2(P);
    std::cout << "opt: " << opt2d << ", mes: " << mes2d << std::endl;
}



