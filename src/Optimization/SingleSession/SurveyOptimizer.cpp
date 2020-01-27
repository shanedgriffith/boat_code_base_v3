//
//  SurveyOptimizer.cpp
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/11/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "SurveyOptimizer.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

#include "FileParsing/ParseSurvey.h"
#include "BoatSurvey/ParseBoatSurvey.hpp"
#include "EvaluateSLAM.h"
#include "Localization/LocalizePose6D.h"
#include "Localization/LocalizePose5D.h"

using namespace std;

const vector<string> SurveyOptimizer::keys = {
    "OPT_OFFSET", "OPT_SKIP", "OPT_STOP", "CAM_OFFSET", "CAM_SKIP"
};

SurveyOptimizer::SurveyOptimizer(const Camera& cam, FactorGraph * _fg, std::string date, std::string results_dir, bool print_data_increments):
cam_(cam), _results_dir(results_dir), FG(_fg), _date(date), _print_data_increments(print_data_increments) {}

SurveyOptimizer::SurveyOptimizer(const Camera& cam, std::string date, std::string results_dir, bool print_data_increments):
cam_(cam), _results_dir(results_dir), _date(date), _print_data_increments(print_data_increments)
{
    FG = new FactorGraph();
    clean_up = true;
}

void SurveyOptimizer::Initialize(){
    initialized = true;
    
    num_cameras_in_traj = 0;
    GTS = GTSAMInterface(FG);
    LoadParameters();
    
    _incremental = ((int) vals[Param::OPT_SKIP]) == 1;
    std::cout << "incremental? " << _incremental << std::endl;
    GTS.SetupSLAM(_incremental);
    if(debug)  GTS.SetPrintSymbols();
    
    std::cout << "Optimizing " << _date;
    if(_incremental) std::cout << " incrementally " << std::endl;
    else             std::cout << " in batch mode " << std::endl;
}

bool SurveyOptimizer::OptimizeThisIteration(int camera_key){
    //determine whether optimization should be run this iteration.
    return camera_key > vals[Param::OPT_OFFSET] && camera_key % (int) vals[Param::OPT_SKIP] == 0;
}

void SurveyOptimizer::RunGTSAM(bool update_everything){
    /*Running GTSAM consumes a lot of time in the update and in computing the MAP value for the saved variables*/
    if(!initialized){cout << "check initialization"<<endl; exit(1);}
    if(!dry_run) GTS.Update(update_everything);
    if(verbose) FG->PrintFactorGraph();
}

void SurveyOptimizer::SaveResults(SaveOptimizationResults& SOR, int iteration, double percent_completed, vector<double> drawscale){
    if((_print_data_increments || percent_completed == 100) and iteration > 5){
        vector<vector<double> > ls = GTS.GetOptimizedLandmarks();
        vector<vector<double> > ts = GTS.GetOptimizedTrajectory(FG->key[(int) FactorGraph::var::X], num_cameras_in_traj);
        vector<vector<double> > vs = GTS.GetOptimizedTrajectory(FG->key[(int) FactorGraph::var::V], num_cameras_in_traj);
        SOR.PlotAndSaveCurrentEstimate(ls, ts, vs, drawscale);
    }
    SOR.StatusMessage(iteration, percent_completed);
}

void SurveyOptimizer::LoadParameters(){
    /*Tries to load the parameters from the file. If they don't exist, 
     the defaults are used.*/
	if(!initialized){cout << "check initialization"<<endl; exit(1);}

    ParamsInterface PI;
    PI.LoadParams(_results_dir);
    SetParams(PI.LoadParams(SurveyOptimizer::Keys(), Params()));
    GTS.SetParams(PI.LoadParams(GTSAMInterface::Keys(), GTS.Params()));
    FG->SetParams(PI.LoadParams(FactorGraph::Keys(), FG->Params()));
}

void SurveyOptimizer::SaveParameters(){
    /*Save the params with the result.*/
    ParamsInterface PI;
    PI.AddParams(Keys(), Params());
    PI.AddParams(GTSAMInterface::Keys(), GTS.Params());
    PI.AddParams(FactorGraph::Keys(), FG->Params());
    PI.SaveParams(_results_dir + _date);
}

void SurveyOptimizer::AddPoseConstraints(double delta_time, gtsam::Pose3 btwn_pos, gtsam::Pose3 vel_est, int camera_key, bool transition){
    static bool transition_prevstep = true;
    
    //the island transition. if the pan/tilt camera moves, use a simple odom factor to get around the yaw constraint.
    if(!transition) {
        if(transition_prevstep) {
            GTS.InitializePose(FG->key[(int)FactorGraph::var::V], camera_key-1, vel_est);
            FG->AddVelocity(camera_key-1, vel_est);
            transition_prevstep = false;
        }
        
        GTS.InitializePose(FG->key[(int)FactorGraph::var::V], camera_key, vel_est);
        FG->AddVelocity(camera_key, vel_est);
        
        FG->AddKinematicConstraint(camera_key, delta_time);
        FG->AddSmoothVelocityConstraint(camera_key);
    } else {
        FG->AddOdomFactor(camera_key, btwn_pos);
        transition_prevstep=true;
        if(debug)
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>transition at<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pose " << camera_key << endl;
    }
}

//void SurveyOptimizer::AddCamera(int camera_key, gtsam::Pose3& measured, gtsam::Pose3& localized){
//    numcam_eras_in_traj++;
//    GTS.InitializePose(FG->key[(int)FactorGraph::var::X], camera_key, localized);
//    FG->AddCamera(camera_key, measured);
//}

void SurveyOptimizer::AddLandmarkTracks(vector<LandmarkTrack>& landmarks){
    for(int i=0; i<landmarks.size(); i++)
    {
        FG->AddLandmarkTrack(cam_.GetGTSAMCam(), landmarks[i]);
    }
}

void SurveyOptimizer::AddActiveLandmarks(vector<LandmarkTrack>& landmarks)
{
    /*Use this for incremental optimization; the active landmarks are ultimately used to estimate the new location of a pose.*/
    for(int i=0; i<landmarks.size(); i++)
    {
        int len = landmarks[i].Length();
        if(len == 2)
        {
            FG->AddLandmarkTrack(cam_.GetGTSAMCam(), landmarks[i]);
        }
        else if(len > 2)
        {
            int smart_factor_idx = FG->GraphHasLandmark(landmarks[i].key);
            if(smart_factor_idx == -1)
            {
                std::cout << "SurveyOptimizer::AddActiveLandmarks() error. the active landmark should have already been added to the factor graph, but it wasn't found." << std::endl;
                exit(-1);
            }
            
            //remove the factor from isam2
            GTS.RemoveLandmarkFactor(smart_factor_idx);
            
            //add the new measurement to the existing factor and add it back to the factor graph, which will be added to isam2
            FG->AddToExistingLandmark(landmarks[i].points[len-1], (int) 'x', landmarks[i].camera_keys[len-1], smart_factor_idx);
        }
    }
}

void SurveyOptimizer::CacheLandmarks(vector<LandmarkTrack>& inactive){
    for(int i=0; i<inactive.size(); i++)
        cached_landmarks[cache_set].push_back(inactive[i]);
}

std::vector<gtsam::Pose3> SurveyOptimizer::LocalizeCurPose2D(int cur_pose_idx)
{   //argument to this function is needed when active.size() is zero.
    int first_pose_idx = cur_pose_idx;
    int latest_pose_idx = cur_pose_idx;
    
    if(active.size() > 3)
    {
        first_pose_idx = active[0].camera_keys[0].index();
        latest_pose_idx = active[0].camera_keys[active[0].Length()-1].index();
    }
    
    int first = std::min(first_pose_idx, latest_pose_idx-2);
    std::shared_ptr<gtsam::Values> v = GTS.getPoseValuesFrom(FG->key[(int)FactorGraph::var::X], first, latest_pose_idx);
    gtsam::Pose3 pose_t2 = v->at<gtsam::Pose3>(gtsam::Symbol(FG->key[(int)FactorGraph::var::X], latest_pose_idx-2));
    gtsam::Pose3 pose_t1 = v->at<gtsam::Pose3>(gtsam::Symbol(FG->key[(int)FactorGraph::var::X], latest_pose_idx-1));
    
    std::vector<gtsam::Pose3> poses = {pose_t2, pose_t1};
    
    if(active.size() <= 3)
    {
        std::cout << "______________________ no localization. the active set is too small. " << active.size() << " active landmark tracks." << std::endl;
        return poses;
    }
    
    if(first_pose_idx > latest_pose_idx-2)
    {
        std::cout << "______________________ no localization. not enough poses in the active set. poses " << first_pose_idx << " to " << latest_pose_idx << std::endl;
        return poses;
    }
//    gtsam::Pose3 pose_t_est = pose_t1.compose(pose_t1.between(pose_t2));
    gtsam::EssentialMatrix e_t1 = gtsam::EssentialMatrix::FromPose3(pose_t1.between(pose_t2));
    
    std::vector<gtsam::Point2> p2d0;
    std::vector<gtsam::Point2> p2d1;
    for(int i=0; i<active.size(); ++i)
    {
        int smart_factor_idx = FG->GraphHasLandmark(active[i].key);
        if(smart_factor_idx < 0)
            continue;
        
        if(active[i].Length() < 3)
            continue;
        
        p2d0.push_back(active[i].points[active[i].Length()-2]);
        p2d1.push_back(active[i].points[active[i].Length()-1]);
    }
    
    if(p2d0.size() < 5)
    {
        std::cout << "______________________ 2D-2D no localization. too few correspondences. " << p2d0.size() << " 2d-2d correspondences of " << active.size() << " total, over poses " << first_pose_idx << " to " << latest_pose_idx << std::endl;
        return poses;
    }
    std::cout << "2D-2D localizing the new pose using: " << p2d0.size() << " of " << active.size() << " total, over poses " << first_pose_idx << " to " << latest_pose_idx << std::endl;

    LocalizePose5D loc(cam_, p2d0, p2d1);
    loc.setDebug(); //TODO: 1) verify that the robust loss is better than explicit filter; 2) tune the weight parameter to make that the case; 3) implement the barron loss.
    loc.setRANSACMethod(LocalizePose5D::METHOD::_NISTER);
    loc.setInitialEstimate(e_t1); //TODO: why is the previous pose a better initial estimate than pose_t1.compose(pose_t2.between(pose_t1));
//    loc.setRobustLoss();
    gtsam::EssentialMatrix localized_e;
    std::vector<double> res;
    bool suc;
    std::tie(suc, localized_e, res) = loc.UseBAIterative();
    if(suc and (res[1] > 0.5 * p2d0.size() or res[1] > 15))
    {
//        poses.push_back(localized_pose);
    }
    else std::cout << "2D-2D localization failed: inlier ratio: " << res[1] << " of " << p2d0.size() << ", all avg rerror: " << res[2] << std::endl;
    
    return {};
}

void
SurveyOptimizer::
testE(const gtsam::Pose3& localized_pose, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d0, std::vector<gtsam::Point2>& p2d1)
{
    gtsam::EssentialMatrix E = gtsam::EssentialMatrix::FromPose3(localized_pose);
    std::cout << "pose: " << gtsam::Pose3::Logmap(localized_pose).transpose() << std::endl;
    for(int i=0; i<p3d.size(); i++)
    {
        gtsam::Point3 tfp = localized_pose.transform_to(p3d[i]);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        
        gtsam::Vector3 va = gtsam::EssentialMatrix::Homogeneous(p2d0[i]).normalized();
        gtsam::Vector3 vb = gtsam::EssentialMatrix::Homogeneous(p2d1[i]).normalized();
        double err = E.error(va, vb);
        double dist = res.distance(p2d1[i]);
        double dp = va.dot( (E.matrix() * vb)) ;
        double angle = std::acos(dp);
        std::cout << "" << std::acos(0) << ", " << std::acos(1) << std::endl;
        std::cout << "proj d: " << dist << "; in image? " << cam_.InsideImage(res) << "; E erro: " << err << "; va: " << va.transpose() << "; vb: " << vb.transpose() << ", " << (E.matrix() * vb).transpose() << "; ndot: "  << dp << ", angle: " << angle << std::endl;
    }
//    exit(1);
}

std::vector<gtsam::Pose3> SurveyOptimizer::LocalizeCurPose(int cur_pose_idx)
{   //argument to this function is needed when active.size() is zero.
    int first_pose_idx = cur_pose_idx;
    int latest_pose_idx = cur_pose_idx;
    
    if(active.size() > 3)
    {
        first_pose_idx = active[0].camera_keys[0].index();
        latest_pose_idx = active[0].camera_keys[active[0].Length()-1].index();
    }
    
    int first = std::min(first_pose_idx, latest_pose_idx-2);
    std::shared_ptr<gtsam::Values> v = GTS.getPoseValuesFrom(FG->key[(int)FactorGraph::var::X], first, latest_pose_idx);
    gtsam::Pose3 pose_t2 = v->at<gtsam::Pose3>(gtsam::Symbol(FG->key[(int)FactorGraph::var::X], latest_pose_idx-2));
    gtsam::Pose3 pose_t1 = v->at<gtsam::Pose3>(gtsam::Symbol(FG->key[(int)FactorGraph::var::X], latest_pose_idx-1));
    
    std::vector<gtsam::Pose3> poses = {pose_t2, pose_t1};
    
    if(active.size() <= 3)
    {
        std::cout << "______________________ no localization. the active set is too small. " << active.size() << " active landmark tracks." << std::endl;
        return poses;
    }
    
    if(first_pose_idx > latest_pose_idx-2)
    {
        std::cout << "______________________ no localization. not enough poses in the active set. poses " << first_pose_idx << " to " << latest_pose_idx << std::endl;
        return poses;
    }
    gtsam::Pose3 pose_t_est = pose_t1.compose(pose_t1.between(pose_t2));
    
    std::vector<gtsam::Point3> p3d;
    std::vector<gtsam::Point2> p2d1;
    std::vector<gtsam::Point2> p2d0;
    for(int i=0; i<active.size(); ++i)
    {
        int smart_factor_idx = FG->GraphHasLandmark(active[i].key);
        if(smart_factor_idx < 0)
            continue;
        
        boost::optional<gtsam::Point3> triangulated;
        try{
            triangulated = GTS.triangulatePointFromPoseValues(*v, smart_factor_idx);
        }catch(std::exception& e)
        {
            //gtsam::ValuesKeyDoesNotExist
            std::cout << e.what() << std::endl;
            std::cout << "camera keys expected: " << first_pose_idx << " to " << latest_pose_idx << "" << std::endl;
            FG->landmark_factors[0][smart_factor_idx].printKeys("camera keys actual:   ");
            
            //active has the wrong key? nope.
            //incorrect mapping from key to smart_factor_idx? (duplicate mapping?)
            for(int j=FG->landmark_keys[0].size()-1; FG->landmark_keys[0][j] >= active[i].key; --j)
            {
                std::cout << " l" << FG->landmark_keys[0][j] << ": ";
                FG->landmark_factors[0][j].printKeys("");
                if(j != FG->GraphHasLandmark(FG->landmark_keys[0][j]))
                {
                    std::cout << " bad mapping" << std::endl;
                    exit(-1);
                }
            } std::cout << std::endl;
            exit(-1);
        }
        if(triangulated)
        {
            p3d.push_back(triangulated.get());
            p2d1.push_back(active[i].points[active[i].Length()-1]);
//            p2d0.push_back(active[i].points[active[i].Length()-2]);
        }
    }
    
    if(p3d.size() <= 3)
    {
        std::cout << "______________________ no localization. too few triangulated points. " << p3d.size() << " 3d points of " << active.size() << " total, over poses " << first_pose_idx << " to " << latest_pose_idx << std::endl;
        return poses;
    }
    std::cout << "3D-2D localizing the new pose using: " << p3d.size() << " of " << active.size() << " points of " << active.size() << " total, over poses " << first_pose_idx << " to " << latest_pose_idx << std::endl;
    
    LocalizePose6D loc(cam_, p3d, p2d1);
    loc.setDebug(); //TODO: 1) verify that the robust loss is better than explicit filter; 2) tune the weight parameter to make that the case; 3) implement the barron loss.
    loc.setRANSACMethod(LocalizePose6D::METHOD::_PNP);
    loc.setInitialEstimate(pose_t1); //TODO: why is the previous pose a better initial estimate than pose_t1.compose(pose_t2.between(pose_t1));
    loc.setRobustLoss();
    gtsam::Pose3 localized_pose;
    std::vector<double> res;
    bool suc;
    std::tie(suc, localized_pose, res) = loc.UseBAIterative();
    if(suc and (res[1] > 0.5 * p3d.size() or res[1] > 15))
    {
        poses.push_back(localized_pose);
    }
    else std::cout << "3D-2D localization failed: inlier ratio: " << res[1] << " of " << p3d.size() << ", all avg rerror: " << res[2] << std::endl;
    
//    testE(poses[1].between(poses[2]), p3d, p2d0, p2d1);
    
    return poses;
}

int SurveyOptimizer::ConstructGraph(std::shared_ptr<ParseSurvey> PS, ParseFeatureTrackFile& PFT, int cidx, int lcidx, bool gap){
    //get the poses
    static gtsam::Pose3 lastcam_ = PS->CameraPose(lcidx);
    gtsam::Pose3 pose_prior = PS->CameraPose(cidx); //the camera pose estimated using sensors.;
    gtsam::Pose3 curcam = pose_prior;
    gtsam::Pose3 btwn_pos;
    static int suc_count = 0;
    //static ParseOptimizationResults POR("/Volumes/Untitled/data/iSAM/", "140106_better_z_values_plus_loop_closure (closer to ground truth)"); //TODO: compare to the batch optimization method.
    
    int camera_key = FG->GetNextCameraKey();
//    std::cout << "at iteration with camera x" << camera_key << std::endl;
    
    //check for camera transitions
    bool flipped = PS->CheckCameraTransition(cidx, lcidx);
    bool transition = flipped || gap;
    if(transition and not _incremental){
        if(debug) std::cout << "flip? " << flipped << ", gap? " << gap << ", at " << cidx << std::endl;
    	if(cache_landmarks) CacheLandmarks(active);
        else AddLandmarkTracks(active);
    	active.clear();
        std::cout << "transition" << std::endl;
    }
    
    //process the landmark measurement
    std::vector<LandmarkTrack> inactive = PFT.ProcessNewPoints((int) 'x', camera_key, active, percent_of_tracks);
    
    //initialize the camera (this is added after processing the landmarks in order to have the active set for localizing the current pose)
    boost::optional<gtsam::Pose3> curpose;
    bool loc_succeeded = true;
    if(_incremental and camera_key > 1)
    {
        std::vector<gtsam::Pose3> poses = LocalizeCurPose(camera_key);
        lastcam_ = poses[1];
        if(poses.size() > 2) {
            LocalizeCurPose2D(camera_key);
            std::cout << "------------------------------------------------------------------------------------------------------------------------" << std::endl;
            curcam = poses[2];
            btwn_pos = poses[1].between(poses[2]);
            ++suc_count;
            loc_succeeded = true;
            std::cout << camera_key << ": localization succeeded. success count: " << suc_count;
            std::vector<double> lastp = GTSAMInterface::PoseToVector(poses[0]);
            std::cout << " last: ";
            for(int i=0; i<6; ++i)
            {
                std::cout << lastp[i] << ", ";
            } std::cout << std::endl;
//            transition = true; //disabled the constant velocity assumption instead.
        }
        else {
            btwn_pos = poses[0].between(poses[1]);
            std::cout << camera_key << ": localization failed. " << active.size() << " in the active set" << std::endl;
            std::cout << "velocity : ";
            std::vector<double> betp = GTSAMInterface::PoseToVector(btwn_pos);
            for(int i=0; i<6; ++i)
            {
                std::cout << betp[i] << ", ";
            } std::cout << std::endl;
            curcam = lastcam_.compose(btwn_pos);
        }
    }
    GTS.InitializePose(FG->key[(int)FactorGraph::var::X], camera_key, curcam);
    
    //add the landmark measurement to the graph
    if(cache_landmarks) CacheLandmarks(inactive);
    else if(_incremental) AddActiveLandmarks(active);
    else AddLandmarkTracks(inactive);
    
    ++num_cameras_in_traj;
    FG->AddCamera(camera_key, pose_prior);
    
    //add the kinematic constraints.
    if(camera_key != 0)
    {
        if(camera_key == 1)
            btwn_pos = lastcam_.between(curcam);
        
        if(dynamic_pointer_cast<ParseBoatSurvey>(PS))
        {
            std::shared_ptr<ParseBoatSurvey> pbs = dynamic_pointer_cast<ParseBoatSurvey>(PS);
            double change_in_yaw = pbs->changeInYaw(PS->timings[lcidx], PS->timings[cidx]);
            gtsam::Rot3 measured_rot = gtsam::Rot3::Ypr(change_in_yaw, 0, 0);
//            std::cout << "btwn: " << std::endl;
//            btwn_pos.print();
//            std::cout << "rot: " << std::endl;
//            measured_rot.print();
            FG->AddOrientationConstraint(camera_key-1, camera_key, measured_rot);
        }
        
        if(PS->ConstantVelocity()) {
//        if(not loc_succeeded) {
            //note: keep constant velocity, but assume the between factor accounts for the time between poses.
            double av = PS->GetAvgAngularVelocity(lcidx, cidx); //Estimate the angular velocity of the boat.
            double delta_t = PS->timings[cidx]-PS->timings[lcidx]; //Get the difference in time.
            std::vector<double> pvec = {av*delta_t, 0.0, 0.0, btwn_pos.x(), btwn_pos.y(), 0.0};
            gtsam::Pose3 vel_est = GTSAMInterface::VectorToPose(pvec);
            AddPoseConstraints(delta_t, btwn_pos, vel_est, camera_key, transition);
        } else {
            FG->AddOdomFactor(camera_key, btwn_pos, loc_succeeded);
            
            //add IMU factor.
            //(custom IMU factor; accumulate the change in rotation with which to constrain the yaw.
        }
    }
    
    if(not _incremental or camera_key < 2)
    {
        lastcam_ = curcam;
    }
    return camera_key;
}

void SurveyOptimizer::Optimize(std::shared_ptr<ParseSurvey> PS) {
    //TODO: implement pose decimation, rather than the 1 by 10 (CAM_SKIP) rule currently used.
	if(!initialized){cout << "SurveyOptimizer::Optimize() check initialization"<<endl; exit(1);}
    
    std::cout << "Constructing the factor graph." << std::endl;
    
    EvaluateSLAM es(cam_, _date, _results_dir);
    es.debug=true;
    
    SaveOptimizationResults SOR(_results_dir + _date);
    SOR.SetSaveStatus();
    SOR.SetDrawMap();
    
    int cidx = 0, lcidx=0;
    for(int i=vals[Param::CAM_OFFSET]; ; i=i+vals[Param::CAM_SKIP]) {
        //Find the AUX file entry that is time-aligned with the visual feature track data. (for the camera pose)
        bool gap = PS->CheckGap(lcidx, lcidx + vals[Param::CAM_SKIP]);
        
        ParseFeatureTrackFile PFT = PS->LoadVisualFeatureTracks(cam_, i, gap);
        cidx = PS->FindSynchronizedAUXIndex(PFT.time, cidx);
        if(cidx==-1) break;
        if(cidx == lcidx) continue; //the feature track file didn't advance anything.
        if(!PS->Useable(cidx, lcidx)) continue; //the camera is being adjusted, don't use the data.
        if(PFT.CheckImageDuplication(active)) continue; //the image was duplicated, so the tracking data is unhelpful, don't use the data.
        
        //Construct the graph (camera, visual landmarks, velocity, and time)
        //int camera_key = ConstructGraph(PS->CameraPose(cidx), PFT, av, PS->timings[cidx]);
        int camera_key = ConstructGraph(PS, PFT, cidx, lcidx, gap);
        
        //Optimize the graph
        if(OptimizeThisIteration(camera_key)){
            SOR.TimeOptimization();
            RunGTSAM(_print_data_increments);
            if(_print_data_increments)
            {
                SaveResults(SOR, camera_key, 100.0*cidx/(PS->timings.size()-1000), PS->GetDrawScale());
                es.ErrorForSurvey(PS->_pftbase, true);
            }
        }
        
        //Log the data alignment.
        int imageno = PS->GetImageNumber(cidx);
        SOR.SaveDataCorrespondence(camera_key, i, cidx, imageno, PS->timings[cidx]);
        if(debug)
            cout << "correspondence: " << camera_key <<", " << i << ", "<<cidx<<", "<<imageno<<", "<<(int) (PS->timings[cidx]/100000) << ((int)PS->timings[cidx])%100000 <<endl;
        
        lcidx = cidx;
        
        if(camera_key >= vals[Param::OPT_STOP])
            break;
    }
    
    //Add the remaining landmarks
    AddLandmarkTracks(active);
    active.clear();
    
    std::cout << "Optimizing..." << std::endl;
    
    //Final optimization
    SOR.TimeOptimization();
    RunGTSAM(true);
    SaveResults(SOR, 0, 100.0, PS->GetDrawScale());
    SaveParameters();
    es.ErrorForSurvey(PS->_pftbase, true);
    es.PrintTots();
}









