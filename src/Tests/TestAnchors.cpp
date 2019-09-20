
#include "testAnchors.hpp"

#include <gtsam/geometry/Pose3.h>
#include <FileParsing/ParamsInterface.h>
#include "RFlowOptimization/InterSessionAnchors.hpp"
#include "FileParsing/ParseFeatureTrackFile.h"
#include <FileParsing/SaveOptimizationResults.h>

TestAnchors::TestAnchors(const Camera& cam):
_cam(cam)
{
    _results_dir  = "/Users/shane/Documents/research/experiments/anchors/";
    _maps_dir = "/Volumes/Untitled/data/maps_localizations/"; //140117/localizations/
    _pftbase = "/Volumes/Untitled/data/Lakeshore_KLT/";
    
    std::vector<std::string> dates_ = {"140106", "140117"};
    
    PORj_ = ParseOptimizationResults("/Volumes/Untitled/data/maps/", dates_[0]);
    PORk_ = ParseOptimizationResults("/Volumes/Untitled/data/maps/", dates_[1]);
    
    lpdi_.LoadLocalizations(_maps_dir + dates_[1], dates_);
    
    std::cout << "localizations loaded " << lpdi_.localizations.size() << std::endl;
    
    rfFG = new RFlowFactorGraph();
    GTS = GTSAMInterface(rfFG);
    
    ParamsInterface PI;
    PI.LoadParams(_results_dir);
    GTS.SetParams(PI.LoadParams(GTSAMInterface::Keys(), GTS.Params()));
    rfFG->SetParams(PI.LoadParams(FactorGraph::Keys(), rfFG->Params()));
}

void
TestAnchors::
AddLandmarks(std::vector<LandmarkTrack>& ended_tracks)
{
    for(int i=0; i<ended_tracks.size(); i++)
    {
        landmarks.push_back(ended_tracks[i]);
    }
}

void
TestAnchors::
BuildLandmarkSet()
{
    std::vector<LandmarkTrack> active;
    for(int i=0; i<PORk_.boat.size(); i++)
    {
        ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + "140117", PORk_.ftfilenos[i]);
        std::vector<gtsam::Point3> p3d = PORk_.GetSubsetOf3DPoints(pftf.ids);
        pftf.ModifyFTFData(p3d);
        std::vector<LandmarkTrack> ended_tracks = pftf.ProcessNewPoints(0, i, active, 100);
        AddLandmarks(ended_tracks);
    }
    AddLandmarks(active);
    
    //sort the cached landmarks by the pose order
    std::qsort(&landmarks[0], landmarks.size(), sizeof(LandmarkTrack), [](const void* a, const void* b) {
        const LandmarkTrack* arg1 = static_cast<const LandmarkTrack*>(a);
        const LandmarkTrack* arg2 = static_cast<const LandmarkTrack*>(b);
        
        if(arg1->key < arg2->key) return -1;
        else if(arg1->key > arg2->key) return 1;
        else if(arg1->camera_keys.size() < arg2->camera_keys.size()) return -1;
        else if(arg1->camera_keys.size() > arg2->camera_keys.size()) return 1;
        return 0;
    });
}

void TestAnchors::ConstructFactorGraph()
{
    int survey = 0;
    for(int i=0; i<PORk_.boat.size(); i++) {
        gtsam::Pose3 traj = PORk_.CameraPose(i);
        rfFG->AddPose(survey, i, traj);
        GTS.InitializePose(rfFG->GetSymbol(survey, i), traj);
        
        if(i>0) {
            gtsam::Pose3 last = PORk_.CameraPose(i-1);
            gtsam::Pose3 btwn = last.between(traj);
            rfFG->AddCustomBTWNFactor(survey, i-1, survey, i, btwn, 0.01);
        }
    }
    
    int percent_landmark_tracks = 10;
    srand(std::time(0));
    for(int i=0; i<landmarks.size(); i++)
    {
        if(rand()%100 > percent_landmark_tracks)
            continue;
        rfFG->AddLandmarkTrack(_cam.GetGTSAMCam(), landmarks[i]);
    }
}

void TestAnchors::AddLocalizations()
{
    double tn = 0.001;
    double rn = 0.01;
    std::vector<double> noise = {tn, tn, tn, rn, rn, rn};
    int cf=0, cb=0;
    int n = 0;
    int anum = 0;
    
    for(int j=0; j<lpdi_.localizations.size(); j++)
    {
        LocalizedPoseData& l = lpdi_.localizations[j];
        
        gtsam::Pose3 pj_a = GTSAMInterface::VectorToPose(PORj_.boat[l.s0time]);
        gtsam::Pose3 hat_pj_a_to_pkj_b = l.GetTFP0ToP1F0();
        gtsam::Pose3 hat_pkj_b = pj_a.compose(hat_pj_a_to_pkj_b);
        
        
        gtsam::Symbol anchor('a', anum);
        rfFG->AddOneSessionAnchor(rfFG->GetSymbol(0, l.s1time), anchor, hat_pkj_b, noise);
        
        
        if(n++ % 100 == 0)
        {
            gtsam::Pose3 pk_b = GTSAMInterface::VectorToPose(PORk_.boat[l.s1time]);
            gtsam::Pose3 pkj = pk_b.compose(hat_pkj_b);
            GTS.InitializePose(anchor, pkj);
            if(anum > 0)
            {
                //add a between.
                
            }
            anum++;
        }
    }
}

void
TestAnchors::
EvaluateAnchors()
{
    
}

void TestAnchors::Run()
{
    
    ConstructFactorGraph();
    
    AddLocalizations();
    
    GTS.SetIdentifier("140117");
    GTS.RunBundleAdjustment();
    
    std::vector<std::vector<double> > opt_landmarks = GTS.GetOptimizedLandmarks(true);
    std::vector<std::vector<double> > opt_poses = GTS.GetOptimizedTrajectory(0, PORk_.boat.size());
    std::vector<std::vector<double> > opt_anchors;
    std::vector<gtsam::Pose3> anchors;
    for(int i=0; i<num_anchors; i++)
    {
        gtsam::Symbol anchorsymbol('a', i);
        gtsam::Pose3 anchor = GTS.PoseResult(anchorsymbol);
        anchors.push_back(anchor);
        std::vector<double> anchor_vec = GTSAMInterface::PoseToVector(anchor);
        opt_anchors.push_back(anchor_vec);
    }
    
    std::vector<double> drawscale = {-300,300,-300,300};
    
    std::cout << "added " << anchors.size() << std::endl;
    
    SaveOptimizationResults SOR(_results_dir + "140117");
    SOR.PlotAndSaveCurrentEstimate(opt_landmarks, opt_poses, opt_anchors, drawscale);
    
    
    
    
}






















