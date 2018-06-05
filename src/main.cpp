#include <iostream>

#include <Optimization/SurveyOptimizer.h>
#include <DataTypes/Camera.hpp>
#include <RFlowOptimization/EvaluateRFlow.hpp>
#include <RFlowOptimization/AcquireISConstraints.hpp>
#include <RFlowEvaluation/AlignVisibilitySet.hpp>
#include <Visualizations/FlickeringDisplay.h>
#include <BoatSurvey/ParseBoatSurvey.hpp>
#include <BikeSurvey/PreprocessBikeRoute.hpp>
#include <BikeSurvey/ParseBikeRoute.hpp>
#include <Tests/TestBikeSurvey.h>
#include <RFlowOptimization/MultiSessionOptimization.hpp>
<<<<<<< HEAD
#include <RFlowOptimization/MultiSessionIterativeSmoothingAndRefinement.hpp>
=======
#include <RFlowOptimization/MultiAnchorsOptimization.hpp>
#include <RFlowOptimization/AnchoredMultiSessionOptimization.hpp>
#include <RFlowOptimization/EfficientMultiSessionOptimization.hpp>
>>>>>>> 6cd30ffd66bc20ccf9d12509d50f9cd50a83f239
#include <Tests/TestTransforms.hpp>
#include <Evaluation/SessionConvergence.hpp>
#include <RFlowOptimization/MultiCascade.hpp>
#include <RFlowEvaluation/ForBMVCFigure.hpp>

using namespace std;

vector<string> cluster_paths = {"/home/shaneg/results/", "/home/shaneg/data/VBags/", "/home/shaneg/data/Lakeshore_KLT/", "/home/shaneg/data/bike_datasets/"};
vector<string> lab_paths = {"/cs-share/dream/results_consecutive/", "/mnt/tale/cedricp/VBags/", "/mnt/tale/shaneg/Lakeshore_KLT/", "/mnt/tale/shaneg/bike_datasets/"};
vector<string> home_paths = {"/Users/shane/Documents/research/data/", "/Volumes/SAMSUNG/VBags/", "/Users/shane/Documents/research/data/Lakeshore_KLT/", ""};

int main(int argc, char *argv[]) {
    if(argc<4) {
        std::cout << "need 3 input arguments" << std::endl;
        exit(-1);
    }
    std::cout << "starting program" << std::endl;
    
    string results_dir = home_paths[0];
    string query_loc = home_paths[1];
    string pftbase = home_paths[2];
    string visibility_dir = "****USE COVISIBILITY****";
    string bike_datasets = home_paths[3];
    if(argc>4){
        results_dir = cluster_paths[0];
        query_loc = cluster_paths[1];
        pftbase = cluster_paths[2];
        bike_datasets = cluster_paths[3];
    }
    
    int prog = atoi(argv[3]);
    switch(prog){
    case 0:{
        int start = -1;
        if(argc == 5) start = atoi(argv[4]);
        Camera axisptz = ParseBoatSurvey::GetCamera();
        AcquireISConstraints acq(axisptz, argv[1], query_loc, pftbase, results_dir);
        acq.Run(start);
        break;}
    case 1:{
        Camera axisptz = ParseBoatSurvey::GetCamera();
        //argv[2] is used here to specify an optimize-up-to date. When not specified, all possible dates are used.
//        MultiSessionOptimization mso(axisptz, results_dir, pftbase, argv[2]);
//        EfficientMultiSessionOptimization mso(axisptz, results_dir, pftbase, argv[2]);
        MultiCascade mso(axisptz, results_dir, pftbase, argv[2]);
//        MultiSessionIterativeSmoothingAndRefinement mso(axisptz, results_dir, pftbase, argv[2]);
        mso.IterativeMerge();
//        mso.SetDryRun();
//        mso.IterativeMerge();
        break;}
    case 2:{
        //keep date 1 the same and change date 2 to have the images be aligned at the same places. 
        Camera axisptz = ParseBoatSurvey::GetCamera();
        AlignVisibilitySet avs(axisptz, argv[1], argv[2], pftbase, query_loc, results_dir, visibility_dir);
        avs.Visibility();
        break;}
    case 3:{
        FlickeringDisplay fd(argv[1], argv[2]);
        string dir = results_dir + argv[1] + "_to_" + argv[2] + "/";
        std::cout << "dir: " << dir << std::endl;
        fd.CompareFromDir(dir);
        break;}
    case 5:{
        PreprocessBikeRoute pbr(bike_datasets, argv[1]);
        pbr.Preprocess();
        break;}
    case 6:{
        ParseBikeRoute pbr(bike_datasets, argv[1]);
        Camera nexus = ParseBikeRoute::GetCamera();
        SurveyOptimizer so(nexus, argv[1], results_dir);
        so.Initialize();
        so.Optimize(pbr);
        break;}
    case 7:{
        ParseBoatSurvey PS(query_loc, pftbase, argv[1]);
        Camera axisptz = ParseBoatSurvey::GetCamera();
        SurveyOptimizer so(axisptz, argv[1], results_dir, true);
        so.Initialize();
        so.Optimize(PS);
        break;}
    case 8:{
        ParseBoatSurvey PS(query_loc, pftbase, argv[1]);
        PS.PlayPoses();
        break;}
    case 9:{
        TestBikeSurvey tbs;
        tbs.GenerateTrajectory();
        tbs.TestVO();
        //tbs.TestVisualOdometry();
        //tbs.TestTriangulation();
        break;}
    case 10: {
        //for the experimental programs.
        Camera axisptz = ParseBoatSurvey::GetCamera();
        //create Timelapse quality map
//        AlignVisibilitySet avs(axisptz, argv[1], argv[2], pftbase, query_loc, results_dir, visibility_dir);
//        avs.VisualizeAllLabelsInOneMap();
        //test transform changes
//        TestTransforms::CheckBtwn(axisptz);
//        TestTransforms::TestLocalization(axisptz);
//        TestTransforms test;
//        test.TestConstraintProportions(axisptz);
        SessionConvergence sc(axisptz, pftbase);
        sc.CompareSessions();
        
        break;}
    }

/*
      FlickeringDisplay fd(argv[1], argv[2]);
      string dir = results_dir + argv[1] + "_to_" + argv[2];
      fd.DisplaySurveyComparisonDir(dir);
*/

    return 0;
}
