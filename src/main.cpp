#include <iostream>

#include <Optimization/SurveyOptimizer.h>
#include <DataTypes/Camera.hpp>
#include <RFlowOptimization/EvaluateRFlow.hpp>
#include <RFlowOptimization/RFlowSurveyOptimizer.hpp>
#include <RFlowOptimization/AcquireISConstraints.hpp>
#include <RFlowEvaluation/AlignVisibilitySet.hpp>
#include <Visualizations/FlickeringDisplay.h>
#include <BoatSurvey/ParseBoatSurvey.hpp>
#include <BikeSurvey/PreprocessBikeRoute.hpp>
#include <BikeSurvey/ParseBikeRoute.hpp>
#include <Tests/TestBikeSurvey.h>
#include <RFlowOptimization/MultiSessionOptimization.hpp>
#include <RFlowOptimization/MultiAnchorsOptimization.hpp>
#include <Tests/TestTransforms.hpp>

using namespace std;

vector<string> cluster_paths = {"/home/shaneg/results/", "/home/shaneg/data/VBags/", "/home/shaneg/data/Lakeshore_KLT/", "/home/shaneg/data/bike_datasets/"};
vector<string> lab_paths = {"/cs-share/dream/results_consecutive/", "/mnt/tale/cedricp/VBags/", "/mnt/tale/shaneg/Lakeshore_KLT/", "/mnt/tale/shaneg/bike_datasets/"};

int main(int argc, char *argv[]) {
    if(argc<4) {
        std::cout << "need 3 input arguments" << std::endl;
        exit(-1);
    }
    std::cout << "starting program" << std::endl;
    
    
    string results_dir = lab_paths[0];
    string query_loc = lab_paths[1];
    string pftbase = lab_paths[2];
    string visibility_dir = "/mnt/tale/shaneg/results/visibility_poses/all/";
    string bike_datasets = lab_paths[3];
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
        MultiAnchorsOptimization mao(axisptz, results_dir, pftbase, query_loc, argv[2]);
        mao.IterativeMerge();
//        MultiSessionOptimization mso(axisptz, results_dir, pftbase, argv[2]);
//        mso.SetDryRun();
//        mso.IterativeMerge();
        //RFlowSurveyOptimizer ra(axisptz, argv[1], results_dir, pftbase);
        //ra.IterativeMerge();
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
        TestTransforms test;
        test.TestConstraintProportions(axisptz);
        
        break;}
    }

/*
      FlickeringDisplay fd(argv[1], argv[2]);
      string dir = results_dir + argv[1] + "_to_" + argv[2];
      fd.DisplaySurveyComparisonDir(dir);
*/

    return 0;
}
