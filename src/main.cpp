#include <iostream>

#include <Optimization/SingleSession/SurveyOptimizer.h>
#include <DataTypes/Camera.hpp>
#include <Visualizations/FlickeringDisplay.h>
#include <BoatSurvey/ParseBoatSurvey.hpp>
#include <BikeSurvey/PreprocessBikeRoute.hpp>
#include <BikeSurvey/ParseBikeRoute.hpp>
#include <Tests/TestBikeSurvey.h>
#include <Tests/TestTransforms.hpp>
#include <Evaluation/SessionConvergence.hpp>
#include <RFlowOptimization/EvaluateRFlow.hpp>
#include <RFlowOptimization/InitialISCAcquisition.hpp>
#include <RFlowOptimization/SessionLocalization.hpp>
#include <Optimization/MultiSession/MultiCascade.hpp>
#include <Optimization/MultiSession/MultiSessionOptimization.hpp>
#include <Optimization/MultiSession/MultiSessionIterativeSmoothingAndRefinement.hpp>
#include <RFlowEvaluation/ForBMVCFigure.hpp>
#include <RFlowEvaluation/AlignVisibilitySet.hpp>
#include <RFlowEvaluation/AlignICPImagePairs.hpp>

using namespace std;

vector<string> cluster_paths = {"/home/shaneg/results/", "/home/shaneg/data/VBags/", "/home/shaneg/data/Lakeshore_KLT/", "/home/shaneg/data/bike_datasets/"};
vector<string> lab_paths = {"/cs-share/dream/results_consecutive/", "/mnt/tale/cedricp/VBags/", "/mnt/tale/shaneg/Lakeshore_KLT/", "/mnt/tale/shaneg/bike_datasets/"};
vector<string> home_paths = {"/Users/shane/Documents/research/", "/Volumes/SAMSUNG/VBags/", "/Users/shane/Documents/research/data/Lakeshore_KLT/", ""};


int main(int argc, char *argv[]) {
    if(argc<4) {
        std::cout << "need 3 input arguments" << std::endl;
        exit(-1);
    }
    std::cout << "starting program" << std::endl;
    
    string results_dir = home_paths[0]; //maps_debug/ "/Users/shane/Documents/research/";//
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
        SessionLocalization acq(axisptz, argv[1], query_loc, pftbase, results_dir);
        acq.Run(start);
        break;}
    case 1:{
        Camera axisptz = ParseBoatSurvey::GetCamera();
        //argv[2] is used here to specify an optimize-up-to date. When not specified, all possible dates are used.
//        MultiSessionOptimization mso(axisptz, results_dir, pftbase, argv[2]);
        MultiCascade mso(axisptz, results_dir, pftbase, argv[1]);
//        MultiSessionIterativeSmoothingAndRefinement mso(axisptz, results_dir, pftbase, argv[2]);
//        mso.IterativeMerge();
        mso.CreateReferenceSet();
//        mso.SetDryRun();
//        mso.IterativeMerge();
        break;}
    case 2:{
        //keep date 1 the same and change date 2 to have the images be aligned at the same places. 
        Camera axisptz = ParseBoatSurvey::GetCamera();
        AlignVisibilitySet avs(axisptz, argv[1], argv[2], pftbase, query_loc, results_dir, visibility_dir);
        avs.Visibility();
//        avs.AlignMapBasedViewset();
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
    case 11:{
        std::vector<std::string> dates = {"140106", "140117", "140122", "140129", "140205", "140314", "140409", "140416", "140424", "140502", "140515", "140528", "140606", "140613", "140625", "140707", "140711", "140718", "140723", "140730", "140812", "140828", "140904", "140911", "140919", "140926", "141003", "141024", "141029", "141107", "141114", "141121", "141128", "141215", "141222"};
        //std::vector<std::string > dates = {"140106", "140117", "140122", "140129", "140205", "140314", "140416"};
        Camera axisptz = ParseBoatSurvey::GetCamera();
        ForBMVCFigure forfig(axisptz, dates, pftbase, query_loc, results_dir);
//        forfig.AlignSection(150, "140106", "140416", 0);
        forfig.GetAlignmentAtSection(argv[1], stoi(argv[2]), false);
        break;}
    case 12:{
        int start = -1;
        if(argc == 5) start = atoi(argv[4]);
        Camera axisptz = ParseBoatSurvey::GetCamera();
        SessionLocalization acq(axisptz, argv[1], query_loc, pftbase, results_dir, "mapsinpar/");
        acq.Run(start);
        break;}
    case 13:{
        Camera axisptz = ParseBoatSurvey::GetCamera();
        MultiCascade mso(axisptz, results_dir, pftbase, argv[2]);
        mso.SetStaticMaps();
        mso.IterativeMerge();
        break;}
    case 14:{
        Camera axisptz = ParseBoatSurvey::GetCamera();
        InitialISCAcquisition acq(axisptz, argv[1], argv[2], query_loc, pftbase, results_dir, results_dir + "origin/");
        acq.Run();
        break;}
    case 15:{
        std::vector<std::string> dates = {"140106", "140117", "140122", "140129", "140205", "140314", "140409", "140416", "140424", "140502", "140515", "140528", "140606", "140613", "140625", "140707", "140711", "140718", "140723", "140730", "140812", "140821", "140828", "140904", "140911", "140919", "140926", "141003", "141010", "141024", "141029", "141107", "141114", "141121", "141128", "141215", "141222"};
        Camera axisptz = ParseBoatSurvey::GetCamera();
//        AlignICPImagePairs icppairs(axisptz, query_loc, results_dir, pftbase, dates, stoi(argv[4]));
//        icppairs.AlignImagesRFlow(results_dir + "image_pairs.csv", stoi(argv[1]), stoi(argv[2]));
        AlignICPImagePairs icppairs(axisptz, query_loc, results_dir, pftbase, dates);
        icppairs.GetResults();
        break;}
    }
    


/*
      FlickeringDisplay fd(argv[1], argv[2]);
      string dir = results_dir + argv[1] + "_to_" + argv[2];
      fd.DisplaySurveyComparisonDir(dir);
*/

    return 0;
}
