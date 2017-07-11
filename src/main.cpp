#include <iostream>

#include <Optimization/SurveyOptimizer.h>
#include <FileParsing/FileParsing.hpp>
//#include <Tests/TestRFlowOptimization.hpp>
#include <DataTypes/Camera.hpp>
#include <RFlowOptimization/EvaluateRFlow.hpp>
#include <RFlowOptimization/RFlowSurveyOptimizer.hpp>
#include <RFlowOptimization/AcquireISConstraints.hpp>
#include <RFlowEvaluation/AlignVisibilitySet.hpp>
#include <Visualizations/FlickeringDisplay.h>
#include <FileParsing/ParseSurvey.h>
#include <BoatSurvey/ParseBoatSurvey.hpp>
#include <BikeSurvey/PreprocessBikeRoute.hpp>
#include <BikeSurvey/ParseBikeRoute.hpp>
#include <BikeSurvey/ImageModification.hpp>
#include <Tests/TestBikeSurvey.h>
#include <RFlowOptimization/MultiSessionOptimization.hpp>


using namespace std;

////cluster:
//const string results_dir = "/home/shaneg/results/";
//const string query_loc = "/home/shaneg/data/VBags";
//const string pftbase = "/home/shaneg/data/Lakeshore_KLT/";
/*
//
const string mnt = "/mnt";
const string results_dir = "/cs-share/dream/results_consecutive/"; //mnt + "/tale/shaneg/results/isc";
const string query_loc = mnt + "/tale/cedricp/VBags";
const string pftbase = mnt + "/tale/shaneg/Lakeshore_KLT/";*/
//const string visibility_dir = mnt + "/tale/shaneg/results/visibility_poses/all/";

vector<string> cluster_paths = {"/home/shaneg/results/", "/home/shaneg/data/VBags", "/home/shaneg/data/Lakeshore_KLT/"};
vector<string> lab_paths = {"/cs-share/dream/results_consecutive/", "/mnt/tale/cedricp/VBags", "/mnt/tale/shaneg/Lakeshore_KLT/"};

int main(int argc, char *argv[]) {
    if(argc<4) {
        std::cout << "need 3 input arguments" << std::endl;
        exit(-1);
    }
    std::cout << "starting program" << std::endl;


//    TestRFlowOptimization trfo;
//    trfo.TestNewImageAlignment();
//    exit(1);
    
    string results_dir = lab_paths[0];
    string query_loc = lab_paths[1];
    string pftbase = lab_paths[2];
    string visibility_dir = "/mnt/tale/shaneg/results/visibility_poses/all/";
    if(argc>4){
        results_dir = cluster_paths[0];
        query_loc = cluster_paths[1];
        pftbase = cluster_paths[2];
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
        MultiSessionOptimization mso(axisptz, results_dir, pftbase);
        mso.SetDryRun();
        mso.IterativeMerge();
        //RFlowSurveyOptimizer ra(axisptz, argv[1], results_dir, pftbase);
        //ra.IterativeMerge();
        break;}
    case 2:{
        Camera axisptz = ParseBoatSurvey::GetCamera();
        AlignVisibilitySet avs(axisptz, argv[2], argv[1], pftbase, query_loc, results_dir, visibility_dir);
        avs.Visibility();
        break;}
    case 3:{
        FlickeringDisplay fd(argv[2], argv[1]);
        string dir = results_dir + argv[2] + "_to_" + argv[1] + "/";
        std::cout << "dir: " << dir << std::endl;
        fd.CompareFromDir(dir);
        break;}
    case 5:{
        PreprocessBikeRoute pbr("/mnt/tale/shaneg/bike_datasets/", argv[1]);
        pbr.Preprocess();
        break;}
    case 6:{
        ParseBikeRoute pbr("/mnt/tale/shaneg/bike_datasets/", argv[1]);
        Camera nexus = ParseBikeRoute::GetCamera();
        SurveyOptimizer so(nexus, argv[1], results_dir);
        so.Initialize();
        so.Optimize(pbr);
        break;}
    case 7:{
        ParseBoatSurvey PS(query_loc + argv[1], pftbase + argv[1]);
        Camera axisptz = ParseBoatSurvey::GetCamera();
        SurveyOptimizer so(axisptz, argv[1], results_dir, true);
        so.Initialize();
        so.Optimize(PS);
        break;}
    case 8:{
        ParseBoatSurvey PS(query_loc + argv[1], pftbase + argv[1]);
        PS.PlayPoses();
        break;}
    case 9:{
        TestBikeSurvey tbs;
        tbs.TestVO();
        //tbs.TestVisualOdometry();
        //tbs.TestTriangulation();
        break;}
    }

/*    int start = -1;
    if(argc>2) start = atoi(argv[2]);
    AcquireISConstraints acq(kingfisher, argv[1]);
    acq.Run(start);
*/
/*     string ref = "140625";
      AlignVisibilitySet avs(kingfisher, ref, argv[1]);
      avs.Visibility(); 
     exit(1);
*//*     
      FlickeringDisplay fd(argv[1], argv[2]);
      string dir = results_dir + argv[1] + "_to_" + argv[2];
      fd.DisplaySurveyComparisonDir(dir);
*/
//    EvaluateRFlow erf(kingfisher, argv[1]);
//    erf.debug = true;
//    erf.Evaluate();

    return 0;
}
