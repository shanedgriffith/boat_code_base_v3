#include <iostream>

#include <Optimization/SurveyOptimizer.h>
#include <RFlowOptimization/RFlowSurveyOptimizer.hpp>
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/Camera.hpp>
#include <RFlowOptimization/EvaluateRFlow.hpp>
#include <RFlowOptimization/AcquireISConstraints.hpp>
#include <RFlowEvaluation/AlignVisibilitySet.hpp>
#include <Visualizations/FlickeringDisplay.h>
#include <FileParsing/ParseSurvey.h>
#include <BoatSurvey/ParseBoatSurvey.hpp>
#include <BikeSurvey/PreprocessBikeRoute.hpp>
#include <BikeSurvey/ParseBikeRoute.hpp>
#include <BikeSurvey/ImageModification.hpp>
#include <Tests/TestBikeSurvey.h>


using namespace std;

const string mnt = "/mnt";
const string results_dir = mnt + "/tale/shaneg/results/isc/";
const string query_loc = mnt + "/tale/cedricp/VBags";
const string pftbase = mnt + "/tale/shaneg/Lakeshore_KLT/";
const string poses_loc = mnt + "/tale/shaneg/results/visibility_poses/all/";
string visibility_dir = mnt + "/tale/shaneg/results/visibility_poses/all/";
const string optimized_datasets = mnt + "/tale/shaneg/results/VerifiedOpt/";

int main(int argc, char *argv[]) {
    if(argc<4) {
        std::cout << "need 3 input arguments" << std::endl;
        exit(-1);
    }
    std::cout << "starting program" << std::endl;


    
    
    int prog = atoi(argv[3]);
    switch(prog){
    case 0:{
        int start = -1;
        if(argc == 5) start = atoi(argv[4]);
        Camera axisptz = ParseBoatSurvey::GetCamera();
        AcquireISConstraints acq(axisptz, argv[1], query_loc, pftbase, optimized_datasets, results_dir);
        acq.Run(start);
        break;}
    case 1:{
        Camera axisptz = ParseBoatSurvey::GetCamera();
        RFlowSurveyOptimizer ra(axisptz, argv[1], results_dir, pftbase);
        ra.IterativeMerge();
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
