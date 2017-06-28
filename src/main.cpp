#include <iostream>

#include <Optimization/SurveyOptimizer.h>
#include <RFlowOptimization/RFlowSurveyOptimizer.hpp>
#include <FileParsing/FileParsing.hpp>
//#include <Tests/TestRFlowOptimization.hpp>
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

const string results_dir = "/cs-share/dream/results_consecutive/";
const string query_loc = "/mnt/tale/cedricp/VBags/";
const string aux_to_pft_dir = "/cs-share/dream/aux_to_pft/";
const string pftbase = "/mnt/tale/shaneg/Lakeshore_KLT/";
const string poses_loc = "/mnt/tale/shaneg/results/visibility_poses/all/";
string visibility_dir = "/mnt/tale/shaneg/results/visibility_poses/all/";
const string optimized_datasets = "/mnt/tale/shaneg/results/VerifiedOpt/";

int main(int argc, char *argv[]) {
    if(argc<4) {
         std::cout << "need 3 input arguments" << std::endl;
        //std::cout << "input the next survey date to optimize."<<std::endl;
        exit(-1);
    }

    //vector<std::string> all = {"140106", "140117", "140122", "140129", "140205", "140314", "140502", "140515", "140528", "140606", "140613", "140625", "140711", "140730", "140812", "140911", "140919", "140926", "141003", "141029", "141107", "141114", "141128", "150216"};

    vector<std::string> all = {"140106", "140117", "140122", "140129", "140205", "140314", "140409", "140416", "140424", "140502", "140515", "140528", "140606", "140613", "140625", "140707", "140711", "140718", "140723", "140730", "140812", "140821", "140828", "140904", "140911", "140919", "140926", "141003", "141010", "141024", "141029", "141107", "141114", "141121", "141128", "141215", "141222", "150111", "150216", "150226", "150305", "150312", "150320", "150327", "150401", "150408", "150414", "150421", "150429", "150505", "150522", "150608", "150620", "150625", "150701", "150708", "150723", "150730", "150806", "150813", "150820", "150827", "150902", "150910", "150918", "150929", "151008", "151019", "151027", "151105", "151111", "151118", "151127", "151209", "151214", "151221", "160201", "160211", "160216", "160305", "160314", "160321", "160401", "160407", "160411", "160418", "160426", "160502", "160524", "160601", "160606", "160616", "160620", "160715", "160719", "160725", "160801", "160808", "160816", "160821", "160829", "160906", "160912", "160923", "160927", "161003", "161010", "161018", "161114", "161123", "161127", "161216", "161223", "170217", "170223", "170303", "170307", "170313", "170320", "170327", "170403"};
    
    //vector<std::string> all = {"140911", "140919", "140926", "141003"};
//    TestRFlowOptimization trfo;
//    trfo.TestNewImageAlignment();
//    exit(1);
    
    
    FileParsing::MakeDir(results_dir);
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
