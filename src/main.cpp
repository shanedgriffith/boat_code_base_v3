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
#include <Optimization/MultiSession/LocalizeSessionToSet.hpp>
#include <RFlowOptimization/LocalizedPoseData.hpp>
#include <Tests/TestRuntime.hpp>
#include <Optimization/SingleSession/EvaluateSLAM.h>
#include <Visualizations/ManualImageCorrespondence.hpp>
#include <Visualizations/ManualKeypointCorrespondence.hpp>

using namespace std;

vector<string> cluster_paths = {"/home/shaneg/results/", "/home/shaneg/data/VBags/", "/home/shaneg/data/Lakeshore_KLT/", "/home/shaneg/data/bike_datasets/"};
vector<string> lab_paths = {"/cs-share/dream/results_consecutive/", "/mnt/tale/cedricp/VBags/", "/mnt/tale/shaneg/Lakeshore_KLT/", "/mnt/tale/shaneg/bike_datasets/"};
//vector<string> home_paths = {"/Users/shane/Documents/research/", "/Volumes/SAMSUNG/VBags/", "/Users/shane/Documents/research/data/Lakeshore_KLT/", ""};
vector<string> home_paths = {"/Volumes/SAMSUNG/Data/", "/Volumes/SAMSUNG/Data/VBags/", "/Volumes/Untitled/data/Lakeshore_KLT/", ""};
//"/Volumes/SAMSUNG/Data/Lakeshore_KLT/"

int main(int argc, char *argv[]) {
    if(argc<4) {
        std::cout << "need 3 input arguments" << std::endl;
        exit(-1);
    }
    std::cout << "starting program" << std::endl;
    
    string results_dir = home_paths[0]; //maps_debug/ ""/Users/shanehome/Documents/Research/";//
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
    std::cout << "running program " << prog << std::endl;
    switch(prog){
    case -1:{
        TestRuntime tr(results_dir + "origin/");
        tr.RunningProcesses();
        break;}
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
        std::vector<std::string> dates = {"140106", "140117", "140122", "140129", "140205", "140314", "140409", "140416", "140424", "140502", "140515", "140528", "140606", "140613", "140625", "140707", "140711", "140718", "140723", "140730", "140812", "140821", "140828", "140904", "140911", "140919", "140926", "141003", "141010", "141024", "141029", "141107", "141114", "141121", "141128", "141215", "141222"};
        //std::vector<std::string> dates = {"140106", "140117", "140122", "140129", "140205", "140314", "140409", "140416", "140424", "140502", "140515", "140528", "140606", "140613", "140625", "140707", "140711", "140718", "140723", "140730", "140812", "140821", "140828", "140904", "140911", "140919", "140926", "141003", "141010", "141024", "141029", "141107", "141114", "141121", "141128", "141215", "141222", "150111", "150216", "150226", "150305", "150312", "150320", "150327", "150401", "150408", "150414", "150421", "150429", "150505", "150522", "150608", "150620", "150625", "150701", "150708", "150723", "150730", "150806", "150813", "150820", "150827", "150902", "150910", "150918", "150929", "151008", "151019", "151027", "151105", "151111", "151118", "151127", "151209", "151214", "151221", "160201", "160211", "160216", "160305", "160314", "160321", "160401", "160407", "160411", "160418", "160426", "160502", "160524", "160601", "160606", "160616", "160620", "160715", "160719", "160725", "160801", "160808", "160816", "160821", "160829", "160906", "160912", "160923", "160927", "161003", "161010", "161018", "161114", "161123", "161127", "161216", "161223", "170217", "170223", "170303", "170307", "170313", "170320", "170327", "170403", "170411", "170419", "170424", "170515", "170626", "170725", "170904", "171004", "171030"};
        Camera axisptz = ParseBoatSurvey::GetCamera();
        ForBMVCFigure forfig(axisptz, dates, pftbase, query_loc, results_dir, 8);
        //        forfig.AlignSection(150, "140106", "140416", 0);
        forfig.MakeTimelapse(argv[1], stoi(argv[2]), false);
//        forfig.MakeTimelapse("140424", 1443, false);
        break;}
    case 12:{
        int start = -1;
        if(argc == 5) start = atoi(argv[4]);
        Camera axisptz = ParseBoatSurvey::GetCamera();
        SessionLocalization acq(axisptz, argv[1], query_loc, pftbase, results_dir);
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
//        std::vector<std::string> dates = {"140106", "140117", "140122", "140129", "140205", "140314", "140409", "140416", "140424", "140502", "140515", "140528", "140606", "140613", "140625", "140707", "140711", "140718", "140723", "140730", "140812", "140821", "140828", "140904", "140911", "140919", "140926", "141003", "141010", "141024", "141029", "141107", "141114", "141121", "141128", "141215", "141222", "150111", "150216", "150226", "150305", "150312", "150320", "150327", "150401", "150408", "150414", "150421", "150429", "150505", "150522", "150608", "150620", "150625", "150701", "150708", "150723", "150730", "150806", "150813", "150820", "150827", "150902", "150910", "150918", "150929", "151008", "151019", "151027", "151105", "151111", "151118", "151127", "151209", "151214", "151221", "160201", "160211", "160216", "160305", "160314", "160321", "160401", "160407", "160411", "160418", "160426", "160502", "160524", "160601", "160606", "160616", "160620", "160715", "160719", "160725", "160801", "160808", "160816", "160821", "160829", "160906", "160912", "160923", "160927", "161003", "161010", "161018", "161114", "161123", "161127", "161216", "161223", "170217", "170223", "170303", "170307", "170313", "170320", "170327", "170403", "170411", "170419", "170424", "170515", "170626", "170725", "170904", "171004", "171030"};
        std::vector<std::string> dates = {"140106", "140117", "140122", "140129", "140205", "140314", "140409", "140416", "140424", "140502", "140515", "140528", "140606", "140613", "140625", "140707", "140711", "140718", "140723", "140730", "140812", "140821", "140828", "140904", "140911", "140919", "140926", "141003", "141010", "141024", "141029", "141107", "141114", "141121", "141128", "141215", "141222"};
//        std::vector<std::string> dates = {"150111", "150216", "150226", "150305", "150312", "150320", "150327", "150401", "150408", "150414", "150421", "150429", "150505", "150522", "150608", "150620", "150625", "150701", "150708", "150723", "150730", "150806", "150813", "150820", "150827", "150902", "150910", "150918", "150929", "151008", "151019", "151027", "151105", "151111", "151118", "151127", "151209", "151214", "151221"};
//        std::vector<std::string> dates = {"160201", "160211", "160216", "160305", "160314", "160321", "160401", "160407", "160411", "160418", "160426", "160502", "160524", "160601", "160606", "160616", "160620", "160715", "160719", "160725", "160801", "160808", "160816", "160821", "160829", "160906", "160912", "160923", "160927", "161003", "161010", "161018", "161114", "161123", "161127", "161216", "161223"};
//        std::vector<std::string> dates = {"170217", "170223", "170303", "170307", "170313", "170320", "170327", "170403", "170411", "170419", "170424", "170515", "170626", "170725", "170904", "171004", "171030"};
        
        Camera axisptz = ParseBoatSurvey::GetCamera();
//        AlignICPImagePairs icppairs(axisptz, query_loc, results_dir, pftbase, dates, stoi(argv[4]));
//        icppairs.AlignImagesRFlow(results_dir + "image_pairs.csv", stoi(argv[1]), stoi(argv[2]));
        AlignICPImagePairs icppairs(axisptz, query_loc, results_dir, pftbase, dates);
//        icppairs.PercentLocalizedPoses(dates);
//        icppairs.GetResultsLabels(false);
//        icppairs.GetResultsLabelsICP();
//        icppairs.CompareRFWithICP();
//        icppairs.AlignmentQualityByPlace();
//        icppairs.CreateTimeLapsesForEvaluation();
//        icppairs.AlignmentQualityByPlace_SPECTRUM();
//        icppairs.CheckRF();
//        icppairs.GetResults();
//        icppairs.GetResultsTimelapse("", "");
//        icppairs.LabelTimelapse();
//        icppairs.CreateTimeLapsesForEvaluation();
//        icppairs.AnalyzeTimeLapses();
//        icppairs.CheckSessions();
//        icppairs.ShowMaps();
//        icppairs.LocalizePoseUsingManualLabels("/Users/shane/Documents/research/results/2018_winter/manual_labeling/");
        icppairs.AnalyzeManualLabels("/Users/shane/Documents/research/results/2018_winter/manual_labeling/");
//        icppairs.CountPosesWithALocalization();
//        icppairs.PareComparisonFile();
//        icppairs.AnalyzeAlignmentQualityTrend();
//        icppairs.AlignImagesWarped();
//        std::string argnum(argv[1]);
//        std::string argdate(argv[2]);
//        icppairs.GetResultsTimelapse(argnum, argdate);
//        icppairs.AlignTimelapsesSFlow(argv[1]);
        break;}
    case 16:{
        Camera axisptz = ParseBoatSurvey::GetCamera();
        LocalizeSessionToSet lss(axisptz, results_dir + "maps/", results_dir + "localized_maps/", argv[1], pftbase, 100);
        lss.LocalizeSession();
        break;}
    case 17:{
        Camera axisptz = ParseBoatSurvey::GetCamera();
        LocalizedPoseData lpd = LocalizedPoseData::Read("/Volumes/SAMSUNG/Data/localized_maps/161223/localizations/673_141114.loc");
        lpd.CheckLPD(axisptz, pftbase, results_dir, query_loc);
        break;}
    case 18:{
            Camera axisptz = ParseBoatSurvey::GetCamera();
            std::vector<std::string> dates = {"140106", "140117", "140122", "140129", "140205", "140314", "140409", "140416", "140424", "140502", "140515", "140528", "140606", "140613", "140625", "140707", "140711", "140718", "140723", "140730", "140812", "140821", "140828", "140904", "140911", "140919", "140926", "141003", "141010", "141024", "141029", "141107", "141114", "141121", "141128", "141215", "141222"};
            for(int i=0; i<dates.size(); ++i)
            {
                EvaluateSLAM es(axisptz, dates[i], results_dir + "maps/");
                es.ErrorForSurvey(pftbase, false);
                es.PrintTots();
            }
        break;}
    case 19:{
            Camera axisptz = ParseBoatSurvey::GetCamera();
//              ManualImageCorrespondence mic(query_loc,
            ManualKeypointCorrespondence mic(axisptz,
                                             query_loc,
                                             pftbase,
                                              "/Volumes/Untitled/data/maps_only/maps_only_2014/",
                                              "/Users/shane/Documents/research/results/2018_winter/manual_labeling/");
//            mic.GetLocalizationList();
//        mic.RunManualCorrespondence("140821", 222, "140416", 131); //couldn't
//        mic.RunManualCorrespondence("140926", 613, "140424", 107);
//        mic.RunManualCorrespondence("141003", 1066, "140926", 1115);
//        mic.RunManualCorrespondence("140424", 1331, "140106", 1555);
        mic.RunManualCorrespondence("140314", 1920, "140911", 2275);
        }
    }
    
    
    
/*
      FlickeringDisplay fd(argv[1], argv[2]);
      string dir = results_dir + argv[1] + "_to_" + argv[2];
      fd.DisplaySurveyComparisonDir(dir);
*/
    
    return 0;
}
