#include <iostream>

#include "Optimization/EvaluateSLAM.h"

#include "ImageAlignment/FlowFrameworks/MultiThreadedAlignment.h"
#include "Visualizations/FlickeringDisplay.h"
#include "Visualizations/MapAlignmentAccuracy.h"
#include "Visualizations/FeatureTracks.hpp"
#include "Optimization/SurveyOptimizer.h"
#include "Optimization/InterSurveyOptimizer.hpp"
#include "Optimization/EvaluateSLAM.h"

#include "ImageAlignment/ConsistentFlow/FlowWebFramework.hpp"
#include "FileParsing/BruteForceMatchFile.hpp"

#include "Tests/TestAlignmentEnergyThreshold.hpp"

#include <ImageAlignment/FlowFrameworks/BruteForceAlignment.hpp>
#include <ImageAlignment/FlowFrameworks/GPSAlignment.hpp>
#include <Optimization/SmallSectionOptimizer.hpp>

#include <Clustering/TestXMeans.hpp>

#include <Tests/TestMatchFlowFilter.hpp>

#include <ImageAlignment/FlowFrameworks/ReprojectionAlignment.hpp>
#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <DataTypes/Camera.hpp>

#include <Tests/TestOptimizedPoses.hpp>
#include <Tests/DebugMapFlowFiles.hpp>

#include <Tests/CreateTimeLapse.hpp>

#include <Evaluation/MultiManualCorrespondence.hpp>

/*for the cluster.*/
//extern const string base = "/home/shaneg/results/alignment/";
//extern const string optimized_datasets = "/home/shaneg/results/VerifiedOpt/";
//extern const string siftloc = "/home/shaneg/data/";
//extern const string query_loc = "shaneg@tale.georgiatech-metz.fr:~/../cedricp/VBags";
//extern const string poses_loc = "/home/shaneg/results/visibility_poses";

/*for my computer.*/
//extern const string base = "/Users/shane/Desktop/temp/csiftflow/ConsistencyExperiments/";

extern const string base = "/home/shane/Desktop/temp/csiftflow/LakeshoreExperiments/55/";
//extern const string query_loc = "/Users/shane/Documents/research - current stuff/data";
extern const string query_loc = "/media/shane/SAMSUNG/Research/DREAM/VBags";
//extern const string query_loc = "/media/shane/tale/cedricp/VBags";
extern const string poses_loc = "/Users/shane/Desktop/temp/visibility_poses/all";
extern const string siftloc = query_loc + "/";
//extern const string siftloc = "/media/shane/tale/shaneg/Lakeshore_KLT/";
//extern const string optimized_datasets = "/Users/shane/Desktop/temp/iros/small1411-section/";//for the nearby surveys with mostly consistent map.
extern const string optimized_datasets = "/home/shane/Desktop/temp/iros/maps/small2030-section/";
//extern const string optimized_datasets = "/Users/shane/Desktop/temp/opt/verified/";



//extern const string optimized_datasets = "/Users/shane/Desktop/temp/opt/cluster/";//"/Users/shane/Documents/research - current stuff/results/optimization/";
//extern const string optimized_datasets = "/Users/shane/Desktop/temp/debug_optimization/";//opt/verified/";
//extern const string optimized_datasets = "/Users/shane/Desktop/temp/csiftflow/LakeshoreExperiments/55/140625-140613/";//"/Users/shane/Documents/research - current stuff/results/optimization/";
//extern const string optimized_datasets = "/Users/shane/Desktop/temp/csiftflow/LakeshoreExperiments/55/";
//extern const string optimized_datasets = "/Users/shane/Desktop/temp/opt/verified/";
//extern const string query_loc = "/Volumes/SAMSUNG/Research/DREAM/VBags";
//extern const string query_loc = "shaneg@tale.georgiatech-metz.fr:~/../cedricp/VBags";
//extern const string query_loc = "griffith_sha@172.21.15.144:/media/griffith_sha/SAMSUNG/Research/DREAM/VBags";//daubigney

#include <Visualizations/ColoredMaps.hpp>

int main(int argc, char *argv[])
{
    std::cout << "\n\n\n" << std::endl;
    std::string date1="140613", date2="140625";
    
    vector<std::string> alldates = {"140106", "140117", "140122", "140129", "140205", "140314", "140502", "140515", "140528", "140606", "140613", "140625", "140711", "140730", "140812", "140911", "140919", "140926", "141003", "141029", "141107", "141114", "141128", "150216"};
    
    vector<std::string> sequence1 = {"140613", "140625"};
    vector<std::string> lastdate = {"150216"};
    vector<std::string> two = {"140122", "140129"};
    vector<std::string> three = {"140911", "140919", "140926"};
    vector<std::string> custom = {"140911", "140919", "140926", "141003"};
    
    vector<std::string> consist = {"140106", "140117", "140122", "140129", "140205", "140314"};// "140613", "140919", "141029", "141114", "141128", "150216"
    
    vector<std::string> all = {"140106", "140117", "140122", "140129", "140205", "140314", "140502", "140515", "140528", "140606", "140613", "140625", "140711", "140730", "140812", "140911", "140919", "140926", "141003", "141029", "141107", "141114", "141128", "150216"};
    vector<std::string> small = {"140528", "140606", "140613", "140625"};
    vector<std::string> consecutive = {"140502", "140528", "140625", "140730", "140812", "140926", "141003", "141029", "141128", "150216"};

    Camera kingfisher(759.308012, 690.43984, 370.91545, 250.909693, 704, 480);
    kingfisher.SetDistortion(-0.302805, 0.171088, 0.001151, -0.00038, 0.0);

//    MultiManualCorrespondence mmc("/home/shane/Desktop/temp/cvpr/labels/validation-align-jjm.txt");

    SurveyOptimizer so(kingfisher, "140911");
    so.Initialize();
    so.Optimize();

//    ConsistencyThreshold(true);//
//    AlignmentEnergy();

//    CreateTimeLapse(0);
//    CreateTimeLapse(25);
//    CreateTimeLapse(75);
//    CreateTimeLapse(128);
//    CreateTimeLapse(175);
//    CreateTimeLapse(225);
//    CreateTimeLapse(275);
//    CreateTimeLapse(325);
//    CreateTimeLapse(375);
//    CreateTimeLapse(425);
//    CreateTimeLapse(475);
//    CreateTimeLapse(525);
//    CreateTimeLapse(575);
//    CreateTimeLapse(625);
//    CreateTimeLapse(645);
    
//    ColoredMaps cm;
////    vector<string> forfig = {"140129", "140314", "140613", "140911"};
////    vector<string> forfig = {"140129", "140911"};
//    cm.CreateMap(all, "/Users/shane/Desktop/temp/iros/maps/small1-section/");
//    cm.SaveMap("/Users/shane/Desktop/bmvcfig_all.vtp");
    
//    CreateTimeLapse(0);
//    CreateTimeLapse(25);
//    CreateTimeLapse(75);
//    CreateTimeLapse(128);
//    CreateTimeLapse(175);
//    CreateTimeLapse(225);
//    CreateTimeLapse(275);
//    CreateTimeLapse(325);
//    CreateTimeLapse(375);
//    CreateTimeLapse(425);
//    CreateTimeLapse(475);
//    CreateTimeLapse(525);
//    CreateTimeLapse(575);
//    CreateTimeLapse(625);
//    CreateTimeLapse(645);
    
//
//    DebugMapFlowFiles dmff;
//    dmff.CompareGeneratedFiles();
    
//        TestOptimizedPosees top;
//        top.Displacement();

//    GPSAlignment gpsa(base, 8);
//    gpsa.AlignSurveys("140613", alldates, 1);

    //    TestOptimizedPosees top;
    //    top.Displacement();
    
//        ReprojectionAlignment ra(custom, "140911", "141003", 8);
//        ra.EvaluateRFLowAndMaskAndEPIAndSFlow(custom);
//        ra.EvaluateGStatistic(alldates);
//        ra.DenseSurveyAlignment("140911", "141003");
    
    //    TestMatchFlowFilter::TestFeatureMatching(alldates);
    //    TestMatchFlowFilter::TestFeatureElimination("140613", "140625");
    
    //    ReprojectionFlow::TestGstat();
    
    //TODO: this code should be part of ConsistentAlignmentLog
    //    ConsistentAlignment ca(date1, date2, 8, 0);
    //    vector<AlignmentResult> dataassociation = ca.TopLevelSurveyAlignment();
    //    for(int i=0; i<dataassociation.size(); i++){
    //        string directory = base + date2 + "-" + date1 + "/alignment_res/";
    //        mkdir(directory.c_str(), (mode_t) (S_IRWXU | S_IRWXG | S_IRWXO));
    //        directory += to_string(i) + "/";
    //        dataassociation[i].Save(directory);
    //    }
    
    //    FlowWebFramework fwf;
    ////    fwf.MakeImageSet("140625", dates);
    //    fwf.DisplayFlowWebTimeLapse("/Users/shane/Desktop/temp/iros/data/1/64/", "/Users/shane/Desktop/temp/iros/results/1/64/dsp_flows/");
    
    //    TestAlignmentEnergyThreshold taet("/Users/shane/Desktop/temp/sequence_alignment_results/sift_flow_results/");
    //    taet.ViabilityOfThreshold(sequence1, 1150000);
    
    //    BruteForceAlignment::TestMappedFlowFile("/Users/shane/Desktop/temp/iros/iros_flows/flows_140625_140613/9555_43720.flow");
    //    BruteForceAlignment::TestMappedFlowFile("/Users/shane/Desktop/temp/csiftflow/LakeshoreExperiments/55/flows_140625_140613/8955_43145.flow");//good
    //    BruteForceAlignment::TestMappedFlowFile("/Users/shane/Desktop/temp/csiftflow/LakeshoreExperiments/55/flows_140613_140625/43145_8955.flow");
    //    BruteForceAlignment bfa(base, 1);
    //    bfa.AlignSurveys(date1, two);
    
    //    BruteForceMatchFile::DebugAlignmentThreshold(alldates, 1150000);
    
    //    SmallSectionOptimizer sso(3);
    //
    //    sso.SetDates(all);
    ////    sso.OptimizeSurveySection();
    //    sso.ReprojectionError();
    //    sso.GetImageVariance();
    //    sso.GetPointVariance();
    ////    sso.OptimizeSurveysIndividually();
    
    //    sso.DisplayReprojectionError("140625", "140613", 570);
    //    for(int i=0; i<sequence1.size(); i++){
    //        EvaluateSLAM em(query_loc, base + "small-section/", sequence1[i]);
    //        em.Evaluate();
    //    }
    
    //    InterSurveyOptimizer vslam(date1, date2);
    //    SurveyOptimizer vslam(date2);
    //    vslam.Optimize();
    
    //    EvaluateSLAM es(query_loc, optimized_datasets, date2);
    //    es.Evaluate();
    
    //    ConsistentAlignment ca(date1, date2, 1, 0);
    //    ca.TestAlignmentMap();
    
    //    FeatureTrackAlignment fta(date1, date2, 1);
    ////    fta.FindFeatureTrackForTest();
    //    fta.ProjectionConstraintFlow();
    
    
    //    FeatureTracks ft(date2);
    //    ft.Run(10002, 5);
    
    //    GenerateTwoGroupsOfShifts("/Users/shane/Pictures/wallpapers/isle of skye- scotland.jpg");
    //    TestKnownAlignmentCases("/Users/shane/Desktop/smalls/test/");
    //    TestConsistencyMapping("/Users/shane/Desktop/smalls/test/");
    //    TestConsistencyMappingDEBUG("/Users/shane/Desktop/smalls/test/");
    //    TestNoisyConsolidation("/Users/shane/Desktop/smalls/test/");
    //    TestIntraSurveyAlignmentAgreement(date1, date2);
    
    //    for(int i=0; i<lastdate.size(); i++){
    //        std::string date1="140625";
    //        std::string date2=lastdate[i];
    //
    //        ConsistentAlignment ca(date1, date2, 8, 5);
    //        ca.EvaluateBundling();
    //        FlickeringDisplay fd(date1, date2);
    //        fd.DisplaySurveyComparisonCustom(true);//false);//
    //    }
    
    //    SFlowBones sf;
    //    AlignmentResult ar = sf.AlignImages("/Users/shane/Desktop/temp/comparison/temp/0-0006174.jpg",
    //                                        "/Users/shane/Desktop/temp/comparison/temp/2-0006174.jpg", {},
    //                                        "/Users/shane/Desktop/temp/comparison/temp/SFLOW_BRIEF.jpg", "", false);
    
    
    //    ca.AlignSurveys();
    
    //        MultiThreadedAlignment mta(date1, date2, 8);
    //        mta.Visibility(true);
    
    //    FlickeringDisplay fd(date1, date2);
    ////    fd.DisplaySurveyComparison();
    //    fd.DisplaySurveyComparisonCustom(true);//false);//
    
    cout << "\nFinished!" << endl;
    return 0;
}
