#include <iostream>

#include "BoatSurvey/ParseBoatSurvey.hpp"
#include "VIOCompetition/ParseDroneRun.hpp"
#include "VIOCompetition/PreprocessDroneRun.hpp"
//#include "VIOCompetition/IMUFactorOptimizationTest.hpp"
#include "VisualOdometry/VisualOdometry.hpp"

#include "Optimization/SingleSession/SurveyOptimizer.h"
#include "Optimization/SingleSession/testAngularVelocity.h"
#include <testRobust.hpp>

#include <RFlowOptimization/Nister5Point.h>

using namespace std;

vector<string> cluster_paths = {"/home/shaneg/results/", "/home/shaneg/data/VBags/", "/home/shaneg/data/Lakeshore_KLT/", "/home/shaneg/data/bike_datasets/"};
vector<string> lab_paths = {"/cs-share/dream/results_consecutive/", "/mnt/tale/cedricp/VBags/", "/mnt/tale/shaneg/Lakeshore_KLT/", "/mnt/tale/shaneg/bike_datasets/"};
//vector<string> home_paths = {"/Users/shane/Documents/research/", "/Volumes/SAMSUNG/VBags/", "/Users/shane/Documents/research/data/Lakeshore_KLT/", ""};
vector<string> home_paths = {"/Volumes/Untitled/data/iSAM/", "/Volumes/SAMSUNG/Data/VBags/", "/Volumes/Untitled/data/Lakeshore_KLT/", "/Volumes/Untitled/bikedata/"};
//vector<string> home_paths = {"/Volumes/SAMSUNG/Data/", "/Volumes/SAMSUNG/Data/VBags/", "/Volumes/Untitled/data/Lakeshore_KLT/", "/Volumes/Untitled/bikedata/"};
//"/Volumes/SAMSUNG/Data/Lakeshore_KLT/"

int main(int argc, char *argv[])
{
    testNister5Point();
    
//    testAngularVelocity::compareAngularVelocityToTraj();
////    testAngularVelocity::testReprojectionWithYawDifference();
//    return 1;
    
//    testRobust tr;
//    tr.image();
//    return 0;
    
//    PreprocessDroneRun predavis("/Users/shane/Documents/projects/VIO/datasets/", "outdoor_forward_1_davis_with_gt/");
////    predavis.FindKLTParams();
//    predavis.ProcessRawVideo();
//    exit(1);
    
//    ParseDroneRun davis("/Users/shane/Documents/projects/VIO/datasets/", "outdoor_forward_1_davis_with_gt/");
//    IMUFactorOptimizationTest testIMU(davis);
////    testIMU.testIMUFactor();
//    testIMU.optimizeDroneRun();
//    exit(1);
    
    string query_loc = "/Volumes/Untitled/data/VBags/";//home_paths[1];
    string pftbase = home_paths[2];
    string results_dir = home_paths[0];

    Camera axisptz = ParseBoatSurvey::GetCamera();
//    LocalizePose loc(axisptz);
//    loc.testLocalizePoses();
//    exit(1);
//    loc.debug = true;
//    loc.testP3P();
//    loc.test();
//    loc.testP3PStatic();
    
    std::string date = "140106";

    std::shared_ptr<ParseSurvey> PS = std::make_shared<ParseBoatSurvey>(query_loc, pftbase, date);
    SurveyOptimizer so(axisptz, date, results_dir);
    so.Initialize();
    so.Optimize(PS);
    
//    VisualOdometry vo(axisptz);
////    vo.test3Dto2DVO();
//    vo.test3Dto2DVOWithTriangulation();
    
    return 0;
}
