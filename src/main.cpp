#include <iostream>


#include "VIOCompetition/ParseDroneRun.hpp"
#include "VIOCompetition/PreprocessDroneRun.hpp"
#include "VIOCompetition/IMUFactorOptimizationTest.hpp"


using namespace std;

vector<string> cluster_paths = {"/home/shaneg/results/", "/home/shaneg/data/VBags/", "/home/shaneg/data/Lakeshore_KLT/", "/home/shaneg/data/bike_datasets/"};
vector<string> lab_paths = {"/cs-share/dream/results_consecutive/", "/mnt/tale/cedricp/VBags/", "/mnt/tale/shaneg/Lakeshore_KLT/", "/mnt/tale/shaneg/bike_datasets/"};
//vector<string> home_paths = {"/Users/shane/Documents/research/", "/Volumes/SAMSUNG/VBags/", "/Users/shane/Documents/research/data/Lakeshore_KLT/", ""};
vector<string> home_paths = {"/Volumes/Untitled/data/", "/Volumes/SAMSUNG/Data/VBags/", "/Volumes/Untitled/data/Lakeshore_KLT/", "/Volumes/Untitled/bikedata/"};
//vector<string> home_paths = {"/Volumes/SAMSUNG/Data/", "/Volumes/SAMSUNG/Data/VBags/", "/Volumes/Untitled/data/Lakeshore_KLT/", "/Volumes/Untitled/bikedata/"};
//"/Volumes/SAMSUNG/Data/Lakeshore_KLT/"



int main(int argc, char *argv[])
{
    
//    PreprocessDroneRun predavis("/Users/shane/Documents/projects/VIO/datasets/", "outdoor_forward_1_davis_with_gt/");
////    predavis.FindKLTParams();
//    predavis.ProcessRawVideo();
//    exit(1);
    
    ParseDroneRun davis("/Users/shane/Documents/projects/VIO/datasets/", "outdoor_forward_1_davis_with_gt/");
    IMUFactorOptimizationTest testIMU(davis);
//    testIMU.testIMUFactor();
    testIMU.optimizeDroneRun();
    exit(1);
    
    
    return 0;
}
