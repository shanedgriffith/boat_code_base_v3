
#include <BikeSurvey/ParseBikeRoute.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <DataTypes/Camera.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Matrix.h>

#include "GeometricComputerVision.h"
#include "TestBikeSurvey.h"

#include <ImageAlignment/DREAMFlow/ImageOperations.h>

using namespace std;

cv::Scalar TestBikeSurvey::ColorByDistance(double dist){
    int val = 255-min(5*dist,255);
    return CV_RGB(val, val, val);
}

cv::Scalar TestBikeSurvey::ColorByHeight(double z){
    int val = min(abs(5*z),255);
    if(z<0) return CV_RGB(255, 0, 0);
    else return CV_RGB(0, val, 0);
}

void TestBikeSurvey::TestTriangulation(){
    string bdbase = "/mnt/tale/shaneg/bike_datasets/";
    string name = "20160831_171816";
    ParseBikeRoute pbr(bdbase, name);
    Camera nexus = ParseBikeRoute::GetCamera();
    gtsam::Cal3_S2::shared_ptr gtcam = nexus.GetGTSAMCam();
    gtsam::Matrix gtmat = gtcam->matrix();
    
    int one = 500;
    int two = 501;
    gtsam::Pose3 u = pbr.CameraPose(one);
    gtsam::Pose3 v = pbr.CameraPose(two);
    
    vector<double> up = pbr.GetPose(one);
    vector<double> vp = pbr.GetPose(two);
    
    ParseFeatureTrackFile PFT0 = pbr.LoadVisualFeatureTracks(nexus, one);
    ParseFeatureTrackFile PFT1 = pbr.LoadVisualFeatureTracks(nexus, two);
    
    cv::string imagepath = pbr.GetImagePath("bdbase" + name, one);
    cv::Mat img = ImageOperations::Load(imagepath);
    
    cvNamedWindow(window_name);
    
    int last = 0;
    int ci = 0;
    for(int i=0; i<PFT0.ids.size(); i++){
        while(PFT1.ids[ci]<PFT0.ids[i]) ci++;
        if(PFT1.ids[ci] != PFT0.ids[i]) continue;

        gtsam::Point3 pw = ProjectImageToWorld(PFT0.imagecoord[i], u, PFT1.imagecoord[j], v, gtmat);
        
        cv::Point2f p = cv::Point2f(PFT0.imagecoord[i].x(), PFT0.imagecoord[i].y());
        double dist = u.range(PFT0.imagecoord[i]);
        
        circle(img, p, 5, ColorByHeight(dist), -1, 8, 0);
        circle(img, p, 3, ColorByDistance(dist), -1, 8, 0);
    }
    
    imshow(window_name, img);
    char c = cvWaitKey();
    if(c=='q') {
        cvDestroyAllWindows();
        exit(0);
    }
    
    cvDestroyAllWindows();
}





















































































