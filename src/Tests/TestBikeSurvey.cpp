
#include <BikeSurvey/ParseBikeRoute.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <DataTypes/Camera.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

#include "GeometricComputerVision.h"

using namespace std;

void TestBikeSurvey::TestTriangulation(){
    string bdbase = "/mnt/tale/shaneg/bike_datasets/";
    string name = "20160831_171816";
    ParseBikeRoute pbr(bdbase, name);
    Camera cam = ParseBikeRoute::GetCamera();
    gtsam::Cal3_S2::shared_ptr gtcam = GetGTSAMCam();
    Matrix gtmat = gtcam->Matrix();
    
    gtsam::Pose3 u = pbr.CameraPose(500);
    gtsam::Pose3 v = pbr.CameraPose(501);
    
    vector<double> up = pbr.GetPose(500);
    vector<double> vp = pbr.GetPose(500);
    
    ParseFeatureTrackFile PFT0 = PS.LoadVisualFeatureTracks(_cam, 500);
    ParseFeatureTrackFile PFT1 = PS.LoadVisualFeatureTracks(_cam, 501);
    
    int last = 0;
    for(int i=0; i<PFT0.ids.size(); i++){
        for(int j=0; j<PFT1.ids.size(); j++){
            if(PFT0.ids[i] == PFT.ids[j]){
                last = j;
                gtsam::Point3 pw = ProjectImageToWorld(PFT0.imagecoord[i], u, PFT1.imagecoord[j], v, gtmat);
                printf("(%.2lf,%.2lf,%.2lf,%.2lf,%.2lf) (%.2lf,%.2lf,%.2lf,%.2lf,%.2lf) (%.2lf,%.2lf,%.2lf)\n",
                       up[0],up[1],up[2],up[3],up[4],up[5],vp[0],vp[1],vp[2],vp[3],vp[4],vp[5],pw.x(),pw.y(),pw.z());
                break;
            }
        }
        
    }
    
}





















































































