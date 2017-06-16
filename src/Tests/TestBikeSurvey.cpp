
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
#include <VisualOdometry/VisualOdometry.hpp>


using namespace std;

cv::Scalar TestBikeSurvey::ColorByDistance(double dist){
    int val = 255-min(255./10*dist,255.);
    return CV_RGB(val, val, val);
}

cv::Scalar TestBikeSurvey::ColorByHeight(double z){
    int val = min(abs(255./10*z),255.);
    if(z<0) return CV_RGB(val, 0, 0);
    return CV_RGB(0, val, 0);
}

cv::Scalar TestBikeSurvey::GetLandmarkColor(int id){
    if(id==-1) id=0;
    int red = (71*(id%10) + id%255)%255;
    int green = (111*(id%10) + (2*id)%255)%255;
    int blue = (27*(id%10) + (3*id)%255)%255;
    return CV_RGB(red, green, blue);
}

std::vector<double> TestBikeSurvey::YPRToRotationMatrix(double y, double p, double r){
    /*Converts ypr to a rotation matrix.
     see http://planning.cs.uiuc.edu/node102.html
     */
    
    std::vector<double> R = {cos(y)*cos(p), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r),
                            sin(y)*cos(p), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r),
                            -sin(p), cos(p)*sin(r), cos(p)*cos(r)};
    return R;
}

std::vector<double> TestBikeSurvey::ComposeRotationMatrices(std::vector<double> A, std::vector<double> B){
    std::vector<double> R = {
        A[0]*B[0]+A[1]*B[3]+A[2]*B[6],
        A[0]*B[1]+A[1]*B[4]+A[2]*B[7],
        A[0]*B[2]+A[1]*B[5]+A[2]*B[8],
        A[3]*B[0]+A[4]*B[3]+A[5]*B[6],
        A[3]*B[1]+A[4]*B[4]+A[5]*B[7],
        A[3]*B[2]+A[4]*B[5]+A[5]*B[8],
        A[6]*B[0]+A[7]*B[3]+A[8]*B[6],
        A[6]*B[1]+A[7]*B[4]+A[8]*B[7],
        A[6]*B[2]+A[7]*B[5]+A[8]*B[8]};
    return R;
}

std::vector<double> TestBikeSurvey::RotationMatrixToRPY(std::vector<double> R){
    std::vector<double> rpy = {atan2(R[7], R[8]), atan2(-1*R[6], sqrt(R[7]*R[7]+R[8]*R[8])), atan2(R[3],R[0])};
    return rpy;
}

gtsam::Pose3 TestBikeSurvey::CameraPose(std::vector<double> p){
    return gtsam::Pose3(gtsam::Rot3::ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

std::vector<double> TestBikeSurvey::TransformPose(std::vector<double> p, int m1, int m2, int m3) {
    vector<double> cam = YPRToRotationMatrix(p[5], p[4], p[3]);
    vector<double> align_with_world = YPRToRotationMatrix(m1*M_PI_2, m2*M_PI_2, m3*M_PI_2);
    std::vector<double> R = ComposeRotationMatrices(cam, align_with_world);
    vector<double> RPY = RotationMatrixToRPY(R);
    return {p[0], p[1], p[2], RPY[0], RPY[1], RPY[2]};
}

void TestBikeSurvey::TestTriangulation(){
    string bdbase = "/mnt/tale/shaneg/bike_datasets/";
    string name = "20160831_171816";
    ParseBikeRoute pbr(bdbase, name);
    Camera nexus = ParseBikeRoute::GetCamera();
    gtsam::Cal3_S2::shared_ptr gtcam = nexus.GetGTSAMCam();
    gtsam::Matrix gtmat = gtcam->matrix();
    
//    VisualOdometry vo(nexus);
    int one = 1275;
    int two = 1277;
    vector<double> ub, vb;
    std::string imagepath;
    ParseFeatureTrackFile PFT0(nexus, bdbase + name, one);
    ParseFeatureTrackFile PFT1(nexus, bdbase + name, two);
    
    const char* window_name = "test poses using point triangulation";
    cvNamedWindow(window_name);
    
    bool updateset=true;
    int m1=0;
    int m2=0;
    int m3=0;
    while(1) {
        if(updateset){
            ub = pbr.GetPose(one);
            vb = pbr.GetPose(two);
            printf("pose u (%lf,%lf,%lf,%lf,%lf,%lf)\n",ub[0],ub[1],ub[2],ub[3],ub[4],ub[5]);
            printf("pose v (%lf,%lf,%lf,%lf,%lf,%lf)\n",vb[0],vb[1],vb[2],vb[3],vb[4],vb[5]);
            
            PFT0.Next(one);
            PFT1.Next(two);
            
            
            imagepath = ParseSurvey::GetImagePath(bdbase + name, one);
            updateset = false;
        }
        
//        gtsam::Pose3 vop = vo.PoseFromEssential(PFT0, PFT1);
//        gtsam::Pose3 u = pbr.CameraPose(one);
//        gtsam::Pose3 v = vop.compose(u);
//        vector<double> ub = PoseToVector(u);
//        vector<double> vb = PoseToVector(v);
//        printf("pose u (%lf,%lf,%lf,%lf,%lf,%lf)\n",ub[0],ub[1],ub[2],ub[3],ub[4],ub[5]);
//        printf("pose v (%lf,%lf,%lf,%lf,%lf,%lf)\n",vb[0],vb[1],vb[2],vb[3],vb[4],vb[5]);
        
        printf("image: (%d). using transform: (%d,%d,%d)\n", one, m1, m2, m3);
        vector<double> up = TransformPose(ub, m1, m2, m3);
        vector<double> vp = TransformPose(vb, m1, m2, m3);
        gtsam::Pose3 u = CameraPose(up);
        gtsam::Pose3 v = CameraPose(vp);
        printf("pose up (%lf,%lf,%lf,%lf,%lf,%lf)\n",up[0],up[1],up[2],up[3],up[4],up[5]);
        printf("pose vp (%lf,%lf,%lf,%lf,%lf,%lf)\n",vp[0],vp[1],vp[2],vp[3],vp[4],vp[5]);
        
        cv::Mat img = ImageOperations::Load(imagepath);
        int ci = 0;
        for(int i=0; i<PFT0.ids.size(); i++){
            while(PFT1.ids[ci]<PFT0.ids[i]) ci++;
            if(PFT1.ids[ci] != PFT0.ids[i]) continue;
            
            gtsam::Point3 pw = ProjectImageToWorld(PFT1.imagecoord[ci], v, PFT0.imagecoord[i], u, gtmat);
            cv::Point2f p = cv::Point2f(PFT0.imagecoord[i].x(), PFT0.imagecoord[i].y());
            double dist = u.range(pw);
            circle(img, p, 5, ColorByHeight(pw.z()), -1, 8, 0); //green for >0 red for <0
            //circle(img, p, 3, ColorByDistance(dist), -1, 8, 0); //white for nearby, black for far away
            circle(img, p, 3, GetLandmarkColor(PFT1.ids[ci]), -1,8,0);
            //printf("(%lf,%lf) ", dist, pw.z());
            //printf("%lf,%lf,%lf; dist: %lf\n",pws[i].x(), pws[i].y(), pws[i].z(), dist);
        }
        
        imshow(window_name, img);
        char c = cvWaitKey();
        switch(c){
            case 'q':
                cvDestroyAllWindows();
                exit(0);
                break;
            case '1':
                m1++;
                break;
            case '2':
                m2++;
                break;
            case '3':
                m3++;
                break;
            case '4':
                m1--;
                break;
            case '5':
                m2--;
                break;
            case '6':
                m3--;
                break;
            case 'f':
                one+=5;
                two+=5;
                updateset=true;
                break;
            case 'b':
                one-=5;
                two-=5;
                updateset=true;
                break;
        }
    }
    
    cvDestroyAllWindows();
}

gtsam::Pose3 TestBikeSurvey::VectorToPose(std::vector<double>& p){
    return gtsam::Pose3(gtsam::Rot3::ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

void TestBikeSurvey::TestVO(){
    string bdbase = "/mnt/tale/shaneg/bike_datasets/";
    string name = "20160831_171816";
    ParseBikeRoute pbr(bdbase, name);
    Camera nexus = ParseBikeRoute::GetCamera();
    gtsam::Cal3_S2::shared_ptr gtcam = nexus.GetGTSAMCam();
    gtsam::Matrix gtmat = gtcam->matrix();
    
    VisualOdometry vo(nexus);
    int one = 1275;
    int two = 1277;
    std::string imagepath = ParseSurvey::GetImagePath(bdbase + name, one);
    ParseFeatureTrackFile PFT0(nexus, bdbase + name, one);
    ParseFeatureTrackFile PFT1(nexus, bdbase + name, two);
    
    vector<double> start = pbr.GetPose(one);
    start[0] = 0;
    start[1] = 0;
    start[2] = 0;
    gtsam::Pose3 u = VectorToPose(start);
    
    const char* window_name = "test poses using point triangulation";
    cvNamedWindow(window_name);
    
    bool updateset=false;
    int m1=0;
    int m2=0;
    int m3=0;
    gtsam::Pose3 v;
    while(1) {
        if(updateset){
            u = v;
            PFT0.Next(one);
            PFT1.Next(two);
            imagepath = ParseSurvey::GetImagePath(bdbase + name, one);
            updateset = false;
        }
        
        vector<double> ub = PoseToVector(u);
        gtsam::Pose3 vop = vo.PoseFromEssential(PFT0, PFT1);
        vector<double> up = TransformPose(ub, m1, m2, m3);
        gtsam::Pose3 cur = VectorToPose(up);
        v = vop.compose(cur);
        vector<double> vb = PoseToVector(v);
        printf("pose u (%lf,%lf,%lf,%lf,%lf,%lf)\n",ub[0],ub[1],ub[2],ub[3],ub[4],ub[5]);
        printf("pose v (%lf,%lf,%lf,%lf,%lf,%lf)\n",vb[0],vb[1],vb[2],vb[3],vb[4],vb[5]);
        
        cv::Mat img = ImageOperations::Load(imagepath);
        int ci = 0;
        for(int i=0; i<PFT0.ids.size(); i++){
            while(PFT1.ids[ci]<PFT0.ids[i]) ci++;
            if(PFT1.ids[ci] != PFT0.ids[i]) continue;
            
            gtsam::Point3 pw = ProjectImageToWorld(PFT1.imagecoord[ci], v, PFT0.imagecoord[i], cur, gtmat);
            cv::Point2f p = cv::Point2f(PFT0.imagecoord[i].x(), PFT0.imagecoord[i].y());
            double dist = cur.range(pw);
            circle(img, p, 5, ColorByHeight(pw.z()), -1, 8, 0); //green for >0 red for <0
            //circle(img, p, 3, ColorByDistance(dist), -1, 8, 0); //white for nearby, black for far away
            circle(img, p, 3, GetLandmarkColor(PFT1.ids[ci]), -1,8,0);
        }
        
        imshow(window_name, img);
        char c = cvWaitKey();
        switch(c){
            case 'q':
                cvDestroyAllWindows();
                exit(0);
                break;
            case '1':
                m1++;
                break;
            case '2':
                m2++;
                break;
            case '3':
                m3++;
                break;
            case '4':
                m1--;
                break;
            case '5':
                m2--;
                break;
            case '6':
                m3--;
                break;
            case 'f':
                one=two;
                two+=2;
                updateset=true;
                break;
            case 'b':
                one-=2;
                two=one;
                updateset=true;
                break;
        }
    }
    
    cvDestroyAllWindows();
}

std::vector<double> TestBikeSurvey::PoseToVector(gtsam::Pose3& cam){
    return {cam.x(), cam.y(), cam.z(), cam.rotation().roll(), cam.rotation().pitch(), cam.rotation().yaw()};
}

void TestBikeSurvey::TestVisualOdometry() {
    string bdbase = "/mnt/tale/shaneg/bike_datasets/";
    string name = "20160831_171816";
    ParseBikeRoute pbr(bdbase, name);
    Camera nexus = ParseBikeRoute::GetCamera();
    gtsam::Cal3_S2::shared_ptr gtcam = nexus.GetGTSAMCam();
    gtsam::Matrix gtmat = gtcam->matrix();
    
    int one = 1275;
    int two = 1277;
    std::string imagepath;
    ParseFeatureTrackFile PFT0(nexus, bdbase + name, one);
    ParseFeatureTrackFile PFT1(nexus, bdbase + name, two);
    
    VisualOdometry vo(nexus);
    gtsam::Pose3 vop = vo.PoseFromEssential(PFT0, PFT1);
    vector<double> vp = PoseToVector(vop);
    printf("pose from vo (%lf,%lf,%lf,%lf,%lf,%lf)\n",vp[0],vp[1],vp[2],vp[3],vp[4],vp[5]);
    vector<double> ub = pbr.GetPose(one);
    vector<double> vb = pbr.GetPose(two);
    printf("pose u (%lf,%lf,%lf,%lf,%lf,%lf)\n",ub[0],ub[1],ub[2],ub[3],ub[4],ub[5]);
    printf("pose v (%lf,%lf,%lf,%lf,%lf,%lf)\n",vb[0],vb[1],vb[2],vb[3],vb[4],vb[5]);
    
    gtsam::Pose3 u = pbr.CameraPose(one);
    gtsam::Pose3 vpu = u.compose(vop);
    vector<double> vvpu = PoseToVector(vpu);
    printf("pose u+odom (%lf,%lf,%lf,%lf,%lf,%lf)\n",vvpu[0],vvpu[1],vvpu[2],vvpu[3],vvpu[4],vvpu[5]);
}



















































































