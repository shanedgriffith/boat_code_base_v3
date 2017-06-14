
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
    int val = 255-min(255./10*dist,255.);
    return CV_RGB(val, val, val);
}

cv::Scalar TestBikeSurvey::ColorByHeight(double z){
    int val = min(abs(255./10*z),255.);
    if(z<0) return CV_RGB(val, 0, 0);
    return CV_RGB(0, val, 0);
}

std::vector<double> TestBikeSurvey::GetRotationMatrix(double X, double Y, double Z) {
    /*Converts the three orientation angles into a rotation matrix.
     see http://www.songho.ca/opengl/gl_anglestoaxes.html
     */
    
    std::vector<double> R = {cos(Y)*cos(Z), -cos(Y)*sin(Z), sin(Y),
        sin(X)*sin(Y)*cos(Z) + cos(X)*sin(Z), -sin(X)*sin(Y)*sin(Z)+cos(X)*cos(Z), -sin(X)*cos(Y),
        -cos(X)*sin(Y)*cos(Z)+sin(X)*sin(Z), cos(X)*sin(Y)*sin(Z)+sin(X)*cos(Z), cos(X)*cos(Y)};
    
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
    vector<double> cam = GetRotationMatrix(p[3], p[4], p[5]);
    vector<double> align_with_world = GetRotationMatrix(m1*M_PI_2, m2*M_PI_2, m3*M_PI_2);
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
    
    int one = 500;
    int two = 501;
    
    vector<double> ub = pbr.GetPose(one);
    vector<double> vb = pbr.GetPose(two);
    printf("pose u (%lf,%lf,%lf,%lf,%lf,%lf)\n",ub[0],ub[1],ub[2],ub[3],ub[4],ub[5]);
    printf("pose v (%lf,%lf,%lf,%lf,%lf,%lf)\n",vb[0],vb[1],vb[2],vb[3],vb[4],vb[5]);
    
    vector<gtsam::Point3> pws;
    vector<cv::Point2f> ps;
    ParseFeatureTrackFile PFT0 = pbr.LoadVisualFeatureTracks(nexus, one);
    ParseFeatureTrackFile PFT1 = pbr.LoadVisualFeatureTracks(nexus, two);
    
    std::string imagepath = ParseSurvey::GetImagePath(bdbase + name, one);
    
    const char* window_name = "test poses using point triangulation";
    cvNamedWindow(window_name);
    
    int m1=0;
    int m2=0;
    int m3=0;
    while(1){
        vector<double> up = TransformPose(up, m1, m2, m3);
        vector<double> vp = TransformPose(vp, m1, m2, m3);
        gtsam::Pose3 u = CameraPose(up);
        gtsam::Pose3 v = CameraPose(vp);
        printf("pose up (%lf,%lf,%lf,%lf,%lf,%lf)\n",up[0],up[1],up[2],up[3],up[4],up[5]);
        printf("pose vp (%lf,%lf,%lf,%lf,%lf,%lf)\n",vp[0],vp[1],vp[2],vp[3],vp[4],vp[5]);
        
        cv::Mat img = ImageOperations::Load(imagepath);
        int ci = 0;
        for(int i=0; i<PFT0.ids.size(); i++){
            while(PFT1.ids[ci]<PFT0.ids[i]) ci++;
            if(PFT1.ids[ci] != PFT0.ids[i]) continue;
            
            gtsam::Point3 pw = ProjectImageToWorld(PFT0.imagecoord[i], u, PFT1.imagecoord[ci], v, gtmat);
            cv::Point2f p = cv::Point2f(PFT0.imagecoord[i].x(), PFT0.imagecoord[i].y());
            double dist = u.range(pw);
            circle(img, p, 5, ColorByHeight(pw.z()), -1, 8, 0);
            circle(img, p, 3, ColorByDistance(dist), -1, 8, 0);
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
        }
        printf("using transform: (%d,%d,%d)\n", m1, m2, m3);
    }
    
    cvDestroyAllWindows();
}





















































































