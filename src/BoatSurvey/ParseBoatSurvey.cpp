//
//  ParseBoatSurvey.cpp
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "ParseBoatSurvey.hpp"
#include <Visualizations/SLAMDraw.h>

using namespace std;

std::vector<double> ParseBoatSurvey::GetRotationMatrix(double X, double Y, double Z) {
    /*Converts the three orientation angles into a rotation matrix.
     see http://www.songho.ca/opengl/gl_anglestoaxes.html
     */
    
    std::vector<double> R = {cos(Y)*cos(Z), -cos(Y)*sin(Z), sin(Y),
        sin(X)*sin(Y)*cos(Z) + cos(X)*sin(Z), -sin(X)*sin(Y)*sin(Z)+cos(X)*cos(Z), -sin(X)*cos(Y),
        -cos(X)*sin(Y)*cos(Z)+sin(X)*sin(Z), cos(X)*sin(Y)*sin(Z)+sin(X)*cos(Z), cos(X)*cos(Y)};
    
    return R;
}

std::vector<double> ParseBoatSurvey::ComposeRotationMatrices(std::vector<double> A, std::vector<double> B){
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

std::vector<double> ParseBoatSurvey::RotationMatrixToRPY(std::vector<double> R){
    std::vector<double> rpy = {atan2(R[7], R[8]), atan2(-1*R[6], sqrt(R[7]*R[7]+R[8]*R[8])), atan2(R[3],R[0])};
    return rpy;
}

vector<double> ParseBoatSurvey::GetCameraPose(double x, double y, double theta, double camera_pan, double tilt) {
    std::vector<double> align_with_world = GetRotationMatrix(0, M_PI_2, -M_PI_2);
    std::vector<double> cam = GetRotationMatrix(tilt, 0, -camera_pan);
    std::vector<double> boat = GetRotationMatrix(0, 0, theta);
    
    std::vector<double> R = ComposeRotationMatrices(ComposeRotationMatrices(boat, cam), align_with_world);
    std::vector<double> RPY = RotationMatrixToRPY(R);
    std::vector<double> ret = {x-default_start.x(), y-default_start.y(), 0, RPY[0], RPY[1], RPY[2]};
    return ret;
}

void ParseBoatSurvey::ProcessLineEntries(int type, vector<string>& lp){
    if(lp[0].at(0)=='%')return;
    if(lp.size()<14)return;
    timings.push_back(stod(lp[0]));
    vector<double> p = GetCameraPose(stod(lp[2]), stod(lp[3]), stod(lp[4]), stod(lp[5]), stod(lp[6]));
    poses.push_back(p);
    imageno.push_back((int)stod(lp[1]));
    cam_pan.push_back(stod(lp[5]));
    omega.push_back(stod(lp[13]));
}

void ParseBoatSurvey::ReadDelimitedFile(string file, int type) {
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

int ParseBoatSurvey::GetImageNumber(int auxidx){
    return imageno[auxidx];
}

int ParseBoatSurvey::GetIndexOfImage(int image){
    /*Finds the index in the aux file for a given image. Direct mapping doesn't work if
     * the aux is cropped anywhere after the beginning or images are skipped. Iterative
     * direct mapping is used.
     */
    if(imageno.size()==0) return -1;
    
    int last_dist = image;
    int idx=0, last_idx=0;
    while(imageno[idx] != image){
        int dist = abs(imageno[idx]-image);
        if(last_dist < dist){
            cout << "ParseBoatSurvey: Warning. Couldn't find the exact image in the aux file." << endl;
            return last_idx;
        }
        last_dist = dist;
        last_idx = idx;
        idx += image - imageno[idx];
    }
    return idx;
}

double ParseBoatSurvey::GetAvgAngularVelocity(int sidx, int eidx) {
    /* The average angular velocity between two indices [sidx, eidx].
     * */
    sidx = max(0,sidx);
    eidx = min((int)omega.size()-1, eidx);
    double sum = 0.0;
    for(int i=sidx; i < eidx; ++i)
    {
        sum += omega[i];
    }
    if(eidx-sidx == 0) return omega[eidx];
    return sum/(eidx-sidx);
}

double
ParseBoatSurvey::
changeInYaw(double t_m1, double t)
{
    int idx_m1 = timestampToIndex(t_m1);
    int idx = timestampToIndex(t);
    double ang=0;
    for(int t=idx_m1+1; t<= idx; ++t)
    {
        double t_step = timings[t] - timings[t-1];
        double& ang_vel = omega[t-1];
        ang += ang_vel * IMU_GYRO_GAIN * t_step; 
    }
    return ang;
}

bool ParseBoatSurvey::Useable(int cidx, int lcidx){
    bool good_cam_pan = (abs(cam_pan[cidx])==1.569978);
    bool good_gps = true;
    if(cidx >= 0 && lcidx >= 0 && cidx<poses.size() && lcidx<poses.size())
        good_gps = !((poses[cidx][0] == poses[lcidx][0]) && (poses[cidx][1] == poses[lcidx][1]));
    if(!good_gps) std::cout << "bad gps detected at (image indices) " << imageno[lcidx] << ", " << imageno[cidx] << std::endl;
    return good_cam_pan && good_gps;
}

Camera ParseBoatSurvey::GetCamera(){
    Camera axisP5512e(759.308012, 690.43984, 370.91545, 250.909693, 704, 480);
    axisP5512e.SetDistortion(-0.302805, 0.171088, 0.001151, -0.00038, 0.0);
    return axisP5512e;
}

vector<double> ParseBoatSurvey::GetDrawScale(){
    return {-300,300,-300,300};
}

void ParseBoatSurvey::PlayPoses(){
    //unit test the poses.
    
    vector<double> scale = GetDrawScale();
    
    SLAMDraw art;
    art.SetScale(scale[0],scale[1],scale[2],scale[3]);
    art.ResetCanvas();
    int skip = 50;
    for(int i=0; i<poses.size(); i=i+skip){
        //std::cout << i<<":"<<poses[i][0] << ", " << poses[i][1] << ", " << poses[i][5] << std::endl;
        printf("%lf,%lf,%lf,%lf,%lf\n",poses[i][0],poses[i][1],poses[i][3]*180/M_PI,poses[i][4]*180/M_PI,poses[i][5]*180/M_PI);
        for(int j=0; j<=i; j++)
            art.AddShape(SLAMDraw::shape::CIRCLE, poses[j][0], poses[j][1], 0, 0, 0);
        art.DrawSight(poses[i][0], poses[i][1], poses[i][3], 0.838, 255, 0, 0);
        art.DrawSight(poses[i][0], poses[i][1], poses[i][4], 0.838, 0, 255, 0);
        art.DrawSight(poses[i][0], poses[i][1], poses[i][5], 0.838, 0, 0, 255);
        char c = art.Display();
        if(c==83) i = (i>=2*skip)?i-2*skip:i-skip;
        art.ResetCanvas();
    }
}






