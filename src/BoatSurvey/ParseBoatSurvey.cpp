//
//  ParseSurvey.cpp
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "ParseBoatSurvey.hpp"


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
    char line[LINESIZE]="";
    
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
    for(int i=sidx; i<=eidx; i++) {
        sum += omega[i];
    }
    if(eidx-sidx==0) return omega[eidx];
    return sum/(eidx-sidx);
}

bool ParseBoatSurvey::Useable(int idx){
    return (abs(cam_pan[idx])==1.569978);
}

Camera ParseBoatSurvey::GetCamera(){
    Camera axisP5512e(759.308012, 690.43984, 370.91545, 250.909693, 704, 480);
    axisP5512e.SetDistortion(-0.302805, 0.171088, 0.001151, -0.00038, 0.0);
    return axisP5512e;
}

