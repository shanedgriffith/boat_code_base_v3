//
//  ParseSurvey.cpp
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "ParseSurvey.h"
#include "ParseFeatureTrackFile.h"


using namespace std;

const string ParseSurvey::_auxfile = "/image_auxilliary.csv";

int ParseSurvey::GetImageNumberFromImagePath(string imagepath) {
    int l = imagepath.rfind(".");
    int s = imagepath.rfind("/");
    int f = imagepath.rfind("/", s-1);
    string imgno =imagepath.substr(s+1,l-s-1);
    string mil = imagepath.substr(f+1, s-f-1);
    return stod(mil)*1000+stod(imgno);
}

vector<double> ParseSurvey::PoseToVector(gtsam::Pose3& cam) {
    return {cam.x(), cam.y(), cam.z(), cam.rotation().roll(), cam.rotation().pitch(), cam.rotation().yaw()};
}

gtsam::Pose3 ParseSurvey::CameraPose(int idx){
        return gtsam::Pose3(gtsam::Rot3::ypr(poses[idx][5], poses[idx][4], poses[idx][3]), gtsam::Point3(poses[idx][0], poses[idx][1], poses[idx][2]));
}

int ParseSurvey::FindSynchronizedAUXIndex(double querytime, int from_idx){
    /*Find the index in the AUX file that is time-aligned with the visual feature track data.*/
    double diff = timings[from_idx] - querytime;
    double epsilon = 0.02;
    //sanity check
    if(querytime<=0) return -1;
    if(abs(diff)>5000){
        cout << "ParseSurvey::FindSynchronizedAUXIndex() Error: Incorrect set of feature tracking files." << endl;
        exit(-1);
    }
    
    if(debug) cout << "looking for time " << querytime << ", from " << timings[from_idx] << endl;
    
    if(diff > 0) return from_idx;
    while(diff>0 && abs(diff) > epsilon && from_idx > 0){
        diff = timings[--from_idx] - querytime;
    }
    while(diff<0 && abs(diff) > epsilon && from_idx < timings.size()){
        diff = timings[++from_idx] - querytime;
    }
    
    if(debug && from_idx < timings.size()) cout << "found at: " << from_idx << ", with " << timings[from_idx] << endl;
    if(from_idx >= timings.size()) return -1;//not sure why this was added: || timings[from_idx] < querytime) return -1;
    
    return from_idx;
}

double ParseSurvey::AngleDistance(double a, double b){
    //[-pi, pi]
    double d = a-b;
    d -= (d > M_PI)?2*M_PI:0;
    d += (d < M_PI)?2*M_PI:0;
    return abs(fmod(d+M_PI, 2*M_PI)-M_PI);
}

bool ParseSurvey::CheckCameraTransition(int cidx, int lcidx){
    /*Large changes in camera pose violate our kinematic constraint of smooth motion. This function detects that.*/
    gtsam::Pose3 cam = CameraPose(cidx);
    gtsam::Pose3 last_cam = CameraPose(lcidx);
    
    double ly = last_cam.rotation().yaw();
    double cy = cam.rotation().yaw();
    
    double trans_diff = pow(cam.x() - last_cam.x(), 2) +
    pow(cam.y() - last_cam.y(), 2) +
    pow(cam.z() - last_cam.z(), 2);
    
    if(AngleDistance(ly,cy) > M_PI_2)
        return true;
    else if (trans_diff > 1.0)
        return true;
    
    return false;
}

ParseFeatureTrackFile ParseSurvey::LoadVisualFeatureTracks(Camera& _cam, int& index){
    /*Proceed when the visual feature track file is good.*/
    static bool found = false;
    if(!found){
        std::vector<string> dirs = ListDirsInDir(_pftbase + _date + "/sift");
        if(dirs.size() == 0){std::cout << "No pft tracking files" << std::endl; exit(-1);}
        std::vector<string> files = ListFilesInDir(_pftbase + _date + "/sift/" + dirs[0], ".csv");
        if(files.size() == 0){std::cout << "No pft tracking files" << std::endl; exit(-1);}
        int start = stoi(dirs[0])*1000 + stoi(files[0].substr(0,files[0].length()-4));
        if(start > index) index = start;
        std::cout << "First PFT file at " << index << std::endl;
    }
    
    ParseFeatureTrackFile PFT(_cam, _pftbase + _date, index);
    int nonexist=0;
    while(PFT.time<=0) {
        if(!PFT.Exists(PFT.siftfile)) {
            nonexist++;
            if(nonexist>10 && found || nonexist>50000) {
                if(found) return PFT;
                cout << "ParseSurvey::LoadVisualFeatureTracks() Error. There are no feature tracking files. Check directory: " << PFT.siftfile << endl;
                exit(-1);
            }
        }
        //if(debug)
            cout << "Empty PFT file at " << index  << ". loading next." << endl;
        PFT.Next(++index);
    }
    cout.precision(5);
    if(debug) cout << "Loaded PFT file at: " << index  << ", has time: " << std::fixed<<PFT.time<< endl;
    found = true;
    return PFT;
}

std::vector<double> ParseSurvey::GetPose(int i){
    if(i>poses.size() || i < 0){
        std::cout << "ParseSurvey::GetPose() Can't get pose "<< i << ", size: " << poses.size()<<std::endl;
        exit(-1);
    }
    return poses[i];
}

string ParseSurvey::GetImagePath(string base, int no, bool makedir) {
    char imgfile[100];
    if(makedir){
        //although an 'images' directory makes sense, adding it would require updating the VBags structure (which probably should be upgraded..)
        sprintf(imgfile, "%s", base.c_str());
        MakeDir(imgfile);
        sprintf(imgfile, "%s/%04d", base.c_str(), (((int) no)/1000));
        MakeDir(imgfile);
    }
    sprintf(imgfile, "%s/%04d/%04d.jpg", base.c_str(), (((int) no)/1000), (((int) no)%1000));
    return string(imgfile);
}



















