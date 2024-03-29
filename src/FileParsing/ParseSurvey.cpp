//
//  ParseSurvey.cpp
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "ParseSurvey.h"
#include "ParseFeatureTrackFile.h"
#include "Optimization/SingleSession/GTSAMInterface.h"

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

gtsam::Pose3 ParseSurvey::CameraPose(int idx){
    return GTSAMInterface::VectorToPose(poses[idx]);
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

int
ParseSurvey::
timestampToIndex(double timestamp)
{
    int s=0;
    int e=timings.size();
    while(e-s>=1)
    {
        int m = s + (e-s)/2;
        if(timings[m] < timestamp) s = m;
        else if(timings[m] > timestamp) e = m;
        else return m;
    }
    throw std::runtime_error("ParseSurvey::timestampToIndex() error. timestamp not found.");
    return -1;
}

double ParseSurvey::AngleDistance(double a, double b){
    //[-pi, pi]
    double d = a-b;
    d -= (d > M_PI)?2*M_PI:0;
    d += (d < M_PI)?2*M_PI:0;
    return abs(fmod(d+M_PI, 2*M_PI)-M_PI);
}

bool ParseSurvey::CheckGap(int last_auxidx, int next_auxidx){
    if(timings.size() <= next_auxidx) {
        return false;
    }
    int im1 = GetImageNumber(last_auxidx);
    int im2 = GetImageNumber(next_auxidx);

    if(im2-im1 > 15) return true;
    return false;
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

int ParseSurvey::IdentifyFirstPFTFileAtOrAfter(int index) {
    std::vector<string> dirs = ListDirsInDir(_pftbase + _date + "/sift");
    if(dirs.size() == 0){std::cout << "No pft tracking files" << std::endl; exit(-1);}
    int dir = index / 1000;
    if(dir >= dirs.size()){std::cout << "No pft tracking files at or after " << index << std::endl; exit(-1);}
    std::vector<string> files = ListFilesInDir(_pftbase + _date + "/sift/" + dirs[dir], ".csv");
    if(files.size() == 0){std::cout << "No pft tracking files" << std::endl; exit(-1);}
    int file = index % 1000;
    for(int i=0; i<files.size(); i++) {
        if(file <= stoi(files[i].substr(0,files[i].length()-4)))
            return stoi(dirs[dir])*1000 + stoi(files[i].substr(0,files[i].length()-4));
    }
    return -1;
}

int ParseSurvey::IdentifyLastPFTFile() {
    std::vector<string> dirs = ListDirsInDir(_pftbase + _date + "/sift");
    if(dirs.size() == 0){std::cout << "No pft tracking files" << std::endl; exit(-1);}
    int curdir = dirs.size()-1;
    while(curdir > 0) {
        std::vector<string> files = ListFilesInDir(_pftbase + _date + "/sift/" + dirs[curdir], ".csv");
        if(files.size() == 0) curdir--;
        return stoi(dirs[curdir])*1000 + stoi(files[files.size()-1].substr(0,files[files.size()-1].length()-4));
    }
    std::cout << "No pft tracking files" << std::endl;
    exit(-1);
    return -1;
}

ParseFeatureTrackFile ParseSurvey::LoadVisualFeatureTracks(const Camera& _cam, int& index, bool gap){
    /*Proceed when the visual feature track file is good.*/
    static bool found = false;
    static int end = -1;
    if(!found){
        int start = IdentifyFirstPFTFileAtOrAfter(index);
        if(start > index) index = start;
        std::cout << "First PFT file at " << index << std::endl;
        end = IdentifyLastPFTFile();
        std::cout << "Last PFT file at " << end << std::endl;
    }
    
    ParseFeatureTrackFile PFT(_cam, _pftbase + _date, index);
    int nonexist = 0;
    while(PFT.time <= 0) {
        if(!gap && !PFT.Exists(PFT.siftfile)) {
            nonexist++;
            if((nonexist>10 && found) || nonexist>50000) {
                if(found) return PFT;
                cout << "ParseSurvey::LoadVisualFeatureTracks() Error. There are no feature tracking files. Check directory: " << PFT.siftfile << endl;
                exit(-1);
            }
        }
        if(!gap)
            cout << "Empty PFT file at " << index  << ". loading next." << endl;
        if(index > end) {
//            std::cout << "ParseSurvey::LoadVisualFeatureTracks() Error. Went beyond the last PFT file." << std::endl;
            return PFT;
        }
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



















