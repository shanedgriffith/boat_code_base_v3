/*
 * Anchors.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: shane
 */

#include "Anchors.hpp"

#include <gtsam/geometry/Pose3.h>
#include <Optimization/EvaluateSLAM.h>
#include <DataTypes/Camera.hpp>
#include <cmath>

using namespace std;

const string Anchors::_anchorsname = "/anchors.txt";

Anchors::Anchors(Camera& _cam, string base, string date):
    _filename(base + date + Anchors::_anchorsname) {
        LoadAnchors();
}


Anchors(Camera& _cam, ParseOptimizationResults& POR, int nanchors, int nposes):
_filename(POR._base + Anchors::_anchorsname) {
    last = POR.boat.size();
    anchors = vector<vector<double>>(nanchors, vector<double>(6,0));
    sections = vector<int>(nanchors, 0);
    int binsz = round(nposes/nanchors);
    for(int i=0; i<nanchors; i++){
        sections[i] = i*binsz;
    }
}


void Anchors::WriteAnchors(){
    FILE * fp = OpenFile(_filename, "w", true);
    for(int i=0; i<anchors.size(); i++) {
        fprintf(fp, "%d, ", sections[i]);
        for(int j=0; j<anchors[i].size(); j++) {
            if(j<anchors[i].size()-1) fprintf(fp, "%lf, ", anchor[i][j]);
            else fprintf(fp, "%lf", anchor[i][j]);
        }
    }
    fflush(fp);
    fclose(fp);
}


void Anchors::LoadAnchors(){
    FILE * fp = OpenFile(_filename.c_str(), "r", true);
    char line[LINESIZE]="";
    while (fgets(line, LINESIZE-1, fp)) {
        vector<double> a(6, 0.0);
        int s;
        int ret = sscanf(line, "%d, %lf, %lf, %lf, %lf, %lf, %lf\n",
                         &s, &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
        if(ret != 7) {
            std::cout << "Anchors::LoadAnchors() read error: " << fname << ", line: " << line << std::endl;
            exit(1);
        }

        anchors.push_back(a);
        sections.push_back(s);
    }
}


int Anchors::NumAnchors(){
    return sections.size();
}


vector<double> Anchors::ShiftPose(int s, gtsam::Pose3& gtp){
    vector<double>& a = anchors[s];
    gtsam::Pose3 gta(gtsam::Rot3::RzRyRx(a[3],a[4],a[5]), gtsam::Point3(a[0],a[1],a[2]));
    return gta.compose(gtp); //order?
    return {comp.x(), comp.y(), comp.z(), comp.rotation().roll(), comp.rotation().pitch(), comp.rotation().yaw()};
}


vector<double> Anchors::ShiftPose(int s, vector<double>& p){
    vector<double>& a = anchors[s];
    gtsam::Pose3 gta(gtsam::Rot3::RzRyRx(a[3],a[4],a[5]), gtsam::Point3(a[0],a[1],a[2]));
    gtsam::Pose3 gtp(gtsam::Rot3::RzRyRx(p[3],p[4],p[5]), gtsam::Point3(p[0],p[1],p[2]));
    gtsam::Pose3 comp = gta.compose(gtp); //order?
    return {comp.x(), comp.y(), comp.z(), comp.rotation().roll(), comp.rotation().pitch(), comp.rotation().yaw()};
}


vector<vector<double>> Anchors::GetShiftedPoses(vector<vector<double>>& poses) {
    vector<vector<double>> shifted(poses.size(), vector<double>(6, 0));
    int s = 0;
    for(int i=0; i<poses.size(); i++){
        while(s < sections.size()-1 && sections[s+1] <= i) s++;
        shifted[i] = ShiftPose(s, poses[i]);
    }
    return shifted;
}


//what if the updated set has a different size? skip for now.
void Anchors::UpdateAnchors(vector<vector<double>>& updated) {
    for(int i=0; i<anchors.size(); i++){
        anchors[i] = updated[i];
    }
}


int Anchors::PoseIdxToAnchorIdx(int pidx){
    int top = sections.size();
    int bot = 0;
    while(top>bot){
        int med = bot + (top-bot)/2;
        if(sections[med] < pidx) bot = med;
        if(sections[med] > pidx) top = med;
        else return med;
    }
    return -1;
}


void Anchors::Print() {
    for(int i=0; i<sections.size(); i++){
        std::cout << sections[i] << ": ";
        for(int j=0; j<anchors[i].size(); j++){
            std::cout << anchors[i][j] << ", ";
        }
        std::cout << std::endl;
    }
}


gtsam::Pose3 Anchors::GetAnchorAsPose(int idx){
    vector<double>& anc = anchors[idx];
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(anc[3],anc[4],anc[5]), gtsam::Point3(anc[0],anc[1],anc[2]));
}

/*
 
 get anchor for a given pose.
 
 update
   number of sections (with a flag for whether to change this or not)
   anchors positions
 
 
 */





/*
void Anchors::FindSections(Camera& _cam){
    //a section is a discontinuity in the optimized trajectory (loss of visual feature tracks, high rerror, ).
    //can I find all the discontinuities in the rerror information? Aren't they zero if the visual feature tracks are lost?
    if(sections.size() > 0) {
        std::cout << "Anchors::FindSections(): Already got sections for " << _date << std::endl;
        exit(1);
    }

    vector<double> rerrs = EvaluateSLAM::LoadRerrorFile(_base, _date);
    ParseOptimizationResults por(_base + _date);
    if(rerrs.size() != por.boat.size()) {
        std::cout << "Anchors::FindSections() Size issue." << std::endl;
        std::cout << "POR File in dir " << _base << _date << std::endl;
        exit(-1);
    }
    sections.push_back(0);
    for(int i=0; i<por.boat.size(); i++) {
        ParseFeatureTrackFile pftf = LoadFTF(_cam, por, i);
        ModifyFTF(pftf, por);
        int persistent_set_size = ProcessNewPoints(i, pftf);
        if(persistent_set_size < 8) {
            if(sections[sections.size()-1] < i-1) {
                sections.push_back(i);
                std::cout << "Anchor["<<sections.size()-1<<"] at section "<< i << " due to the loss of visual feature tracking. number of persistent visual features: " << persistent_set_size << std::endl;
            }
        } else if(rerrs[i] == 0 || rerrs[i] > avgbadthreshold) {
            if(sections[sections.size()-1]<i-1){
                sections.push_back(i);
                std::cout << "Anchor["<<sections.size()-1<<"] at section "<< i <<" due to the high (or zero) reprojection error of " << rerrs[i] << std::endl;
            }
            std::cout << "reprojection error " << rerrs[i] << std::endl;
        }
    }

    int last = por.boat.size()-1;
    for(int i=sections.size()-1; i>=0; i--){
        int diff = last - sections[i];
        last = sections[i];
        if(diff < 5) sections.erase(sections.begin()+i, sections.begin()+i+1);
    }

    anchor = std::vector<std::vector<double>>(sections.size(), std::vector<double>(6, 0.0));
    std::cout << "Found " << sections.size() << " sections"<< std::endl;
}

void Anchors::SetAnchor(int section, std::vector<double> anc){
    if(section > sections.size()) {
        std::cout << "Anchors::SetAnchor(): oob. " << std::endl;
        exit(1);
    }
    anchor[section] = anc;
}

int Anchors::GetSection(int idx){
    for(int i=1; i<sections.size(); i++){
        if(sections[i] > idx) return i-1;
    }
    return sections.size()-1;
}

std::vector<double> Anchors::GetAnchor(int idx){
    int anum = GetSection(idx);
    if(anum<0) return {};
    return anchor[anum];
}

gtsam::Pose3 Anchors::GetAnchorAsPose(int idx){
    vector<double> anc = GetAnchor(idx);
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(anc[3],anc[4],anc[5]), gtsam::Point3(anc[0],anc[1],anc[2]));
}




vector<double> Anchors::GetAnchoredPose(int i, vector<double> p){
    vector<double> anc = GetAnchor(i);
    if(anc.size()==0 || p.size()<6) return {};
    gtsam::Pose3 gta(gtsam::Rot3::RzRyRx(anc[3],anc[4],anc[5]), gtsam::Point3(anc[0],anc[1],anc[2]));
    gtsam::Pose3 gtp(gtsam::Rot3::RzRyRx(p[3],p[4],p[5]), gtsam::Point3(p[0],p[1],p[2]));
    gtsam::Pose3 comp = gta.compose(gtp);
    return {comp.x(), comp.y(), comp.z(), comp.rotation().roll(), comp.rotation().pitch(), comp.rotation().yaw()};
}

*/
