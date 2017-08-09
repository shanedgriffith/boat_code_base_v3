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

const string Anchors::_anchorsname = "/anchors.txt";

void Anchors::WriteAnchors(){
    string fname = _base + _date + _anchorsname;
    FILE * fp = fopen(fname.c_str(), "w");
    if(!fp) {
        std::cout << "Anchors::WriteAnchors(). Couldn't save em." << fname << std::endl;
        exit(1);
    }

    int lastl=0;
    for(int i=0; i<anchor.size(); i++){
        for(int j=0; j<anchor[i].size(); j++)
            fprintf(fp, "%lf, ", anchor[i][j]);
        fprintf(fp, "%d\n", sections[i]);
    }
    fflush(fp);
    fclose(fp);
}

bool Anchors::ReadAnchors(){
    string fname = _base + _date + _anchorsname;
    FILE * fp = fopen(fname.c_str(), "r");
    if(!fp) return false;
    char line[LINESIZE]="";
    int count = 0;
    while (fgets(line, LINESIZE-1, fp)) {
        std::vector<std::string> vals = ParseLine(line);
        vector<double> anc(6,0);
        int section = 0;

        if(vals.size()<7) {
            std::cout << "something went wrong with the anchors file. Check it: " << fname << std::endl;
            std::cout << "Line: " << line << std::endl;
            exit(1);
        }
        for(int i=0; i<anc.size(); i++) anc[i] = stod(vals[i]);
        section = stoi(vals[6]);

        anchor.push_back(anc);
        sections.push_back(section);
        count++;
    }
    return (count>0)?true:false;
}

bool Anchors::IsValid(int section){
    //an anchor is nonvalid if it's all zeros (i.e., uninitialized)
    if(section > sections.size()) return false;
    for(int i=0; i<anchor[section].size(); i++){
        if(abs(anchor[section][i]) > 0.000001) return true;
    }
    return false;
}

int Anchors::NumAnchors(){
    return sections.size();
}

bool Anchors::HaveAnchors(){
    return sections.size() > 0;
}

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
//    WriteAnchors();
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

void Anchors::Print() {
    std::cout << "Anchor " << _date << std::endl;
    for(int i=0; i<anchor.size(); i++){
        std::cout << sections[i] << ": ";
        for(int j=0; j<anchor[i].size(); j++){
            std::cout << anchor[i][j] << ", ";
        }
        std::cout << std::endl;
    }
}

