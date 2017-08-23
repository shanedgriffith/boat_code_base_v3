/*
 * Anchors.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: shane
 */

#include "Anchors.hpp"

#include <gtsam/geometry/Pose3.h>
#include <RFlowOptimization/EvaluateRFlowAnchors.hpp>
#include <cmath>

using namespace std;

const string Anchors::_anchorsname = "/anchors.txt";

Anchors::Anchors(Camera& cam, string base, string date):
_cam(cam), _date(date), _filename(base + date + Anchors::_anchorsname) {
        LoadAnchors();
}


Anchors::Anchors(Camera& cam, std::string date, ParseOptimizationResults& POR, int nanchors, int nposes):
_cam(cam), _date(date), _filename(POR._base + Anchors::_anchorsname) {
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
            if(j<anchors[i].size()-1) fprintf(fp, "%lf, ", anchors[i][j]);
            else fprintf(fp, "%lf", anchors[i][j]);
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
            std::cout << "Anchors::LoadAnchors() read error: " << _filename << ", line: " << line << std::endl;
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
    gtsam::Pose3 comp = gta * gtp;
    return {comp.x(), comp.y(), comp.z(), comp.rotation().roll(), comp.rotation().pitch(), comp.rotation().yaw()};
}


vector<double> Anchors::ShiftPose(int s, vector<double>& p){
    gtsam::Pose3 gtp(gtsam::Rot3::RzRyRx(p[3],p[4],p[5]), gtsam::Point3(p[0],p[1],p[2]));
    return ShiftPose(s, gtp);
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


void Anchors::UpdateAnchors(vector<vector<double>>& updated) {
    for(int i=0; i<updated.size(); i++){
        for(int j=0; j<6; j++)
            anchors[i][j] = updated[i][j];
    }
}


int Anchors::PoseIdxToAnchorIdx(int pidx){
    int top = sections.size();
    int bot = 0;
    while(top>bot){
        int med = bot + (top-bot)/2;
        if(sections[med] < pidx) bot = med;
        else if(sections[med] > pidx) top = med;
        else return med;
    }
    return -1;
}


bool Anchors::IsTransition(int t){
    //transition if p_t and p_tm1 connect to different anchors.
    if(t == 0) return false;
    int a_t = PoseIdxToAnchorIdx(t);
    int a_tm1 = PoseIdxToAnchorIdx(t-1);
    return a_t != a_tm1;
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


std::vector<bool> Anchors::SplitAnchors(const std::vector<std::vector<double> >& landmarks, std::vector<double>& rerrors, ParseOptimizationResults& POR, std::string _pftset){
    vector<bool> split(sections.size(), false);
    double mult = 2;
    EvaluateRFlowAnchors erfintra(_cam);
    for(int i=0; i<anchors.size(); i++) {
        int sidx = sections[i];
        int eidx = 0;
        if(i<sections.size()-1) eidx = sections[i+1];
        else eidx = last;
        if(eidx-sidx <= 1) continue;
        
        double rerror = 0;
        double old = 0;
        for(int j=sidx; j<eidx; j++) {
            rerror = erfintra.ComputeAnchorRError(anchors[i], POR, j, _pftset, landmarks);
            old += rerrors[j];
        }
        rerror = rerror/(eidx-sidx);
        old = old/(eidx-sidx);
        
        //expand the set
        if(rerror > mult*old){
            split[i] = true;
            int news = (eidx-sidx)/2 + sidx;
            vector<double> a = anchors[i];
            anchors.insert(anchors.begin() + i, a);
            sections.insert(sections.begin() + i, news);
            split.insert(split.begin()+i, true);
            i++;
        }
    }
    
    return split;
}


void Anchors::MergeAnchors(ParseOptimizationResults& POR, std::string _pftset, std::vector<bool>& split, const std::vector<std::vector<double> >& landmarks){
    
    EvaluateRFlowAnchors erfintra(_cam);
    
    int iters=0;
    double bound=6;
    int countmerged = 0;
    for(int i=1; i<anchors.size(); i++) {
        iters++;
        std::cout << iters <<" iters " << i << ", " << sections[i] << std::endl;
        if(split[i] || split[i-1]) continue;
        int s = sections[i-1];
        int e = (i==anchors.size()-1)?last:sections[i+1];
        vector<double> mergedrerror(e - s, 0);
        
        //get the weighted average pose.
        double w = (1.0*sections[i]-s)/(e-s);
        vector<double> avgd(6, 0);
        for(int j=0; j<6; j++)
            avgd[j] = w*anchors[i-1][j] + (1-w)*anchors[i][j];
        
        //test merge
        //  endpoints and then full merge
        bool merged = false;
        mergedrerror[sections[i]-s-1] = erfintra.ComputeAnchorRError(avgd, POR, sections[i]-1, _pftset, landmarks);
        mergedrerror[sections[i]-s] = erfintra.ComputeAnchorRError(avgd, POR, sections[i], _pftset, landmarks);
        if(mergedrerror[sections[i]-s-1] < bound && mergedrerror[sections[i]-s] < bound) {
            double tot = 0;
            for(int i=0; i<mergedrerror.size(); i++) {
                if(mergedrerror[i] == 0) mergedrerror[i] = erfintra.ComputeAnchorRError(avgd, POR, s+i, _pftset, landmarks);
                tot += mergedrerror[i];
            }
            if(tot/mergedrerror.size() < bound) merged = true;
        }
        std::cout << "section test: endpoints: "<<sections[i]-1<<", " <<sections[i] <<", error: " << mergedrerror[sections[i]-s-1] << ", " << mergedrerror[sections[i]-s] << ", merged? " << merged << std::endl;
        //shrink the set
        if(merged) {
            anchors.erase(anchors.begin()+i);
            sections.erase(sections.begin()+i);
            std::swap(anchors[i-1], avgd);
            i--;
            countmerged++;
        }
    }
    std::cout << "Merged " << countmerged << std::endl;
}


void Anchors::ModifyAnchors(const std::vector<std::vector<double> >& landmarks, std::vector<double>& rerrors, ParseOptimizationResults& POR, string _pftset){
    last = POR.boat.size();
    std::vector<bool> split = SplitAnchors(landmarks, rerrors, POR, _pftset);
    MergeAnchors(POR, _pftset, split, landmarks);
}




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

*/
