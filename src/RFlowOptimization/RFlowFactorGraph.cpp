/*
 * RFlowFactorGraph.cpp
 *
 *  Created on: Jul 28, 2016
 *      Author: shane
 */



#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include "RFlowFactorGraph.hpp"
#include "AnchorISCFactor.h"

#include "LocalizationFactor.h"
#include "VirtualBetweenFactor.h"

void RFlowFactorGraph::InitializeNoiseModels(){
    gtsam::Vector6 v60;//GPS_NOISE, GPS_NOISE, 0.03, 0.05, 0.05, COMPASS_NOISE
    v60(0,0) = 0.2;
    v60(1,0) = 0.2;
    v60(2,0) = 0.2;
    v60(3,0) = 0.1;
    v60(4,0) = 0.1;
    v60(5,0) = 0.1;
    poseNoise0 = gtsam::noiseModel::Diagonal::Sigmas(v60);
    
    gtsam::Vector6 v61;//GPS_NOISE, GPS_NOISE, 0.03, 0.05, 0.05, COMPASS_NOISE
    v61(0,0) = 0.0001;
    v61(1,0) = 0.0001;
    v61(2,0) = 0.0001;
    v61(3,0) = 0.0001;
    v61(4,0) = 0.0001;
    v61(5,0) = 0.0001;
    poseNoise1 = gtsam::noiseModel::Diagonal::Sigmas(v61);
    
    gtsam::Vector6 v62;//GPS_NOISE, GPS_NOISE, 0.03, 0.05, 0.05, COMPASS_NOISE
    v62(0,0) = 10;
    v62(1,0) = 10;
    v62(2,0) = 0.03;//this value?
    v62(3,0) = 0.05;
    v62(4,0) = 0.05;
    v62(5,0) = 0.1745;
    poseNoiseP = gtsam::noiseModel::Diagonal::Sigmas(v62);
}

gtsam::Pose3 RFlowFactorGraph::VectorToPose(std::vector<double>& p) {
    return gtsam::Pose3(gtsam::Rot3::ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

gtsam::Symbol RFlowFactorGraph::GetSymbol(int survey, int pnum) {
    int s = pnum_to_ckey.size();
    int cnum = 0;
    auto search = surveycnum.find(survey);
    if(search != surveycnum.end()) {
        s = search->second;
        auto val = pnum_to_ckey[s].find(pnum);
        if(val != pnum_to_ckey[s].end()) {
            cnum = val->second;
        } else {
            cnum = ++lastcnums[s];
            pnum_to_ckey[s][pnum] = cnum;
        }
    } else {
        surveycnum[survey] = s;
        std::unordered_map<int, int> pto_ckey;
        pto_ckey[pnum] = cnum;
        pnum_to_ckey.push_back(pto_ckey);
        lastcnums.push_back(cnum);
    }

    return gtsam::Symbol((char) survey, cnum);
}

bool RFlowFactorGraph::VariableExists(int survey, int pnum) {
    auto search = surveycnum.find(survey);
    if(search != surveycnum.end()) {
        int s = search->second;
        auto val = pnum_to_ckey[s].find(pnum);
        if(val != pnum_to_ckey[s].end()) return true;
    }
    return false;
}

bool RFlowFactorGraph::AddPose(int survey, int pnum, gtsam::Pose3 p) {
    //only the p1frame0 poses are added, and only if they're not already added.
    if(VariableExists(survey, pnum)) return false;
    gtsam::Symbol s = GetSymbol(survey, pnum);
    return AddPose(s, p);
}

bool RFlowFactorGraph::AddPose(gtsam::Symbol s, gtsam::Pose3 p) {
    //NOTE: the poseNoise used here is what is used in the original FactorGraph.
    //Otherwise, RError is much worse, for inter, intra, or both.
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(s, p, poseNoiseP));
    return true;
}

void RFlowFactorGraph::AddAnchorFactor(int survey0, int anum0, int survey1, int anum1, gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 btwn, double val){
    gtsam::Symbol symb1 = GetSymbol(survey0, anum0);
    gtsam::Symbol symb3 = GetSymbol(survey1, anum1);
    gtsam::Vector6 v6;
    v6.setConstant(val);
    gtsam::noiseModel::Diagonal::shared_ptr btwnnoise = gtsam::noiseModel::Diagonal::Sigmas(v6);
    graph.add(AnchorISCFactor(symb1, symb3, p0, p1, btwn, btwnnoise));
}

void RFlowFactorGraph::AddLocalizationFactors(gtsam::Cal3_S2::shared_ptr k, int survey, int pnum, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers) {
    gtsam::Symbol symb = GetSymbol(survey, pnum);
    for(int i=0; i<p2d.size(); i++) {
        if(inliers[i]<0.001 || inliers[i]>6.0) continue;
        if(p3d[i].x() == 0.0 && p3d[i].y()==0.0 && p3d[i].z()==0.0) continue;
        gtsam::noiseModel::Isotropic::shared_ptr measurementNoise1 = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);//inliers[i]);
        graph.push_back(LocalizationFactor<gtsam::Pose3, gtsam::Cal3_S2>(p2d[i], p3d[i], measurementNoise1, symb, k));
    }
}

void RFlowFactorGraph::AddVirtualBTWNFactor(int survey0, int pnum0, int survey1, int pnum1, gtsam::Pose3 p0, gtsam::Pose3 p1, double val) {
    gtsam::Symbol symb1 = GetSymbol(survey0, pnum0);
    gtsam::Symbol symb3 = GetSymbol(survey1, pnum1);
    gtsam::Vector6 v6;
    v6.setConstant(val);
    gtsam::noiseModel::Diagonal::shared_ptr btwnnoise = gtsam::noiseModel::Diagonal::Sigmas(v6);
    graph.add(VirtualBetweenFactor(symb1, symb3, p0, p1, btwnnoise));
}

void RFlowFactorGraph::AddBTWNFactor(int survey0, int pnum0, int survey1, int pnum1, gtsam::Pose3 odom, bool tight) {
    gtsam::Symbol symb0 = GetSymbol(survey0, pnum0);
    gtsam::Symbol symb1 = GetSymbol(survey1, pnum1);
    if(tight) graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symb0, symb1, odom, poseNoise1));
    else graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symb0, symb1, odom, poseNoise0));
}

void RFlowFactorGraph::AddCustomBTWNFactor(int survey0, int pnum0, int survey1, int pnum1, gtsam::Pose3 odom, double val) {
    gtsam::Symbol symb0 = GetSymbol(survey0, pnum0);
    gtsam::Symbol symb1 = GetSymbol(survey1, pnum1);
    gtsam::Vector6 v6;
    v6.setConstant(val);
    gtsam::noiseModel::Diagonal::shared_ptr customnoise = gtsam::noiseModel::Diagonal::Sigmas(v6);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symb0, symb1, odom, customnoise));
}

void RFlowFactorGraph::Clear(){
    FactorGraph::Clear();
    lastcnums.clear();
    pnum_to_ckey.clear();
    surveycnum.clear();
}



