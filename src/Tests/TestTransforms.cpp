//
//  TestTransforms.cpp
//
//
//  Created by Shane Griffith on 8/11/17.
//  Copyright (c) 2017 Shane Griffith. All rights reserved.
//


#include "TestTransforms.hpp"
#include <RFlowOptimization/LocalizedPoseData.hpp>
#include <RFlowOptimization/LocalizePose.hpp>
#include <FileParsing/ParseOptimizationResults.hpp>



void TestTransforms::CheckBtwn(Camera& _cam){
    LocalizePose lp(_cam);
    
    vector<double> p = {5, 4, 0, 0.2, 0.1, 1.5};
    vector<double> t = {3, 4, 1, 0.3, 0.4, 1.3};
    gtsam::Pose3 gp = lp.VectorToPose(p);
    gtsam::Pose3 gt = lp.VectorToPose(t);
    std::cout << "p btwn t: " << gp.between(gt) << std::endl;
    std::cout << "t btwn p: " << gt.between(gp) << std::endl;
    std::cout << "(p btwn t)^-1: " << gp.between(gt).inverse() << std::endl;
}

void TestTransforms::TestLocalization(Camera& _cam){
    std::string lpd = "/cs-share/dream/results_consecutive/maps/140117/localizations/234.loc";
    LocalizedPoseData lpd = LocalizedPoseData::Read(lpd);
    ParseOptimizationResults POR0("/cs-share/dream/results_consecutive/maps/140106");
    ParseOptimizationResults POR1("/cs-share/dream/results_consecutive/maps/140117");
    
    LocalizePose lp(_cam);
    std::vector<std::vector<double> > candidates = lp.RobustDualBA(POR0.boat[lpd.s0time], POR1.boat[lpd.s1time],
                                                                   lpd.p3d0, lpd.p2d1, lpd.rerrorp, lpd.b3d1, lpd.b2d0, lpd.rerrorb);
    if(candidates.size()==0) return -1;
    
    perc_dc = candidates[2][1]/(count1+count2);
    if(debug) std::cout<<"localization quality check: \n\tForward and Backward:  "
        <<((int)1000*perc_dc)/10.0 << "% inliers with "
        << candidates[2][3] << " avg reprojection error" << std::endl;
    
}
