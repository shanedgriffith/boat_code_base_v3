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
#include <FileParsing/ParseOptimizationResults.h>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <BoatSurvey/ParseBoatSurvey.hpp>



using namespace std;

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
    std::string lpdfile = "/cs-share/dream/results_consecutive/maps/140117/localizations/289.loc";
    LocalizedPoseData lpd = LocalizedPoseData::Read(lpdfile);
    ParseOptimizationResults POR0("/cs-share/dream/results_consecutive/maps/140106");
    ParseOptimizationResults POR1("/cs-share/dream/results_consecutive/maps/140117");
    
    LocalizePose lp(_cam);
    lp.debug = true;
    std::vector<std::vector<double> > candidates = lp.RobustDualBA(POR0.boat[lpd.s0time], POR1.boat[lpd.s1time],
                                                                   lpd.p3d0, lpd.p2d1, lpd.rerrorp, lpd.b3d1, lpd.b2d0, lpd.rerrorb);
    if(candidates.size()==0) return;
    
    double perc_dc = candidates[2][1]/(lpd.p3d0.size()+lpd.b3d1.size());
    std::cout<<"localization quality check: \n\tForward and Backward:  "
        <<((int)1000*perc_dc)/10.0 << "% inliers with "
        << candidates[2][3] << " avg reprojection error" << std::endl;
    
}


double TestTransforms::GetLikelihood(gtsam::Pose3 val, gtsam::Pose3 expected, std::vector<double> var){
    gtsam::Vector v = expected.localCoordinates(val);
    double res = 0;
    for(int i=0; i<v.size(); i++)
        res += pow(v[i],2)/var;
    res *= 0.5;
    return res;
}

double TestTransforms::FeatureLikelihood(Camera& _cam, gtsam::Pose3 pose, std::vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& imagecoord, double var) {
    double llhd = 0;
    for(int i=0; i<p3.size(); i++){
        if(p3[i].x()==0 && p3[i].y()==0 && p3[i].z()==0) continue;
        gtsam::Point3 p3_est = pose.transform_to(p3[i]);
        gtsam::Point2 p2_est = _cam.ProjectToImage(p3_est);
        llhd += (pow(imagecoord[i].x() - p2_est.x(),2) + pow(imagecoord[i].y() - p2_est.y(),2))/var;
    }
    return 0.5 * llhd;
}

double TestTransforms::GetLikelihoodOdom(gtsam::Pose3 p1, gtsam::Pose3 p2, gtsam::Pose3 c1, gtsam::Pose3 c2, std::vector<double> var){
    return GetLikelihood(p1.between(p2), c1.between(c2), var);
}

void TestTransforms::TestConstraintProportions(Camera& _cam){
    ParseOptimizationResults POR("/cs-share/dream/results_consecutive/maps/140106");
    ParseBoatSurvey PS("/mnt/tale/cedricp/VBags/", "/mnt/tale/shaneg/Lakeshore_KLT/", "140106");
    
    std::vector<double> vals = {
        10, 10, 0.03, 0.05, 0.05, 0.1745,
        0.15, 0.15, 0.05, 1.0, 1.0, 0.015,
        0.05, 0.05, 10, 1.0, 1.0, 0.25,
        0.4, 0.4, 0.04, 0.025, 0.025, 0.02,
        1, 100, 100
    };
    
    for(int i=0; i<10; i++) {
        int aidx = POR.auxidx[i];
        
        double llhd=0;
        gtsam::Pose3 origposet1 = PS.CameraPose(i);
        gtsam::Pose3 origposet2 = PS.CameraPose(i+1);
        gtsam::Pose3 optposet1 = POR.CameraPose[i];
        gtsam::Pose3 optposet2 = POR.CameraPose[i+1];
        std::vector<double> odomvar(var.begin()+12, var.begin()+18);
        std::vector<double> priorvar(var.begin(), var.begin()+6);
        if(i>0) {
            gtsam::Pose3 origposet0 = PS.CameraPose(i-1);
            gtsam::Pose3 optposet0 = POR.CameraPose[i-1];
            llhd += GetLikelihoodOdom(optposet0, optposet1, origposet0, origposet1, odomvar);
        }
        llhd += GetLikelihoodOdom(optposet1, optposet2, origposet1, origposet2, odomvar);
        llhd += GetLikelihood(optposet1, origposet1, priorvar);
        ParseFeatureTrackFile pftf = PS.LoadVisualFeatureTracks(_cam, POR.ftfilenos[i]);
        std::vector<gtsam::Point3> p3 = POR.GetSubsetOf3DPoints(pftf.ids);
        llhd += FeatureLikelihood(_cam, optposet1, p3, pftf.imagecoord, 1.0);
        std::cout << "Likelihood["<<i<<"]: " << llhd << std::endl;
    }
    
    
}
