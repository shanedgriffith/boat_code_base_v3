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
#include <gtsam/base/Vector.h>
#include <math.h>
#include <random>
#include <DataTypes/Map.hpp>
#include <RFlowEvaluation/AlignImageMachine.hpp>

#include "Optimization/SingleSession/EvaluateSLAM.h"
#include "Optimization/SingleSession/GTSamInterface.h"

using namespace std;

void TestTransforms::CheckBtwn(Camera& _cam){
    vector<double> p = {5, 4, 0, 0.2, 0.1, 1.5};
    vector<double> t = {3, 4, 1, 0.3, 0.4, 1.3};
    gtsam::Pose3 gp = GTSamInterface::VectorToPose(p);
    gtsam::Pose3 gt = GTSamInterface::VectorToPose(t);
    std::cout << "p btwn t: " << gp.between(gt) << std::endl;
    std::cout << "t btwn p: " << gt.between(gp) << std::endl;
    std::cout << "(p btwn t)^-1: " << gp.between(gt).inverse() << std::endl;
}

void TestTransforms::TestLocalization(Camera& _cam){
    std::string lpdfile = "/cs-share/dream/results_consecutive/maps/140117/localizations/289.loc";
    LocalizedPoseData lpd = LocalizedPoseData::Read(lpdfile);
    ParseOptimizationResults POR0("/cs-share/dream/results_consecutive/maps/", "140106");
    ParseOptimizationResults POR1("/cs-share/dream/results_consecutive/maps/", "140117");
    
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
    gtsam::Pose3 nearzero = val.between(expected);
    gtsam::Pose3 zeros = gtsam::Pose3::identity();
    gtsam::Vector v = zeros.localCoordinates(nearzero);
    double res = 0;
    for(int i=0; i<v.size(); i++)
        res += pow(v[i],2)/var[i];
    res *= 0.5;
    return res;
}

double TestTransforms::FeatureLikelihood(Camera& _cam, gtsam::Pose3 pose, std::vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& imagecoord, double var) {
    double dist = 0;
    for(int i=0; i<p3.size(); i++){
        if(p3[i].x()==0 && p3[i].y()==0 && p3[i].z()==0) continue;
        gtsam::Point3 p3_est = pose.transform_to(p3[i]);
        gtsam::Point2 p2_est = _cam.ProjectToImage(p3_est);
        dist += pow(imagecoord[i].x() - p2_est.x(),2) + pow(imagecoord[i].y() - p2_est.y(),2);
    }
    return 0.5 * dist/var;
}

double TestTransforms::GetLikelihoodOdom(gtsam::Pose3 p1, gtsam::Pose3 p2, gtsam::Pose3 c1, gtsam::Pose3 c2, std::vector<double> var){
    return GetLikelihood(p1.between(p2), c1.between(c2), var);
}

double TestTransforms::GetF(gtsam::Pose3 val, gtsam::Pose3 expected, std::vector<double> var){
    gtsam::Pose3 nearzero = val.between(expected);
    gtsam::Pose3 zeros = gtsam::Pose3::identity();
    gtsam::Vector v = zeros.localCoordinates(nearzero);
    double res = 1;
    for(int i=0; i<v.size(); i++)
        res *= pow(2*M_PI*var[i], -0.5) * exp(-0.5*pow(v[i],2)/var[i]);
    return res;
}

double TestTransforms::FeatureF(Camera& _cam, gtsam::Pose3 pose, std::vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& imagecoord, double var){
    double res = 1;
    for(int i=0; i<p3.size(); i++){
        if(p3[i].x()==0 && p3[i].y()==0 && p3[i].z()==0) continue;
        gtsam::Point3 p3_est = pose.transform_to(p3[i]);
        gtsam::Point2 p2_est = _cam.ProjectToImage(p3_est);
        res *= pow(2*M_PI*var, -0.5) * exp(-0.5*(pow(imagecoord[i].x() - p2_est.x(),2) + pow(imagecoord[i].y() - p2_est.y(),2))/var); //underflow?
    }
    return res;
}

double TestTransforms::GetFOdom(gtsam::Pose3 p1, gtsam::Pose3 p2, gtsam::Pose3 c1, gtsam::Pose3 c2, std::vector<double> var){
    return GetF(p1.between(p2), c1.between(c2), var);
}

double TestTransforms::SampleValue(double u, double v){
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(u,v);
    return distribution(generator);
}

gtsam::Pose3 TestTransforms::SamplePose(vector<double> mean, vector<double> var){
    vector<double> p(mean.size(), 0);
    for(int i=0; i<mean.size(); i++)
        p[i] = SampleValue(mean[i], var[i]);
    return GTSamInterface::VectorToPose(p);
}

double TestTransforms::MapToConstraint(double val){
    val /=3200;
    if(val>0) return 0.0001/val;
    else val *= -0.0001;
    return val;
}

void TestTransforms::TestConstraintProportions(Camera& _cam){
    int nsamples = 100;
    ParseOptimizationResults POR("/cs-share/dream/results_consecutive/maps/", "140106");
    ParseBoatSurvey PS("/mnt/tale/cedricp/VBags/", "/mnt/tale/shaneg/Lakeshore_KLT/", "140106");
    EvaluateSLAM ESlam(_cam, "140106", "/cs-share/dream/results_consecutive/maps/");
    vector<double> rerrs = ESlam.LoadRerrorFile();
    vector<gtsam::Pose3> offsets(nsamples, gtsam::Pose3());
    
    std::vector<double> vals = {
        10, 10, 0.03, 0.05, 0.05, 0.1745,
        0.15, 0.15, 0.05, 1.0, 1.0, 0.015,
        0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
        0.4, 0.4, 0.04, 0.025, 0.025, 0.02,
        1, 100, 100
    };
    std::vector<double> odomvar(vals.begin()+12, vals.begin()+18);
    std::vector<double> priorvar(vals.begin(), vals.begin()+6);
    std::vector<double> posehood(vals.begin()+18, vals.begin()+24);
    
    vector<double> zeros(6, 0);
    for(int j=0; j<nsamples; j++)
        offsets[j] = SamplePose(zeros, posehood);
    
    
    for(int i=1; i<500; i++) {
        int aidx = POR.auxidx[i];
        
        ParseFeatureTrackFile pftf = PS.LoadVisualFeatureTracks(_cam, POR.ftfilenos[i]);
        std::vector<gtsam::Point3> p3 = POR.GetSubsetOf3DPoints(pftf.ids);
        
        gtsam::Pose3 origposet0 = PS.CameraPose(POR.auxidx[i-1]);
        gtsam::Pose3 origposet1 = PS.CameraPose(POR.auxidx[i]);
        gtsam::Pose3 origposet2 = PS.CameraPose(POR.auxidx[i+1]);
        
        gtsam::Pose3 optposet0 = POR.CameraPose(i-1);
        gtsam::Pose3 optposet2 = POR.CameraPose(i+1);
        gtsam::Pose3 optposet1 = POR.CameraPose(i);
        
        double lodomo = GetLikelihoodOdom(optposet0, optposet1, origposet0, origposet1, odomvar);
        lodomo += GetLikelihoodOdom(optposet1, optposet2, origposet1, origposet2, odomvar);
        double lprioro = GetLikelihood(optposet1, origposet1, priorvar);
        double lfeato = FeatureLikelihood(_cam, optposet1, p3, pftf.imagecoord, 1.0);
        double llhdo = lodomo + lprioro + lfeato;
        //<<"= " << lprioro << " + " << lodomo << " + " << lfeato << std::endl;
        
        double sum = 0;
        for(int j=0; j<nsamples; j++) {
            
            gtsam::Pose3 sampled = optposet1.compose(offsets[j]);//SamplePose(POR.boat[i], posehood);
            double lodom = GetLikelihoodOdom(optposet0, sampled, origposet0, origposet1, odomvar);
            lodom += GetLikelihoodOdom(sampled, optposet2, origposet1, origposet2, odomvar);
            double lprior = GetLikelihood(sampled, origposet1, priorvar);
            double lfeat = FeatureLikelihood(_cam, sampled, p3, pftf.imagecoord, 1.0);
            sum += (lodom + lprior + lfeat)-llhdo;
        }
        sum /= 100; //the smaller sum is, the larger the area around opt that's good, so the constraint can be looser.
        
        double constraint = MapToConstraint(sum);
        
        std::cout << "rerror: " << rerrs[i] << " Likelihood["<<i<<"]: " << sum << ", constraint: " << constraint << std::endl;
    }
}

void TestTransforms::TestImageAlignment(Camera& _cam, std::string query_loc, std::string results_dir, std::string pftbase) {
    std::string d1 = "141010";
    int im1 = 18791;
    std::string d2 = "140129";
    int im2 = 14029;
    
    AlignImageMachine aim(_cam);
    aim.SetDirs(pftbase, query_loc, results_dir);
    std::string maps_dir(results_dir + "maps/");
    
    ParseOptimizationResults por1(maps_dir, d1);
    ParseOptimizationResults por2(maps_dir, d2);
    Map m1(maps_dir);
    m1.LoadMap(d1);
    Map m2(maps_dir);
    m2.LoadMap(d2);
    
    std::string savename = "/Users/shane/Desktop/";// + d1 + "." + to_string(im1) + "_" + d2 + "." + to_string(im2)
    
    int pose1 = por1.GetNearestPoseToImage(im1);
    int pose2 = por2.GetNearestPoseToImage(im2);
    
    aim.Setup(pose1, savename, pose2);
    aim.SetMaps({&m1, &m2});
    aim.SetDates({d1, d2});
    aim.SetPOR({&por1, &por2});
    aim.Run();
    aim.LogResults();
}



