
/*
 * FactorsToConstraint.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: shane
 */

#include <FileParsing/ParseOptimizationResults.h>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <BoatSurvey/ParseBoatSurvey.hpp>
#include <gtsam/base/Vector.h>
#include <math.h>
#include <random>

#include "FactorsToConstraint.hpp"

using namespace std;

FactorsToConstraint::FactorsToConstraint(Camera& cam, std::string map_dir, std::string pftbase, std::string query_loc, std::string date):
_cam(cam), _map_dir(map_dir), _pftbase(pftbase), _query_loc(query_loc), _date(date)
{
    std::vector<double> vals = {
        10, 10, 0.03, 0.05, 0.05, 0.1745,
        0.15, 0.15, 0.05, 1.0, 1.0, 0.015,
        0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
        0.4, 0.4, 0.04, 0.025, 0.025, 0.02,
        1, 100, 100
    };
    odomvar = std::vector<double>(vals.begin()+12, vals.begin()+18);
    priorvar = std::vector<double>(vals.begin(), vals.begin()+6);
    posehood = std::vector<double>(vals.begin()+18, vals.begin()+24);
}

double FactorsToConstraint::GetLikelihood(gtsam::Pose3 val, gtsam::Pose3 expected, std::vector<double> var){
    gtsam::Pose3 nearzero = val.between(expected);
    gtsam::Pose3 zeros = gtsam::Pose3::identity();
    gtsam::Vector v = zeros.localCoordinates(nearzero);
    double res = 0;
    for(int i=0; i<v.size(); i++)
        res += pow(v[i],2)/var[i];
    res *= 0.5;
    return res;
}

double FactorsToConstraint::FeatureLikelihood(Camera& _cam, gtsam::Pose3 pose, std::vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& imagecoord, double var) {
    double dist = 0;
    for(int i=0; i<p3.size(); i++){
        if(p3[i].x()==0 && p3[i].y()==0 && p3[i].z()==0) continue;
        gtsam::Point3 p3_est = pose.transform_to(p3[i]);
        gtsam::Point2 p2_est = _cam.ProjectToImage(p3_est);
        dist += pow(imagecoord[i].x() - p2_est.x(),2) + pow(imagecoord[i].y() - p2_est.y(),2);
    }
    return 0.5 * dist/var;
}

double FactorsToConstraint::GetLikelihoodOdom(gtsam::Pose3 p1, gtsam::Pose3 p2, gtsam::Pose3 c1, gtsam::Pose3 c2, std::vector<double> var){
    return GetLikelihood(p1.between(p2), c1.between(c2), var);
}

double FactorsToConstraint::GetF(gtsam::Pose3 val, gtsam::Pose3 expected, std::vector<double> var){
    gtsam::Pose3 nearzero = val.between(expected);
    gtsam::Pose3 zeros = gtsam::Pose3::identity();
    gtsam::Vector v = zeros.localCoordinates(nearzero);
    double res = 1;
    for(int i=0; i<v.size(); i++)
        res *= pow(2*M_PI*var[i], -0.5) * exp(-0.5*pow(v[i],2)/var[i]);
    return res;
}

double FactorsToConstraint::FeatureF(Camera& _cam, gtsam::Pose3 pose, std::vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& imagecoord, double var){
    double res = 1;
    for(int i=0; i<p3.size(); i++){
        if(p3[i].x()==0 && p3[i].y()==0 && p3[i].z()==0) continue;
        gtsam::Point3 p3_est = pose.transform_to(p3[i]);
        gtsam::Point2 p2_est = _cam.ProjectToImage(p3_est);
        res *= pow(2*M_PI*var, -0.5) * exp(-0.5*(pow(imagecoord[i].x() - p2_est.x(),2) + pow(imagecoord[i].y() - p2_est.y(),2))/var); //underflow?
    }
    return res;
}

double FactorsToConstraint::GetFOdom(gtsam::Pose3 p1, gtsam::Pose3 p2, gtsam::Pose3 c1, gtsam::Pose3 c2, std::vector<double> var){
    return GetF(p1.between(p2), c1.between(c2), var);
}

double FactorsToConstraint::SampleValue(double u, double v){
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(u,v);
    return distribution(generator);
}

gtsam::Pose3 FactorsToConstraint::SamplePose(vector<double> mean, vector<double> var){
    vector<double> p(mean.size(), 0);
    for(int i=0; i<mean.size(); i++)
        p[i] = SampleValue(mean[i], var[i]);
    return gtsam::Pose3(gtsam::Rot3::ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

double FactorsToConstraint::MapToConstraint(double val){
    val /=3200;
    if(val>0) return 0.0001/val;
    else val *= -0.0001;
    return val;
}

void FactorsToConstraint::GetOffsets(){
    offsets = vector<gtsam::Pose3>(nsamples, gtsam::Pose3());
    vector<double> zeros(6, 0);
    for(int j=0; j<nsamples; j++)
        offsets[j] = SamplePose(zeros, posehood);
}

double FactorsToConstraint::GetConstraint(int i){
    if(i<0 || i > constraints.size()){
        std::cout << "FactorsToConstraint::GetConstraint() Error. " << std::endl;
        exit(-1);
    }
    return constraints[i];
}

void FactorsToConstraint::AcquireConstraints() {
    std::cout << " Acquiring constraints for " << _date << std::endl;
    
    ParseOptimizationResults POR(_map_dir + _date);
    ParseBoatSurvey PS(_query_loc, _pftbase, _date);
    constraints = vector<double>(POR.boat.size(), 0.0);
    
    for(int i=1; i<POR.boat.size(); i++) {
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
        
        double sum = 0;
        for(int j=0; j<nsamples; j++) {
            
            gtsam::Pose3 sampled = optposet1.compose(offsets[j]);//SamplePose(POR.boat[i], posehood);
            double lodom = GetLikelihoodOdom(optposet0, sampled, origposet0, origposet1, odomvar);
            lodom += GetLikelihoodOdom(sampled, optposet2, origposet1, origposet2, odomvar);
            double lprior = GetLikelihood(sampled, origposet1, priorvar);
            double lfeat = FeatureLikelihood(_cam, sampled, p3, pftf.imagecoord, 1.0);
            sum += (lodom + lprior + lfeat)-llhdo;
        }
        sum /= nsamples; //the smaller sum is, the larger the area around opt that's good, so the constraint can be looser.
        
        constraints[i] = MapToConstraint(sum);
        
        //std::cout << "rerror: " << rerrs[i] << " Likelihood["<<i<<"]: " << sum << ", constraint: " << constraint << std::endl;
    }
    constraints[0] = constraints[1];
}
