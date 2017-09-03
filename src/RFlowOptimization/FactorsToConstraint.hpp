//
//  FactorsToConstraint.hpp
//
//  Created by Shane Griffith on 8/31/17.
//  Copyright (c) 2017 shane. All rights reserved.
//

#ifndef SRC_RFLOWOPTIMIZATION_FACTORSTOCONSTRAINT_HPP_
#define SRC_RFLOWOPTIMIZATION_FACTORSTOCONSTRAINT_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <DataTypes/Camera.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

class FactorsToConstraint {
protected:
    double GetLikelihood(gtsam::Pose3 val, gtsam::Pose3 expected, std::vector<double> var);
    double FeatureLikelihood(Camera& _cam, gtsam::Pose3 pose, std::vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& imagecoord, double var);
    double GetLikelihoodOdom(gtsam::Pose3 p1, gtsam::Pose3 p2, gtsam::Pose3 c1, gtsam::Pose3 c2, std::vector<double> var);
    
    double GetF(gtsam::Pose3 val, gtsam::Pose3 expected, std::vector<double> var);
    double FeatureF(Camera& _cam, gtsam::Pose3 pose, std::vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& imagecoord, double var);
    double GetFOdom(gtsam::Pose3 p1, gtsam::Pose3 p2, gtsam::Pose3 c1, gtsam::Pose3 c2, std::vector<double> var);
    
    double SampleValue(double u, double v);
    gtsam::Pose3 SamplePose(std::vector<double> mean, std::vector<double> var);
    double MapToConstraint(double val);
    
    std::vector<double> odomvar;
    std::vector<double> priorvar;
    std::vector<double> posehood;
    std::vector<gtsam::Pose3> offsets;
    
    std::vector<double> constraints;
    
    const int nsamples = 100;
    
    Camera& _cam;
public:
    std::string _map_dir, _date, _pftbase, _query_loc;
    
    FactorsToConstraint(Camera& cam, std::string map_dir, std::string pftbase, std::string query_loc, std::string date);
    
    void AcquireConstraints();
    
    double GetConstraint(int i);
};



#endif /* SRC_RFLOWOPTIMIZATION_FACTORSTOCONSTRAINT_HPP_ */
