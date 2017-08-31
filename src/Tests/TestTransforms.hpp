//
//  TestTransforms.hpp
//
//
//  Created by Shane Griffith on 8/11/17.
//  Copyright (c) 2017 Shane Griffith. All rights reserved.
//


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

#include <DataTypes/Camera.hpp>

class TestTransforms {
private:
    double GetLikelihood(gtsam::Pose3 val, gtsam::Pose3 expected, std::vector<double> var);
    double FeatureLikelihood(Camera& _cam, gtsam::Pose3 pose, std::vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& imagecoord, double var);
    double GetLikelihoodOdom(gtsam::Pose3 p1, gtsam::Pose3 p2, gtsam::Pose3 c1, gtsam::Pose3 c2, std::vector<double> var);
public:
    TestTransforms();
    static void CheckBtwn(Camera& _cam);
    static void TestLocalization(Camera& _cam);
    static void TestConstraintProportions(Camera& _cam);
};










