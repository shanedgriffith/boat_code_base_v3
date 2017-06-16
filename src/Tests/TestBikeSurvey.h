//
//  TestBikeSurvey.h
//  boat_code_base_v2
//
//  Created by Shane on 6/2/17.
//  Copyright © 2017 Shane. All rights reserved.
//

#ifndef TestBikeSurvey_h
#define TestBikeSurvey_h

#include <vector>

class TestBikeSurvey{
protected:
    
    cv::Scalar ColorByDistance(double dist);
    cv::Scalar ColorByHeight(double z);
    cv::Scalar GetLandmarkColor(int id);
    
    std::vector<double> YPRToRotationMatrix(double X, double Y, double Z);
    std::vector<double> ComposeRotationMatrices(std::vector<double> A, std::vector<double> B);
    std::vector<double> RotationMatrixToRPY(std::vector<double> R);
    std::vector<double> TransformPose(std::vector<double> p, int m1, int m2, int m3);
    
    std::vector<double> PoseToVector(gtsam::Pose3& cam);
    
    gtsam::Pose3 CameraPose(std::vector<double> p);
public:
    TestBikeSurvey(){}
    
    void TestTriangulation();
    void TestVisualOdometry();
};



#endif /* TestBikeSurvey_h */
