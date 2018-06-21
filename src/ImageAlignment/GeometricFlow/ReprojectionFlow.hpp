//
//  ReprojectionFlow.hpp
//  SIFTFlow
//
//  Created by Shane Griffith on 2/21/16.
//  Copyright © 2016 shane. All rights reserved.
//

#ifndef SRC_GEOMETRICFLOW_REPROJECTIONFLOW_HPP_
#define SRC_GEOMETRICFLOW_REPROJECTIONFLOW_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <gtsam/config.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>

#include <FileParsing/ParseFeatureTrackFile.h>
#include <Visualizations/IMDraw.hpp>
#include <DataTypes/Map.hpp>
#include <DataTypes/Camera.hpp>

class ReprojectionFlow{
private:
    double pose_distance_threshold = 5.0; //meters
    double pose_angle_threshold = 20.0;//degrees.

    bool DistanceCriterion(std::vector<double>& pose1, std::vector<double>& pose2);
    static double GStatistic(std::vector<int>& contingency_table);
    std::vector<int> ExtractContingencyTable(std::vector<bool>& vi1, std::vector<bool>& vi2);
    gtsam::Pose3 VectorToPose(std::vector<double>& p);
    std::vector<gtsam::Point2> ProjectPoints(std::vector<double>& boat, std::vector<bool>& valid_indices);
    void SparseFlow(std::vector<bool>& iA, std::vector<bool>& iB, std::vector<gtsam::Point2>& rpA, std::vector<gtsam::Point2>& rpB);
    void EliminateOutliers(int active_set);

    cv::Point2f Scale(cv::Point2f p, bool up);
    
    typedef struct{
    	gtsam::Point2 pim1;
    	gtsam::Point2 pim2;
    	gtsam::Point2 pflow;
    	int map_idx;
    	double rerror;
    } rfpoint;
    
    std::vector<std::vector<ReprojectionFlow::rfpoint> > restrictedset;
    std::vector<ReprojectionFlow::rfpoint> viewset;

    Map& _map;
    Camera& _cam;
public:
    bool debug = false;
    int outw=0, outh=0;
    
    /*takes as input the desired width and height for the flow.*/
    ReprojectionFlow(Camera& cam, Map& map):
    	_cam(cam), _map(map){
    	_map.CheckSize();
    }

    int MapSize(){return _map.map.size();}
    void Reset();
    double GStatisticForPose(std::vector<double>& camA, std::vector<double>& camB);
    void ComputeFlow(std::vector<double>& camA, std::vector<double>& camB);
    int IdentifyClosestPose(std::vector<std::vector<double> >& camA, std::vector<double>& camB, double * gresult=NULL, bool save=true);
    static void TestGstat();

    void OutputSize(int w, int h){outw=w; outh=h;}
    
    void CreateMirroredRestrictedSet(int res_set, int src_set);
    bool HaveTwoSets();
    void CreateRestrictedSet(int survey, ParseFeatureTrackFile& pft, bool forward=true);
    std::vector<cv::Point2f> GetRestrictedSetOrig(int active_set);
    std::vector<cv::Point2f> GetRestrictedSetMapped(int active_set);
    int GetNumberOfConstraints(int active_set);
    std::vector<double> GetConstraint(int active_set, int i);
    std::vector<double> GetAverageConstraint(int active_set);
    std::vector<double> GetMaxConstraint(int active_set);
    std::vector<double> GetConstraintBounds(int active_set);
    std::vector<double> GetkNNConstraint(int active_set, int x, int y, int k);
    
    std::vector<double> MeasureFlowAgreement(cv::Mat& flow, int active_set=0);
    std::vector<double> MeasureDeviationsPerSurvey(cv::Mat &flow);
    
    void DrawFlowPoints(cv::Mat& image, int active_set=0);
    void DrawMapPoints(cv::Mat& image, int active_set=0);
    void DrawFlowSurvey(cv::Mat& imageB, int survey);
    void DrawFlowSurvey(cv::Mat& imageB, int survey, ParseFeatureTrackFile& pft);
};







#endif /* SRC_GEOMETRICFLOW_REPROJECTIONFLOW_HPP_ */
