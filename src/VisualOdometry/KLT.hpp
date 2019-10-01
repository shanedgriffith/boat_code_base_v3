//
//  Cedric's code (adapted from ViSP), with ROS removed, and now in OpenCV.
//
//  This version created by Shane Griffith on 3/20/17.
//

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <map> //unordered_map
#include <set>

#include <DataTypes/Camera.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>

//Features detected using SIFT. Tracked using LT.
class KLT {
protected:
    typedef struct{
        int id;
        float x, y;
    } feature_positions;
    
    std::vector<long> fid;      //!< Keypoint id
    long m_next_points_id;
    std::vector<std::vector<cv::Point2f> > m_points; //!< Previous [0] and current [1] keypoint location
    cv::Mat prev_view;
    
    cv::TermCriteria m_termcrit;
    int m_winSize = 10;
    float m_qualityLevel = 0.05;
    int m_minDistance = 15;
    float m_harris_k = 0.04;
    int m_blockSize = 9;
    int m_useHarrisDetector = 1;
    int m_pyrMaxLevel = 3;
    
    int grid_rows, grid_cols, max_features, max_features_per_cell;
    float grid_delta_x,grid_delta_y;
    int print_feature;
    double max_feature_jump, max_feature_dy;
    double max_distance_fundamental, confidence_fundamental;
    bool first_run,display,display_image,display_grid,display_tracks,display_features,publish;
    bool filter_with_fundamental;
    std::vector< std::vector<long> > grid, grid_new;
    typedef std::map<long, KLT::feature_positions> KLTMap; //unordered_map
    KLTMap all_points;
    typedef std::vector<cv::Point2f> KPPath;
    typedef std::map<long, KPPath> KPMap;
    KPMap all_keypoints;
    long maxid;
    
    // for optical flow
    bool optflow;
    cv::Mat_<cv::Vec2f> flow;
    cv::Mat_<float> flowmag;
    float max_flow;
    
    void AddFeature(long id, float x, float y);
    void reset_grids();
    bool recordKeypoint(int id,float x,float y);
    size_t ransacTest();
    void SuppressFeature(int index);
    
    void InitializeTracking(cv::Mat& view);
    bool track(cv::Mat &view);
    void ContinueTracking(cv::Mat& view);
    std::vector<KLT::feature_positions> UpdateCurrentPointSet();
    void PrintPointSet();

    const Camera& _cam;
public:
    
    KLT(const Camera& cam):_cam(cam) {
        m_points = std::vector<std::vector<cv::Point2f> >(2);
        print_feature = -1;
        m_next_points_id = 0;
        filter_with_fundamental = true;
        maxid = -1;
        max_distance_fundamental = 3.0;
        confidence_fundamental = 0.98;
        
        m_termcrit = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
        m_minDistance = 15;
        m_winSize = 10;
        m_qualityLevel = 0.05;
        m_harris_k = 0.04;
        m_blockSize = 9;
        m_useHarrisDetector = 1;
        m_pyrMaxLevel = 3;

        grid_rows = 12;     //should be dependent on feature size.
        grid_cols = 20;
        max_features = 300;
        max_features_per_cell = 5;
        max_feature_jump = 100.0;
        max_feature_dy = 20.0;
    }
    
    void SetSizes(std::vector<double> tenparams);

    ParseFeatureTrackFile TrackKLTFeatures(cv::Mat& view_color, std::string base, int image_num, double timestamp);
    
    void DrawFeatures(cv::Mat& image);
};



