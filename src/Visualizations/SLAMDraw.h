//
//  SLAMDrawingFunctions.h
//  BeginningSLAM
//
//  Created by Shane Griffith on 1/21/14.
//  Copyright (c) 2014 Shane Griffith. All rights reserved.
//

#pragma once

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"

class SLAMDraw {
private:
    const cv::Scalar WATER_COLOR = CV_RGB(50,50,255);
    const cv::Scalar SHORE_COLOR = CV_RGB(255,211,155);
    const cv::Scalar LANDMARK_COLOR = CV_RGB(139,69,19);
    const cv::Scalar BOAT_COLOR = CV_RGB(0,0,0);
    const cv::Scalar SIGHT_COLOR = CV_RGB(255,255,255);
    const cv::Scalar SEEN_COLOR = CV_RGB(255,255,255);
    const cv::Scalar LAND_COLOR = CV_RGB(100,169,100);
    const cv::Scalar RAY_COLOR = CV_RGB(0,255,255);
    
    const cv::Scalar STARBOARD_COLOR = CV_RGB(0,255,0);
    const cv::Scalar PORT_COLOR = CV_RGB(255,0,0);
    
    const int POINT_IS_ABOVE_LINE = -1;
    const int POINT_IS_BELOW_LINE = 1;
    
    constexpr static double FOV = 0.86847586; //in radians, computed using the camera info.
    
    //canvas size
    int img_w, img_h;
    
    int boat_l = 6;
    int boat_w = 4;
    
    cv::Mat canvas;
    
    double scale_x_max = -10000000.0;
    double scale_x_min = 10000000.0;
    double scale_y_max = -10000000.0;
    double scale_y_min = 10000000.0;
    
    //use the yaw offset of M_PI_2 if the camera is passed, otherwise use 0 if the boat is passed.
    //later I could fix this by separting the angles used for drawing the boat and the camera.
    double YAW_OFFSET = M_PI_2;
    
    char ShowImage(char * window_name, cv::Mat img, bool flipped);
    void SaveImage(char * filename, cv::Mat img, bool flipped);
    
    void TestGRID();
    double ScaleToDisplay(double val, double min, double max, double display_size);

    std::vector<double> GetEquationOfLineForTest(int test, double x, double y, double yaw, double field_of_view=FOV);

    std::vector<double> EquationOfLine(cv::Point2d p1, cv::Point2d p2);

    int LocationOfPointRelativeToLine(double m, double b, cv::Point2d p);
    
    
    
    void DrawBoat(double x, double y, double yaw);
    
    void DrawUpwardSight(double img_width, double field_of_view=FOV);
    
    CvScalar GetLandmarkColor(int id);
    
public:

    SLAMDraw(int w = 500, int h = 500) : img_w(w), img_h(h) {
        canvas = cv::Mat(img_w, img_h, CV_8UC3, LAND_COLOR);
    }
    
    enum class shape{CIRCLE, SQUARE, X};
    
    void ResetCanvas();
    
    char Display();
    
    void SaveDrawing(std::string filename);
    cv::Mat GetDrawing();
    
    cv::Point2f Scale(cv::Point2f p);
    void SetScale(double xmin, double xmax, double ymin, double ymax);
    
    void SetScale(std::vector<cv::Point3d> boatpos);
    
    void AddPointLandmark(double x, double y, int l = -1);
    void AddPointPath(double x, double y,  int r=-1, int g=-1, int b=-1);
    
    void TestCanvas(std::vector<cv::Point3d> boatpos);
    
    void DrawSight(double x, double y, double yaw, double field_of_view=FOV, int r=-1, int g=-1, int b=-1);
    void ProjectRay(double x, double y, double yaw, double x_ratio, double field_of_view=FOV);
    
    void PlotTriangulation(double x1, double y1, double yaw1, double x2, double y2, double yaw2, cv::Point p, cv::Point p1, cv::Point p2);
    
    //functions for mapping the alignment accuracy.
    void AddShape(SLAMDraw::shape s, double x, double y, int r, int g, int b);
    void AddArrow(double x, double y, double ax, double ay);
//    void AddCircle(double x, double y, int r, int g, int b);
//    void AddSquare(double x, double y, int r, int g, int b);
//    void AddX(double x, double y, int r, int g, int b);
    void PutText(double x, double y, std::string text);
};

