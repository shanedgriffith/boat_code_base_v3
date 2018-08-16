//
//  SLAMDraw.cpp
//  BeginningSLAM
//
//  Created by Shane Griffith on 1/21/14.
//  Copyright (c) 2014 Shane Griffith. All rights reserved.
//

#include "SLAMDraw.h"

#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;


char SLAMDraw::ShowImage(char * window_name, Mat img, bool flipped){
    /*Show the specified image. flipped specifies the orientation of the image in the y channel.*/
    
    if(flipped) {
        Mat flipped;
        flip(img, flipped, 0);
        img = flipped;
    }
    
    cvNamedWindow(window_name);
    imshow(window_name, img);
    
    char c = cvWaitKey();
    if(c=='q') {
        cvDestroyAllWindows();
        exit(0);
    } else if(c=='b') {
        cout <<"bad match\n";
    } else if(c=='g') {
        cout<<"good match\n";
    }
    return c;
}

void SLAMDraw::SaveDrawing(string filename){
    SaveImage((char *) filename.c_str(), canvas, true);
}

void SLAMDraw::SaveImage(char * filename, Mat img, bool flipped) {
    if(flipped) {
        Mat flipped;
        flip(img, flipped, 0);
        img = flipped;
    }
    
    imwrite((const char*)filename, img);
}

void SLAMDraw::TestCanvas(vector<Point3d> boatpos) {
    for(int i=1; i<boatpos.size(); i++) {
        Point2f from = Scale(Point2f(boatpos[i-1].x, boatpos[i-1].y));
        Point2f to = Scale(Point2f(boatpos[i].x, boatpos[i].y));
        line(canvas, from, to, SIGHT_COLOR);
    }
    
    //opencv drawing thinks y=0 is at the top of the image, so we have to flip it...
    ShowImage((char *) "test canvas with the boat path", canvas, true);
}

void SLAMDraw::ResetCanvas() {
    canvas = Mat(img_w, img_h, CV_8UC3, LAND_COLOR);
}

void SLAMDraw::ProjectRay(double x, double y, double yaw, double x_ratio, double field_of_view) {
    /*Projects a ray through x_ratio, which is the x pixel coordinate/image width*/
    Point2f bpt = Scale(Point2f(x, y));
    
    double SIGHT_LENGTH = 1000;

    //ahh, the field of view should be shrunk in the x direction if there's more space
    //compared to the y direction. How to model that? maybe it already is?
    
    double angle = yaw + YAW_OFFSET + (0.5 - x_ratio) * field_of_view;
    Point2f trns = Scale(Point2f(SIGHT_LENGTH * cos(angle), SIGHT_LENGTH * sin(angle)));
    double px = bpt.x + trns.x;
    double py = bpt.y + trns.y;
    line(canvas, bpt, Point2f(px,py), RAY_COLOR);
}

char SLAMDraw::Display(){
    return ShowImage((char *) "SLAM map", canvas, true);
}

void SLAMDraw::AddPointLandmark(double x, double y, int l) {
    CvScalar color;
    if(l==-1) color = LANDMARK_COLOR;
    else color = GetLandmarkColor(l);
    
    circle(canvas, Scale(Point2f(x, y)), 2, color, -1, 8, 0);
}

void SLAMDraw::AddPointPath(double x, double y, int r, int g, int b) {
    cv::Scalar linecol = BOAT_COLOR;
    if(r>-1) linecol = CV_RGB(r,g,b);
    circle(canvas, Scale(Point2f(x, y)), 2, linecol, -1, 8, 0);
}

void SLAMDraw::SetScale(vector<Point3d> boatpos){
    /*Find the max and min x,y coordinates for plotting */
    
    //finds values that put the boat
    for(int i=0; i<boatpos.size(); i++) {
        if(boatpos[i].x < scale_x_min) scale_x_min = boatpos[i].x;
        if(boatpos[i].x > scale_x_max) scale_x_max = boatpos[i].x;
        if(boatpos[i].y < scale_y_min) scale_y_min = boatpos[i].y;
        if(boatpos[i].y > scale_y_max) scale_y_max = boatpos[i].y;
    }
}

void SLAMDraw::SetScale(double xmin, double xmax, double ymin, double ymax){
    scale_x_min = xmin;
    scale_x_max = xmax;
    scale_y_min = ymin;
    scale_y_max = ymax;
}

double SLAMDraw::ScaleToDisplay(double val, double min, double max, double display_size){
    return display_size * (val - min)/(max-min);
}

Point2f SLAMDraw::Scale(Point2f p){
    if(scale_x_max < scale_x_min) {printf("The scale isn't set.\n"); exit(-1);}
    double x = ScaleToDisplay(p.x, scale_x_min, scale_x_max, img_w);
    double y = ScaleToDisplay(p.y, scale_y_min, scale_y_max, img_h);
    return Point2f(x,y);
}

void SLAMDraw::TestGRID(){
    double xbin = (scale_x_max - scale_x_min)/9;
    double ybin = (scale_y_max - scale_y_min)/9;
    
    for(int i=0; i<10; i++) {
        for(int j=0; j<10; j++) {
            circle(canvas, Point2f(scale_x_min+xbin*i, scale_y_min+ybin*j), 2, LANDMARK_COLOR, -1, 8, 0);
        }
    }
}

void SLAMDraw::DrawBoat(double x, double y, double yaw_raw) {
    /*Draw the boat. OpenCV's rectangle function assumes the bounding box orientation, so this
     function uses the fillConvexPoly method*/
    
    double dist_to_corner = sqrt(pow(boat_l/2.0,2.0) + pow(boat_w/2.0,2.0));
    double theta = acos((boat_w/2.0)/dist_to_corner);
    
    Point pts[4];
    
    //my real implementation uses camera_pose in place of boat_pose (which is wrong)...
    
    //add pi/2 to get back to the boat pose
    //add another pi/2 to get from pointing right to pointing up.
    double yaw = yaw_raw + YAW_OFFSET + M_PI_2;
    
    Point2f tr;
    tr.x = x + dist_to_corner*cos(yaw - M_PI/2.0 + theta);
    tr.y = y + dist_to_corner*sin(yaw - M_PI/2.0 + theta);
    
    Point2f tl;
    tl.x = x + dist_to_corner*cos(yaw + M_PI/2.0 - theta);
    tl.y = y + dist_to_corner*sin(yaw + M_PI/2.0 - theta);
    
    Point2f br;
    br.x = tl.x - 2*dist_to_corner*cos(yaw + M_PI/2.0 - theta);
    br.y = tl.y - 2*dist_to_corner*sin(yaw + M_PI/2.0 - theta);
    
    Point2f bl;
    bl.x = tr.x - 2*dist_to_corner*cos(yaw - M_PI/2.0 + theta);
    bl.y = tr.y - 2*dist_to_corner*sin(yaw - M_PI/2.0 + theta);
    
    pts[0] = Scale(tr);
    pts[1] = Scale(tl);
    pts[2] = Scale(bl);
    pts[3] = Scale(br);
    
    fillConvexPoly(canvas, pts, 4, BOAT_COLOR);
    circle(canvas, pts[0], 2, STARBOARD_COLOR, -1, 8, 0);
    circle(canvas, pts[1], 2, PORT_COLOR, -1, 8, 0);
}

void SLAMDraw::DrawSight(double x, double y, double yaw, double field_of_view, int r, int g, int b) {
    cv::Scalar linecol = SIGHT_COLOR;
    if(r>-1) linecol = CV_RGB(r,g,b);
    
    /*Draws the line of sight from the camera on the center of the boat.*/
    double SIGHT_LENGTH = 1000;
    
    //add pi/2 to get from pointing right to pointing up.
    double angle = yaw+YAW_OFFSET;
    double top = angle + field_of_view/2.0;
    double bottom = angle - field_of_view/2.0;
    
    Point2f boat = Scale(Point2f(x, y));
    Point2f origin = Scale(Point2f(0, 0));
    
    Point2d topline;
    Point2f topsight = Scale(Point2f(SIGHT_LENGTH*cos(top), SIGHT_LENGTH*sin(top)));
    topline.x = (boat.x-origin.x) + topsight.x;
    topline.y = (boat.y-origin.y) + topsight.y;
    
    Point2d botline;
    Point2f botsight = Scale(Point2f(SIGHT_LENGTH*cos(bottom), SIGHT_LENGTH*sin(bottom)));
    botline.x = (boat.x-origin.x) + botsight.x;
    botline.y = (boat.y-origin.y) + botsight.y;
    
    line(canvas, boat, topline, linecol);
    line(canvas, boat, botline, linecol);
}

void SLAMDraw::DrawUpwardSight(double img_width, double field_of_view) {
    /*Draws the line of sight from the camera on the center of the boat.*/
    double SIGHT_LENGTH = 1000;
    
    double angle = YAW_OFFSET;
    double top = angle + field_of_view/2.0;
    double bottom = angle - field_of_view/2.0;
    
    printf("boat pose: %lf\n", angle);
    printf("top: %lf, bottom: %lf\n", top, bottom);
    
    Point2d leftline;
    leftline.x = img_width/4.0 + SIGHT_LENGTH*cos(top);
    leftline.y = SIGHT_LENGTH*sin(top);
    
    Point2d rightline;
    rightline.x = img_width/4.0 + SIGHT_LENGTH*cos(bottom);
    rightline.y = SIGHT_LENGTH*sin(bottom);
    
    Point2d boat(img_width/4.0, 0.0);
    
    line(canvas, boat, leftline, SIGHT_COLOR);
    line(canvas, boat, rightline, SIGHT_COLOR);
}

CvScalar SLAMDraw::GetLandmarkColor(int id){
    /*Disperses colors among landmarks according to their ids. The implementation works for
     the case in which two points lying nearby one another have very similar landmark ids*/
    int red = (71*(id%10) + id%255)%255;
    int green = (111*(id%10) + (2*id)%255)%255;
    int blue = (27*(id%10) + (3*id)%255)%255;
    return CV_RGB(red, green, blue);
}

void SLAMDraw::PlotTriangulation(double x1, double y1, double yaw1, double x2, double y2, double yaw2, Point p, Point p1, Point p2){
    //start with a blank canvas
    canvas = Mat(img_w, img_h, CV_8UC3, LAND_COLOR);
    
    circle(canvas, Scale(Point2f(x1, y1)), 4, CV_RGB(255,0,255), -1, 8, 0); //CV_RGB(139,69,19)
    
    circle(canvas, Scale(Point2f(x2, y2)), 4, CV_RGB(255,0,0), -1, 8, 0); //CV_RGB(139,69,19)
    
    circle(canvas, Scale(p), 2, GetLandmarkColor(17), -1, 8, 0); //CV_RGB(139,69,19)
    
    DrawSight(x1, y1, yaw1);
    DrawSight(x2, y2, yaw2);
    
    ProjectRay(x1, y1, yaw1, p1.x/704);
    ProjectRay(x2, y2, yaw2, p2.x/704);
    
    ShowImage((char *) "triangulation", canvas, true);
}

vector<double> SLAMDraw::GetEquationOfLineForTest(int test, double x, double y, double yaw, double field_of_view){
    /*test is either +1 (for top) or -1 (for bottom)*/
    if(test!=1 && test!=-1) {
        printf("The test condition must be +1 or -1\n");
        exit(-1);
    }
    
    Point2d boat(x, y);
    
    double SIGHT_LENGTH = 1000.0;
    double angle = yaw + test*field_of_view/2.0;
    
    Point2d line;
    line.x = x + SIGHT_LENGTH*cos(angle);
    line.y = y + SIGHT_LENGTH*sin(angle);
    
    return EquationOfLine(boat, line);
}

vector<double> SLAMDraw::EquationOfLine(Point2d p1, Point2d p2){
    /*equation of a line*/
    double m = (1.0*p1.y-p2.y)/(p1.x-p2.x);
    double b = p1.y - m*p1.x;
    vector<double> eol;
    eol.push_back(m);
    eol.push_back(b);
    return eol;
}

int SLAMDraw::LocationOfPointRelativeToLine(double m, double b, Point2d p){
    double y = m*p.x + b;
    if(y>p.y) {
        return POINT_IS_BELOW_LINE;
    } else if(y<p.y) {
        return POINT_IS_ABOVE_LINE;
    }
    return 0;
}

void SLAMDraw::AddArrow(double x, double y, double ax, double ay){
    Point p = Scale(Point2f(x, y));
    Point a = Scale(Point2f(ax, ay));
    int line_thickness = 1;
    arrowedLine(canvas, p, a, CV_RGB(0,0,0));//, 1, 8, 0, 0.1);
}

void SLAMDraw::AddShape(SLAMDraw::shape s, double x, double y, int r, int g, int b){
    Point2f p = Scale(Point2f(x, y));
    //TODO: possibly make a param argument, which contains these, and maybe the rgb colors.
    int rad = 5;
    int size = 4;
    int line_thickness = 3;
    
    switch(s){
        case shape::CIRCLE:
        {
            circle(canvas, p, rad, CV_RGB(r, g, b), -1, 8, 0);
            break;
        }
        case shape::SQUARE:
        {
            Point2f sq(size, size);
            rectangle(canvas, p-sq, p+sq, CV_RGB(r, g, b), -1, 8, 0);
            break;
        }
        case shape::X:
        {
            line(canvas, Point2d(p.x-size,p.y-size), Point2d(p.x+size,p.y+size), CV_RGB(r, g, b), line_thickness, 8, 0);
            line(canvas, Point2d(p.x-size,p.y+size), Point2d(p.x+size,p.y-size), CV_RGB(r, g, b), line_thickness, 8, 0);
            break;
        }
        default:
            break;
    }
}

void SLAMDraw::PutText(double x, double y, string text) {
    Point2f p = Scale(Point2f(x, y));
    putText(canvas, text, p, FONT_HERSHEY_PLAIN, 1, CV_RGB(0,0,0));
}
