








#include "HierarchicalOccupancyGrid.hpp"


//intial bounding box.


/*
 
 c, w, h
 b leaf; //if there's only one point.
 int pidx; separate point list //even for non-leaves?
 vec<struc*> subs(4); // tl, tr, bl, br
 
 */

/*
 AddPoint
  -if there's already a point, set it to -1 and create a subtree.
  -if it is a non-leaf, recurse down the correct subtree.
 CreateST
  -to add a new point.
  -
 IsVisible
  -if the center, then recurse down all
  -//what about at the top, where maybe none of the corners or the center is visible, but a large portion of the box is part of the fov?
  -//the cam is inside.
  -//what about for a very very thing fov?
  -//seem to need a way to check for the fov specifically, rather than points.
  -//and the camera is specified using pose3; although, the x,y,theta can be extracted.
  -//if x,y,theta intersects a box, then the (why not just check all the boxes intersected by x,y,theta?)
   -//seem to need the fov; could use both fov lines to check each box. if above and below, then expand, or add all points, or check all points.
  -
 
 */



vector<double> SLAMDraw::GetEquationOfLineForTest(double x, double y, double yaw){
    Point2d boat(x, y);
    
    double SIGHT_LENGTH = 1000.0;
    
    Point2d line;
    line.x = x + SIGHT_LENGTH*cos(yaw);
    line.y = y + SIGHT_LENGTH*sin(yaw);
    
    return EquationOfLine(boat, line);
}

vector<double> SLAMDraw::EquationOfLine(Point2d p1, Point2d p2){
    /*equation of a line*/
    double m = (1.0*p1.y-p2.y)/(p1.x-p2.x);
    double b = p1.y - m*p1.x;
    return {m, b};
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

























































































