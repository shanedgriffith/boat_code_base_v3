/*
 * RFlowFME.cpp
 *
 *  Created on: Sep 30, 2016
 *      Author: shane
 */


#include "RFlowFME.hpp"

using namespace std;


//set the standard deviation for gaussian distances.
void RFlowFME::setSTD(float std){
    stdp = std;
}


void RFlowFME::SetHypSpace(int hypspace){
    _h = hypspace;
}


double RFlowFME::EvaluateGaussian(int dline, double stdp){
    return 1.0/(stdp*pow(2*M_PI,0.5))*exp(-1*pow(dline, 2)/(2*pow(stdp,2)));
}


void RFlowFME::GetHypothesisSpaceOutlier(vector<double>& hypspace, cv::Point2f center, cv::Point2f offset, cv::Vec3f line){
    //here, center is what the pixel was mapped to. (to-be-warped image)
    //  the epipolar line is computed from the pixel it was mapped from. (reference image)

    //Equation of a line:
    //ax + by + c = 0;

    //point on the line?
    // -need to know the area around which the hypothesis is centered.
    // -if there are no correspondences... this won't be known.
	int winsize = 2*_h+1;
    for(int i=-_h; i<=_h; i++){
        for(int j=-_h; j<=_h; j++){
        	int idx = (i+_h)*winsize + (j+_h);
            int x = center.x+j-offset.x;
            int y = center.y+i-offset.y;

            double dist_to_line = abs(x*line[0] + y*line[1] + line[2])/pow(pow(line[0],2)+pow(line[1],2),0.5);
            hypspace[idx] = EvaluateGaussian(dist_to_line, stdp);
            //hypspace[idx] = 255 * dist_to_line; //achieves lower alignment energy, but the quality is much worse.
        }
    }
}


double RFlowFME::EvaluateGaussian(int fx, int fy, double ux, double uy, double stdp){
    //this is valid if x and y are uncorrelated and have the same variance
    //it's equivalent to the Gaussian method above for the distance.
    return 1.0/(stdp*pow(2*M_PI,0.5))*exp(-1*(pow(fx-ux, 2) + pow(fy-uy,2))/(2*pow(stdp,2)));
}


void RFlowFME::GetHypothesisSpaceInlier(vector<double>& hypspace, double cx, double cy){
    //here, center is what the pixel was mapped to. (to-be-warped image)
    //  the line is computed from the pixel it was mapped from. (reference image)
    int winsize = 2*_h+1;
    for(int i=-_h; i<=_h; i++){
        for(int j=-_h; j<=_h; j++){
        	int idx = (i+_h)*winsize + (j+_h);
            hypspace[idx] = EvaluateGaussian(j, i, cx, cy, stdp);
            //hypspace[idx] = 255.0*pow(pow(j-cx,2.0) + pow(i-cy, 2.0), 0.5);
        }
    }
}


bool RFlowFME::ComputeHypothesisSpace(vector<vector<double> >& hypspace, vector<cv::Point2f>& mapped, vector<unsigned char>& labels){
    //Because the hypothesis space is centered around the mapped point, the epipolar constraints are defined in the hypothesis space around it.

    int count = 0;
    for(int i=0; i<mapped.size(); i++){
    	vector<cv::Point2f> pixel = {cv::Point2f(i%_width, i/_width)};
		vector<cv::Vec3f> line;
		cv::computeCorrespondEpilines(cv::Mat(pixel), 1, F, line);		//for a given pixel, compute the epiline in the other image.

        if(labels[i]==0) GetHypothesisSpaceOutlier(hypspace[i], pixel[0], mapped[i], line[0]); //the use of mapped here is correct; bpflow centers the hypothesis space around the flow of the previous layer
        else { GetHypothesisSpaceInlier(hypspace[i]); count++;}
    }
    if(debug) cout << "Ratio of inliers to outliers: "<<count<<"/"<<mapped.size()<<"="<<1.0*count/mapped.size() << endl;
    return true;
}


bool RFlowFME::ComputeHypothesisSpaceFirst(vector<vector<double> >& hypspace, vector<cv::Point2f>& orig_sparse, vector<cv::Point2f>& mapped_sparse, cv::Mat& flow, vector<unsigned char>& labels){
    /*
     * A special function is required if this is the first hypothesis space because the flow is set to zero.
     * */

	double * f = (double *) flow.data;
	//compute the hypothesis space of epipolar constraints only
	for(int i=0; i<_height*_width; i++){
		vector<cv::Point2f> pixel = {cv::Point2f(i%_width, i/_width)};
		vector<cv::Vec3f> line;
		cv::computeCorrespondEpilines(cv::Mat(pixel), 1, F, line);		//TODO: check!!!
		cv::Point2f off = cv::Point2f(0,0);
		if(flow.rows>0) off = cv::Point2f(f[i*2], f[i*2+1]);
		GetHypothesisSpaceOutlier(hypspace[i], pixel[0], off, line[0]); //add offset (of the pixel) here.
	}

	//add the known points
	for(int i=0; i<orig_sparse.size(); i++){
		if(labels[i]==0) continue;
		cv::Point2f off = cv::Point2f(0,0);
		if(flow.rows>0) {
			int idx = orig_sparse[i].y * _width + orig_sparse[i].x;
			off = cv::Point2f(f[idx*2], f[idx*2+1]);
		}
		double cx = mapped_sparse[i].x-orig_sparse[i].x - off.x; //check offset here
		double cy = mapped_sparse[i].y-orig_sparse[i].y - off.y; //check offset here
		int idx = round(orig_sparse[i].y)*_width + round(orig_sparse[i].x);
		GetHypothesisSpaceInlier(hypspace[idx], cx, cy);
	}
    return true;
}


void RFlowFME::TransformFlow(cv::Mat& flow, vector<cv::Point2f>& orig, vector<cv::Point2f>& mapped){
    double * f = (double *) flow.data;
    for(int i=0; i<flow.rows; i++){
        for(int j=0; j<flow.cols; j++){
            int idx = (i*flow.cols + j);
            orig[idx] = cv::Point2f(j,i);
            mapped[idx]= cv::Point2f(j+f[2*idx], i+f[2*idx+1]);
        }
    }
}


bool RFlowFME::IdentifyHypothesisSpace(vector<vector<double> >& hypspace, cv::Mat& flow, int h){
	/*Specify orig and mapped if the flow points are different from the constraint space, e.g., for the first iteration.*/
    _h = h;

    vector<cv::Point2f> orig(flow.rows*flow.cols, cv::Point2f());
    vector<cv::Point2f> mapped(flow.rows*flow.cols, cv::Point2f());
    vector<unsigned char> inliers;
    TransformFlow(flow, orig, mapped);

    //find the points that match the epipolar constraint.
    int ninliers = IdentifyInliersAndOutliers(_cam, orig, mapped, inliers);
    if(!AreInliersMeaningful(ninliers))return false;

    return ComputeHypothesisSpace(hypspace, mapped, inliers);
}


bool RFlowFME::IdentifyHypothesisSpace(vector<vector<double> >& hypspace, vector<cv::Point2f>& orig_sparse, vector<cv::Point2f>& mapped_sparse, cv::Mat& flow, int h){
	/*Specify orig and mapped if the flow points are different from the constraint space, e.g., for the first iteration.*/
	_h = h;

	//find the points that match the epipolar constraint.
    vector<unsigned char> inliers;
    int ninliers = IdentifyInliersAndOutliers(_cam, orig_sparse, mapped_sparse, inliers);
	if(!AreInliersMeaningful(ninliers))return false;

    return ComputeHypothesisSpaceFirst(hypspace, orig_sparse, mapped_sparse, flow, inliers);
}
