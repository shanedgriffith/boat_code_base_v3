//
//  FeatureMatchElimination.cpp
//  SIFTFlow
//
//  Created by Shane Griffith on 2/28/16.
//  Copyright © 2016 shane. All rights reserved.
//

#include "FeatureMatchElimination.hpp"

/*
void FeatureMatchElimination::drawEpipolarLines(std::vector<cv::Point2f>& points1, // keypoints 1
                                                std::vector<cv::Point2f>& points2, // keypoints 2)
                                                int whichImage) // image to compute epipolar lines in
{
    // Compute F matrix from 7 matches
    cv::Mat F = cv::findFundamentalMat(cv::Mat(points1), // points in object image
                                       cv::Mat(points2), // points in scene image
                                       cv::FM_7POINT); // 7-point method
    
    std::vector<cv::Vec3f> lines1;
    
    // Compute corresponding epipolar lines
    cv::computeCorrespondEpilines(cv::Mat(points1), // image points
                                  whichImage, // in image 1 (can also be 2)
                                  F, // F matrix
                                  lines1); // vector of epipolar lines
    
//    // for all epipolar lines
//    for (std::vector<cv::Vec3f>::const_iterator it = lines1.begin(); it!=lines1.end(); ++it)
//    {
//        // Draw the line between first and last column
//        cv::line(image_out,
//                 cv::Point(0,-(*it)[2]/(*it)[1]),
//                 cv::Point(image2.cols,-((*it)[2]+(*it)[0]*image2.cols)/(*it)[1]),
//                 cv::Scalar(255,255,255));
//    }
    
}*/

int FeatureMatchElimination::CountInliers(std::vector<unsigned char>& labels){
	int count=0;
	for(int i=0; i<labels.size(); i++){
		if(labels[i])count++;
	}
	return count;
}

bool FeatureMatchElimination::AreInliersMeaningful(int n){
    //Consider anything less than 8 as unusable since computing the the fundamental matrix
	//requires 8 points with the 8-point algorithm.
    return n >= 8;
}

int FeatureMatchElimination::IdentifyInliersAndOutliers(Camera& _cam,
                                                         std::vector<cv::Point2f>& points1,
                                                         std::vector<cv::Point2f>& points2,
                                                         std::vector<unsigned char>& inliers) {

    if (points1.size()<8 || points2.size()!=points1.size()){
        if(debug) std::cout << "Unable to measure the quality of the feature correspondences."<<std::endl;
        return false;
    }

    /*
     * Normalization is required. Otherwise the linear system can become extremely ill-conditioned, and thus the result is unstable.
     * See http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MOHR_TRIGGS/node50.html
     * */
    inliers=std::vector<unsigned char>(points1.size(), 0);
    p1 = std::vector<cv::Point2f>(points1.size());
    p2 = std::vector<cv::Point2f>(points2.size());
    cv::undistortPoints(points1, p1, _cam.IntrinsicMatrix(), _cam.Distortion());
    cv::undistortPoints(points2, p2, _cam.IntrinsicMatrix(), _cam.Distortion());


    //the use of both estimation methods to get F and E shouldn't be necessary, but
    //for some reason E = K.t() * F * K; doesn't work.
    //Could be due to the distortion coefficients. TODO
    double threepix = 3.0/_cam.w();//was statically set to 0.0005.
    F = cv::findFundamentalMat(cv::Mat(p1), cv::Mat(p2), cv::Mat(inliers), cv::FM_RANSAC, threepix, 0.999);

    int fininliers = CountInliers(inliers);
    if(!AreInliersMeaningful(fininliers)) {
    	F = cv::Mat();
    	E = cv::Mat();
    	return fininliers;
    }

    if(debug) std::cout<<"Number of inliers pre/post/fin: ("<<fininliers<<") of total: " << points1.size()<<std::endl;

    return fininliers;
}

void FeatureMatchElimination::CorrectMatches(std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2){
    //Use F to correct the matches.
	//i.e., inliers are typically off from the epipolar lines by a few pixels. This function modifies them to match the epipolar lines.
	//NOTE: currently unused because the results were worse. I may not have been using it correctly.
	cv::correctMatches(F, points1, points2, points1, points2);
}

std::vector<cv::Vec3f> FeatureMatchElimination::GetDenseEpipolarLines(std::vector<cv::Point2f>& orig, cv::Mat& F){
    std::vector<cv::Vec3f> lines;
    
    // Compute corresponding epipolar lines
    cv::computeCorrespondEpilines(cv::Mat(orig), // image points
                                  1, // in image 1 (can also be 2)
                                  F, // F matrix
                                  lines); // vector of epipolar lines
    return lines;
}

cv::Mat FeatureMatchElimination::GetFundamentalMat(){
	return F;
}

cv::Mat FeatureMatchElimination::GetEssentialMat(){
    std::vector<unsigned char> inliers(p1.size(), 0);
    E = findEssentialMat(cv::Mat(p1), cv::Mat(p2), 1.0, cv::Point2d(0, 0), cv::RANSAC, 0.999, 0.0001,  inliers);
    
    //enforce rank 2.
    cv::SVD decomp = cv::SVD(E);
    cv::Mat corr = cv::Mat::eye(3,3,CV_64FC1);
    corr.at<double>(2,2) = 0.0;
    E = decomp.u * corr * decomp.vt;
    return E;
}
