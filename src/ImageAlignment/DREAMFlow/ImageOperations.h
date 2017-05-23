//
//  ImageOperations.h
//  SIFTFlow
//
//  Created by Shane Griffith on 3/24/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __SIFTFlow__ImageOperations__
#define __SIFTFlow__ImageOperations__

#include <stdio.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <DataTypes/Camera.hpp>

class ImageOperations{
private:
    static void CheckType(cv::Mat& image, int type);
    
    static double ValueAtPixel(cv::Mat& image, int x, int y, int c);
    
    static void SetPixel(cv::Mat& image, int x, int y, int c, double val);
    
    static cv::Mat imfilter_hv(cv::Mat& source1, double *filter, int fsize);
    
    // function to get the derivative of the image
    static cv::Mat derivative(cv::Mat& source, bool horizontal);
    
    static void filtering(cv::Mat& source, cv::Mat& pDstImage,double* pfilter1D,int fsize, bool horizontal);
    
    // warps im2 according to the flow vectors in pVx and pVy
    static void warpImage(cv::Mat& image, cv::Mat& imgdata, cv::Mat& pV);
    
    static int EnforceRange(int x,const int MaxValue) {return std::min(std::max(x,0),MaxValue-1);};
    static inline void BilinearInterpolate(cv::Mat& image, double x, double y, unsigned char* result);
    
public:
    
    static cv::Mat imBRIEF(cv::Mat& imsrc);
    
    /*
     * imsift = 128 byte sift image.
     *
     *Note: The sift image is smaller than the input image by 4*cellSize rows and columns.
     * */
    static cv::Mat imSIFT(cv::Mat& imsrc, int cellSize=3, int stepSize=1, bool IsBoundaryIncluded=true, int nBins=8);
    
    static cv::Mat CreateWarpedImage(cv::Mat& image, cv::Mat& v);
    
    static cv::Mat Load(std::string path);
    static void Undistort(Camera& _cam, cv::Mat& img);
    static cv::Mat readUncompressedImage(std::string filename);
//    static Mat LoadFlow(string path);
    
    static void Save(cv::Mat im, std::string path);
    static bool writeUncompressedImage(cv::Mat& im, std::string path);
//    static void SaveFlow(Mat& im, string path);
    
    static cv::Mat HalveImage(cv::Mat& im, int g_hsize=7, double g_sigma=1.0);
    
    static void DoubleImage(cv::Mat& src, cv::Mat& dest);
    
    static void DoubleImage(cv::Mat& im);
    
};





#endif /* defined(__SIFTFlow__ImageOperations__) */
