//
//  ImageLayer.h
//  SIFTFlow
//
//  Created by Shane Griffith on 9/2/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __SIFTFlow__ImageLayer__
#define __SIFTFlow__ImageLayer__

#include <stdio.h>
#include <ImageAlignment/DREAMFlow/ImageOperations.h>

class ImageLayer
{
public:
    double consistency = 0;
    double verified_ratio = 0;
    int rows, cols, nchan;
    vector<cv::Mat> im;
    int layer_number=0;
    
    void SetLayerNumber(int no){
        layer_number = no;
    }
    
    ImageLayer(int height, int width): rows(height), cols(width){}

    ImageLayer(vector<cv::Mat> ims): im(ims), rows(ims[0].rows), cols(ims[0].cols), nchan(ims[0].channels()) {}
    
    int height(){
        return rows;
    }
    
    int width(){
        return cols;
    }
    
    int channels(){
    	return nchan;
    }

    Mat HalveImage(Mat& im, int g_hsize=7, double g_sigma=1.0){
        //note: an adaptive gaussian based on the image size doesn't seem to help.
        return ImageOperations::HalveImage(im, g_hsize, g_sigma);
    }
    
    ImageLayer HalfSize(){
    	vector<cv::Mat> halves;
    	for(int i=0; i<im.size(); i++){
    		halves.push_back(HalveImage(im[i]));
    	}
    	return ImageLayer(halves);
    }
};

#endif /* defined(__SIFTFlow__ImageLayer__) */
