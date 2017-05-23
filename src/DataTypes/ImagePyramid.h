//
//  ImagePyramid.h
//  SIFTFlow
//
//  Created by Shane Griffith on 6/24/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SIFTFlow_ImagePyramid_h
#define SIFTFlow_ImagePyramid_h

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <DataTypes/AlignmentResult.h>
#include <ImageAlignment/DREAMFlow/ImageOperations.h>
#include "ImageLayer.h"
#include "SIFTImageLayer.h"


class ImagePyramid{
public:
    int _nlayers = 0;
    int start_layer = 1;
    int stop_layer = 1;
    int pyr_type = 0;
    int start_sift_layer = 1;
    std::vector<SIFTImageLayer> layers;
    enum pyramid_type{
        RGB,
        SIFT,
        BRIEF
    };
    
    void Stack(SIFTImageLayer il){
        layers.push_back(il);
        if(layers.size() < _nlayers) {
        	if(layers.size() == start_sift_layer){
				if(pyr_type == pyramid_type::SIFT) {
                    cv::Mat i1_64, i2_64;
					il.im[0].convertTo(i1_64, CV_64FC(il.im[0].channels()));
					il.im[1].convertTo(i2_64, CV_64FC(il.im[1].channels()));
					vector<cv::Mat> ims = {i1_64, i2_64};
					for(unsigned int i=2; i<il.im.size(); i++){
						ims.push_back(il.im[i]);
					}
					SIFTImageLayer change_types(ims);
					SIFTImageLayer half = change_types.HalfSize();

                    cv::Mat half1 = ImageOperations::imSIFT(half.im[0]);
                    cv::Mat half2 = ImageOperations::imSIFT(half.im[1]);
					ims = {half1, half2};
					for(unsigned int i=2; i<il.im.size(); i++){
						ims.push_back(half.im[i]);
					}
					SIFTImageLayer res(ims);
					Stack(res);
				} else if(pyr_type == pyramid_type::BRIEF) {
					//Note: With BRIEF, the SIFT Flow parameters have to be decreased by a factor of 30. (divide by 30).
					SIFTImageLayer half = il.HalfSize();

                    cv::Mat half1 = ImageOperations::imBRIEF(half.im[0]);
                    cv::Mat half2 = ImageOperations::imBRIEF(half.im[1]);
					std::vector<cv::Mat> ims = {half1, half2};
					for(unsigned int i=2; i<il.im.size(); i++){
						ims.push_back(il.im[i]);
					}
					SIFTImageLayer res(ims);
					Stack(res);
				}
        	} else {
                SIFTImageLayer half = il.HalfSize();
                Stack(half);
            }
        }
    }
    
    ImagePyramid(){}

    ImagePyramid(int nlayers, int _stop_layer, vector<cv::Mat> ims, int type=pyramid_type::SIFT)
    {
        _nlayers = nlayers;
        start_layer = nlayers - 1;
        stop_layer = _stop_layer;
        pyr_type = type;

        Stack(SIFTImageLayer(ims));
    }
    

    void SetStartLayer(int l){
        start_layer = l;
    }
    
    void SetStopLayer(int l){
        stop_layer = l;
    }
    
    int TopLayer(){
        return _nlayers - 1;
    }
    
    AlignmentResult GetAlignmentResult(int res_layer){
        /*This implementation is specific to using a SIFT Image pyramid with the top layer
         being the original RGB images.
         */
        
        if(pyr_type == pyramid_type::RGB){
            std::cout << "ImagePyramid: No alignment result. Indicate a descriptor."<<std::endl;
            exit(-1);
        }
        
        res_layer = std::min(res_layer, TopLayer());
        res_layer = std::max(res_layer, 0);
        
        //cannot simply use i1 and i2; they're the SIFT images.
        //only the base layer images are the input images.
        SIFTImageLayer half = layers[0];
        for(int i=0; i<res_layer; i++){
            half = half.HalfSize();
        }
        
        int available_layer = std::max(res_layer, stop_layer);
        
        SIFTImageLayer sil = layers[available_layer];
        AlignmentResult ar = sil.GetAlignmentResult(available_layer - res_layer);
        ar.SetReferenceImage(half.im[0].data);
        ar.SetSecondImage(half.im[1].data);
        if(layers[res_layer].im.size() > 2) ar.SetMask(layers[res_layer].im[2].data);
        ar.SetSIFT(layers[start_sift_layer].im[0], layers[start_sift_layer].im[1]);
        
        return ar;
    }
    
};

#endif
