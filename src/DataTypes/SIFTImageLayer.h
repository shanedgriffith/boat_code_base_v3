//
//  SIFTImageLayer.h
//  SIFTFlow
//
//  Created by Shane Griffith on 9/2/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SIFTFlow_SIFTImageLayer_h
#define SIFTFlow_SIFTImageLayer_h

#include "ImageLayer.h"
#include <ImageAlignment/DREAMFlow/ImageOperations.h>

class SIFTImageLayer: public ImageLayer{
private:
public:
    double alignment_energy=0;
    double alignment_energy_desc=0;
    Mat f;
    Mat nrg;
    
    void Init(){
        SetEmptyFlow();
        SetEmptyEnergy();
    }
    
    SIFTImageLayer(int rows, int cols):
    ImageLayer(rows, cols){
    	Init();
    }

    SIFTImageLayer(vector<cv::Mat> ims):
    ImageLayer(ims)
    {
        Init();
    }
    
    void SetEmptyFlow(){
        f = Mat(rows, cols, CV_64FC2, cv::Scalar::all(0));
    }
    
    void SetEmptyEnergy(){
        nrg = Mat(rows, cols, CV_64FC1, cv::Scalar::all(-1));
    }
    
    void CopyResult(SIFTImageLayer& sil){
    	if(sil.rows != rows || sil.cols != cols){
    		cout << "Incompatible sift image layer size." << endl;
    		exit(-1);
    	}
    	memcpy(f.data, sil.f.data, f.total()*f.channels()*sizeof(double));
        memcpy(nrg.data, sil.nrg.data, nrg.total() * sizeof(double));
        alignment_energy = sil.alignment_energy;
        alignment_energy_desc = sil.alignment_energy_desc;
        consistency = sil.consistency;
        verified_ratio = sil.verified_ratio;
    }

    void SetFlow(double * v){
        memcpy(f.data, v, f.total()*f.channels()*sizeof(double));
    }
    
    void SetEnergy(double * energy_image){
        memcpy(nrg.data, energy_image, nrg.total() * sizeof(double));
    }
    
    void SetAlignmentEnergy(double e, double e_d=-1){
        alignment_energy = e;
        alignment_energy_desc = e_d;
    }
    
    double GetAlignmentEnergy(){
        return alignment_energy;
    }
    
    Mat HalveImage(Mat& im, int g_hsize=5, double g_sigma=0.67){
        /*
         Note the parameters used to halve the image are different from those used for an RGB image.
         -SIFT features are apparently quite sensitive to gaussian blur.
         */
        //anything over 16 channels is treated as an image of descriptors.
        if(im.channels() < 16) return ImageLayer::HalveImage(im);
        return ImageOperations::HalveImage(im, g_hsize, g_sigma);
    }
    
    SIFTImageLayer HalfSize(){
    	vector<cv::Mat> halves;
    	for(int i=0; i<im.size(); i++){
    		halves.push_back(HalveImage(im[i]));
    	}
    	return SIFTImageLayer(halves);
    }
    
    Mat DoubleFlow(){
        Mat doubled_flow;
        ImageOperations::DoubleImage(f, doubled_flow);
        return doubled_flow.mul(2.0);
    }
    
    vector<unsigned char *> ImageRefs(int idx1){
    	//!forward flips even and odd indexed images; e.g., the image and the mask order are flipped.
        int idx2 = (idx1+1)%2;
    	 vector<unsigned char *> refs = {im[idx1].data, im[idx2].data};
    	 return refs;
    }

    AlignmentResult GetAlignmentResult(int times_extrapolated=0){
        
        Mat flow =f;
        Mat energy = nrg;
        double ares = alignment_energy;
        double e_d = alignment_energy_desc;
        
        for(int i=0; i<times_extrapolated; i++)
        {
            Mat doubled;
            ImageOperations::DoubleImage(flow, doubled);
            flow = doubled.mul(2.0);
            
            ImageOperations::DoubleImage(energy, energy);
            
            ares = ares*4;
            e_d = e_d*4;
        }
        
        AlignmentResult res(flow.rows, flow.cols);
        res.SetEnergy((double *) energy.data);
        res.SetFlow((double *) flow.data);
        res.SetAlignmentEnergy(ares, e_d);
        res.SetConsistency(consistency);
        res.SetVerifiedRatio(verified_ratio);
        
        return res;
    }

};

#endif
