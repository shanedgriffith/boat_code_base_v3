/*
 * SFlowDREAM2RF.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: shane
 */

#include "SFlowDREAM2RF.hpp"

using namespace std;

void SFlowDREAM2RF::SetHeterogeneousHypothesisSpace(BPFlow& bpflow, SIFTImageLayer& il, int imset){
//    int HYP_SPACE_PADDING = 2;
//    
    std::vector<double> mcur = rflows[imset]->GetConstraintBounds(0);
    if(mcur.size()<6){
        std::cout << "SFlowDREAM2RF::SetHeterogeneousHypothesisSpace() Error. Not using hxhy."<<std::endl;
        return;
    }
    
    bpflow.CreateHypSpace(il.width(), il.height());
    int winSizex = max(0.0, max((double) abs(mcur[1]-mcur[2]), (double) abs(mcur[2]-mcur[0])));
    int winSizey = max(0.0, max((double) abs(mcur[4]-mcur[5]), (double) abs(mcur[5]-mcur[3])));

    for(int i=0; i<il.height(); i++){
        for(int j=0; j<il.width(); j++){
            bpflow.SetHypSpaces(winSizex, winSizey, j, i);
        }
    }
}

void SFlowDREAM2RF::ApplyRFlowConstraints(BPFlow& bpflow, SIFTImageLayer& il, double scale, int imset){
	bpflow.UseRConstraints();
	int nc = rflows[imset]->GetNumberOfConstraints(0);
	rflows[imset]->OutputSize(il.width(), il.height());
	for(int i=0; i<nc; i++){
		std::vector<double> c = rflows[imset]->GetConstraint(0, i);
		bpflow.AddConstraint(round(c[0]), round(c[1]), c[2], c[3], c[4]/scale);//I might want to increase the error on c[4], e.g., double or quadruple it.
	}
}

void SFlowDREAM2RF::AdaptiveOffsetConstraint(int height, int width){
	/*Use RF to 1) center the hypothesis space around the correct alignment; and 2) size the hypothesis space.
	 *
	 * Generate an offset image that's in the middle of min and max of the constraints, then set h
	 * to be HYP_SPACE_PADDING outside of this range.
	 * */
    HYP_SPACE_PADDING = 0;
    int hmax = 0;
	for(int imset=0; imset<2; imset++){
        rflows[imset]->OutputSize(width, height);
		std::vector<double> mcur = rflows[imset]->GetConstraintBounds(0);
		offsets.push_back(cv::Mat(height, width, CV_64FC2, cv::Scalar(mcur[2], mcur[5])));
        
        /*
        double * offset = (double *) offsets[imset].data;
        
        for(int i=0; i<height; i++){
            for(int j=0; j<width; j++){
                int idx = (i*width + j)*2;
                vector<double> flow_scaled = rflows[imset]->GetkNNConstraint(0, j, i, 1);
                if(flow_scaled.size() == 0) return;
                offset[idx] = (int) flow_scaled[0];
                offset[idx+1] = (int) flow_scaled[1];
            }
        }*/

		//size the hypothesis space.
		int hx = max((double) abs(mcur[1]-mcur[2]), (double) abs(mcur[2]-mcur[0]));
		int hy = max((double) abs(mcur[4]-mcur[5]), (double) abs(mcur[5]-mcur[3]));
		hmax = max(hmax, max(hx, hy));
		if(debug)
            cout << "Image Alignment " << imset<< " offset: ["<<mcur[2] << "," <<mcur[5] << "] bounds: [" << hx <<", " << hy << "] actual bound: " << hmax << " + " << HYP_SPACE_PADDING << endl;
	}
    
	//set the hypothesis space size
    topwsize = hmax + HYP_SPACE_PADDING;
}

void SFlowDREAM2RF::EpipolarConstraintsFromRF(){
	RFlowFME fme(_cam, rflows[0]->outw, rflows[0]->outh);
	fme.setSTD(epi_std);
	for(int j=0; j<2; j++){
		std::vector<cv::Point2f> orig_sparse = rflows[j]->GetRestrictedSetOrig(0);
		std::vector<cv::Point2f> mapped_sparse = rflows[j]->GetRestrictedSetMapped(0);
		bool success = fme.IdentifyHypothesisSpace(hypspace[j], orig_sparse, mapped_sparse, offsets[j], topwsize);
		if(!success){
			std::cout << "SFlowDREAM2RF::EpipolarConstraintsFromRF() Error: Failed to generate epipolar constraints from RF points. " << std::endl;//Unexpected behavior.
			std::cout << "  ..continuing without Reprojection Flow (no epipolar constraints from them, no initial hypothesis space offset, no direct constraints)."<<std::endl;
			reprojection_flow = false;
		}
	}
}

bool SFlowDREAM2RF::SetReprojectionFlow(std::vector<ReprojectionFlow*> rfs){
	if(rfs.size()<2){
		return false;
	}
	reprojection_flow = true;
	rflows = rfs;
	rf = rflows[0];//needed for some checks in the superclass.
	return true;
}

void SFlowDREAM2RF::Reset(){
	rflows = {};
	SFlowDREAM::Reset();
}
