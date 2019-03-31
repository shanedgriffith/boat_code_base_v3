/*
 * SFlowDREAM.hpp
 *
 *  Created on: Apr 7, 2016
 *      Author: shane
 */

#ifndef SRC_IMAGEALIGNMENT_DREAMFLOW_SFLOWDREAM_HPP_
#define SRC_IMAGEALIGNMENT_DREAMFLOW_SFLOWDREAM_HPP_


#include <stdio.h>

#include <DataTypes/ImagePyramid.h>
#include <DataTypes/SIFTImageLayer.h>
#include <DataTypes/AlignmentResult.h>
#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <ImageAlignment/LiuFlow/BPFlow.h>
#include "RFlowFME.hpp"
#include <DataTypes/Camera.hpp>

/*An extended version of image alignment using SIFT Flow, which incorporates reprojection flow, epipolar constraints, two-cycle consistency, and alignment verification.*/
class SFlowDREAM{
protected:
	double  alpha=      1.0*255;                    //smoothness multiplier
	double  d=          40*255;			            //smoothness truncation
	double  gamma=      0.001*255;				    //regularization multiplier
	double  topwsize=   11;
	int     nIterations=100;

	//parameters for the image pyramid.
	double  nlayers=     5;
	int stop_layer =     1;
	int res_layer  =     0;

	bool debug = false;
	bool timed = false;

	ImagePyramid ip;

    int VERIFICATION_OFFSET_X = 3;
    int VERIFICATION_OFFSET_Y = 3;
	double verification_threshold = 0.4;
	double verifyalignment = false;
	double ranverification = false;
    double epi_std = 2.5; //0.014204545;// TODO: CHECK!
	bool twocycle = false;
	int two_cycle_iterations = 19; //this should be odd and >= 1
	int cycle_down_to_layer = 4;
	double _cycle_weight = 16;
	double consistency_threshold = 0.95;
	bool mandatory_consistency_condition = false;
	cv::Mat consistent_set;

	//probably should be true by default, since it adds little cost and performs better.
	bool epipolar = false;
	std::vector<std::vector<std::vector<double> > > hypspace;
	int HYP_SPACE_PADDING = 0;

	bool reprojection_flow = false;
	ReprojectionFlow* rf = NULL;
    std::vector<cv::Mat> offsets;

	double computation_time = 0.0;

    void RunBP(SIFTImageLayer& il, double gamma, int h, int nHierarchy, double scale, int imset=0, std::vector<std::vector<double> >* two_cycle_hypspace = NULL);
	void VerifiedBP(SIFTImageLayer& sil, double gamma, int h, int nHierarchy, double scale, int imset=0, std::vector<std::vector<double> >* two_cycle_hypspace = NULL);
	cv::Mat AddOffset(cv::Mat im, int x, int y);
	double ComputeVerificationRatio(SIFTImageLayer& sil, SIFTImageLayer& woff, int x, int y);
	void ComputeCycleConsistencyWeights(std::vector<std::vector<double> >& hypspace, SIFTImageLayer& il, int h);
	double MeasureConsistency(std::vector<SIFTImageLayer>& vec, int ref);
	bool ComputeEpipolarWeights(SIFTImageLayer& ref, SIFTImageLayer& comp, int imset, int h);
	void CyclicAlignment(SIFTImageLayer& sil, double g, int h, int nHierarchy);

    virtual void SetHeterogeneousHypothesisSpace(BPFlow& bpflow, SIFTImageLayer& il, int imset);
    virtual void ApplyRFlowConstraints(BPFlow& bpflow, SIFTImageLayer& il, double scale, int imset);
    virtual void AdaptiveOffsetConstraint(int height, int width);
	virtual void EpipolarConstraintsFromRF();

	const Camera& _cam;
public:
	int term_layer = 5;
	int cycle_iter = 0; //used for logging the number of iterations of two cycle consistency.

	SFlowDREAM(const Camera& cam): _cam(cam){}

	void Reset();

	AlignmentResult GetAlignmentResult();

	void AlignImages();

	void SetDryRun(bool b=true);
	void SetTopWsize(int t);

	void SetVerifyAlignment(bool set=true);
	void SetTwoCycleConsistency(bool set=true);
	void SetLastCycleConsistencyLayer(int l);
	void ContinueOnConsistency(bool set=true);
	void SetEpipolar(bool set=true);
	void SetMirrorRFConstraints(bool set=true);

	bool SetReprojectionFlow(ReprojectionFlow* rf);

    void ConstructImagePyramid(std::string fim1, std::string fim2);
	void ConstructImagePyramid(cv::Mat& im1, cv::Mat& im2);
	void SetImagePyramid(ImagePyramid& pyr);
};


#endif /* SRC_IMAGEALIGNMENT_DREAMFLOW_SFLOWDREAM_HPP_ */
