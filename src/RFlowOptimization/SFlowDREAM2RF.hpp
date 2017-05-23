/*
 * SFlowDREAM2RF.hpp
 *
 *  Created on: Dec 21, 2016
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_SFLOWDREAM2RF_HPP_
#define SRC_RFLOWOPTIMIZATION_SFLOWDREAM2RF_HPP_

#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <ImageAlignment/DREAMFlow/SFlowDREAM.hpp>

#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


/*
 * Uses a different format for the two Rflows, which enables maps from inconsistent sets to be aligned.
 * */
class SFlowDREAM2RF: public SFlowDREAM{
protected:
	virtual void ApplyRFlowConstraints(BPFlow& bpflow, SIFTImageLayer& il, double scale, int imset);
	virtual void AdaptiveOffsetConstraint(int height, int width);
	virtual void EpipolarConstraintsFromRF();
	std::vector<ReprojectionFlow*> rflows;
public:
	SFlowDREAM2RF(Camera& cam):SFlowDREAM(cam){
        HYP_SPACE_PADDING = 0;
    }
	bool SetReprojectionFlow(std::vector<ReprojectionFlow*> rfs);
	void Reset();
};



#endif /* SRC_RFLOWOPTIMIZATION_SFLOWDREAM2RF_HPP_ */
