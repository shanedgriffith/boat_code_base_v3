/*
 * ImageRetrieval.hpp
 *
 *  Created on: Aug 19, 2016
 *      Author: shane
 */

#ifndef SRC_IMAGEALIGNMENT_DREAMFLOW_IMAGERETRIEVAL_HPP_
#define SRC_IMAGEALIGNMENT_DREAMFLOW_IMAGERETRIEVAL_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <DataTypes/Camera.hpp>
#include <ImageAlignment/FlowFrameworks/MachineManager.h>
#include <ImageAlignment/DREAMFlow/IRMachine.hpp>

class ImageRetrieval {
private:
    bool debug = false;
    double POSE_RANGE_THRESHOLD = 3.0;
	double pose_distance_threshold = 5.0; //meters
	double pose_angle_threshold = 20.0; //degrees.
    
	int CAM_SKIP = 0;
	double ALIGN_ENERGY_THRESHOLD = 1120000; //energy of the low-res alignment
	double ALIGN_CONSISTENCY_THRESHOLD = 0.95; //consistency of forward-backward alignment.
	double DELTA = 70000; //for image retrieval search: a jump in alignment energy by this much indicates the images no longer capture the same scene.
    
	double verification_threshold = 0.4;
    
	void InitializeMachine(int nthreads);
    
    bool DistanceCriterion(std::vector<double>& pose1, std::vector<double>& pose2);
	std::vector<int> IdentifyNeighborPoses(std::vector<std::vector<double> >& poses, std::vector<int>& imageno, std::vector<double>& p2);
	std::vector<double> DirectionalSearch(std::string base, std::vector<int>& imageno, std::string image2, std::vector<int>& neighbor_poses);
	std::vector<double> NaiveSearch(std::string base, std::vector<int>& imageno, std::string image2, std::vector<int>& neighbor_poses);
    
    int GetMin(std::vector<double>& res);
    int GetMax(std::vector<double>& ver);
    bool HaveVerified(int i, std::vector<double>& ver);
    bool ContinueInDirection(int off, int direction, std::vector<double>& res);
    bool ArrayMatchesSignal010(std::vector<double>& res);
    std::vector<double> MultiThreadedSearch(std::string base, std::vector<int>& imageno, std::string image2, std::vector<int>& neighbor_poses);
    
    MachineManager man;
    std::vector<IRMachine*> ws;
    Camera& _cam;
public:
    ImageRetrieval(Camera& cam, int nthreads=1): _cam(cam) {
        InitializeMachine(nthreads);
    }

    ~ImageRetrieval(){
        man.WaitForMachine(true);
        for(int i=0; i<ws.size(); i++){
            delete(ws[i]);
        }
    }

	void SetCamSkip(int val){CAM_SKIP=val;}//use 0 for optimized surveys, use 5 or 10 or so for unoptimized surveys in which the images overlap a lot.
	int IdentifyClosestPose(std::string base, std::vector<std::vector<double> >& poses, std::vector<int>& imageno, std::vector<double> pose2, std::string image2, double * ae = NULL);
	std::vector<double> ParallelIRWithKnownInput(std::vector<std::string> refimages, std::string image2);
};



#endif /* SRC_IMAGEALIGNMENT_DREAMFLOW_IMAGERETRIEVAL_HPP_ */
