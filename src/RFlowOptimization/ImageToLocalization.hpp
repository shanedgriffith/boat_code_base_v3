/*
 * ImageToLocalization.hpp
 *
 *  Created on: Feb 22, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_IMAGETOLOCALIZATION_HPP_
#define SRC_RFLOWOPTIMIZATION_IMAGETOLOCALIZATION_HPP_


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <ImageAlignment/FlowFrameworks/AlignmentMachine.h>
#include <DataTypes/Camera.hpp>
#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <FileParsing/ParseOptimizationResults.h>

#include "LocalizedPoseData.hpp"

class ImageToLocalization : public AlignmentMachine{
private:
    double PERCENT_DENSE_CORRESPONDENCES = 0.4;
    
    gtsam::Pose3 CameraPose(std::vector<double>& p);
    
    CvScalar GetLandmarkColor(int id);
    void DrawFlowPoints(cv::Mat& imageA, cv::Mat& imageB, std::vector<double> inliers, std::vector<gtsam::Point3>& p3B, std::vector<gtsam::Point2>& p2dB, gtsam::Pose3 posea, gtsam::Pose3 poseb);
    void DrawMatchPoints(cv::Mat& imageA, cv::Mat& imageB, std::vector<gtsam::Point2>& p0, std::vector<gtsam::Point2>& p1, std::vector<unsigned char>& inliers);
    
    AlignmentResult MatchToSet(std::string image1, std::string image2);
    int MapPoints(AlignmentResult& ar, std::vector<gtsam::Point2>& imagecoord, std::vector<int> ids, std::vector<gtsam::Point3>& p3d,
                  std::vector<gtsam::Point2>& p1, std::vector<gtsam::Point2>& p2, std::vector<gtsam::Point3>& subset3d, std::vector<int>& subsetids, bool forward);
    bool ApplyEpipolarConstraints(std::vector<gtsam::Point2>& p0, std::vector<gtsam::Point2>& p1, std::vector<unsigned char>& inliers);
    double RobustAlignmentConstraints(AlignmentResult& ar, ParseFeatureTrackFile& pftf0,  ParseFeatureTrackFile& pftf1);
    
    std::vector<std::vector<double> > poses;
    std::vector<ReprojectionFlow*> rf;
    std::vector<ParseOptimizationResults*> por;
    std::vector<std::string> dates;
    std::vector<int> sids;
    std::vector<int> portimes;
    bool hasRF = false;
    
    LocalizedPoseData lpd;
    LocalizedPoseData * _res;
    LocalizedPoseData * toverify;
    gtsam::Pose3 p1_tm1;
    
    //return vals
    double perc_dc = -1;
    double * _perc_dc;
    bool ver_result = false;
    bool * _verified;
    
    Camera& _cam;
public:
    std::string _results_dir;
    std::string _query_loc;
    std::string _pftbase;
    
    ImageToLocalization(Camera& cam, std::string results_dir, std::string query_loc, std::string pftbase):
    _cam(cam), _query_loc(query_loc), _pftbase(pftbase), _results_dir(results_dir) {Reset();}
    void SetLastLPD(LocalizedPoseData * last){toverify = last;}
    void SetPoses(std::vector<std::vector<double> > p){poses = p;}
    void SetRF(std::vector<ReprojectionFlow*> rflow){rf = rflow; hasRF = true;}
    void SetPOR(std::vector<ParseOptimizationResults*> pores){por=pores;}
    void SetDates(std::vector<std::string> d){dates=d;}
    void SetSurveyIDs(std::vector<int> s){sids=s;}
    void SetPORTimes(std::vector<int> ps){portimes = ps;}
    void SetVPose(gtsam::Pose3 p){p1_tm1 = p;}
    void Setup(LocalizedPoseData * res, double * perc_dc, bool * verified);
    
    void Reset();
    void * Run();
    void LogResults();
};




#endif /* SRC_RFLOWOPTIMIZATION_IMAGETOLOCALIZATION_HPP_ */
