/*
 * MultiSessionIterativeSmoothingAndRefinement.hpp
 *
 *  Created on: Oct 11, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_MULTISESSIONITERATIVESMOOTHINGANDREFINEMENT_HPP_
#define SRC_RFLOWOPTIMIZATION_MULTISESSIONITERATIVESMOOTHINGANDREFINEMENT_HPP_


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

#include "MultiSessionOptimization.hpp"

/*GOAL:
 To retain the accuracy of MSO, but run much faster and use much less memory as the number of surveys is increased.
 */
class MultiSessionIterativeSmoothingAndRefinement: public MultiSessionOptimization {
protected:
    const int ITER_STALE = 10;
    const int MAX_CASC_ITERS = 500;
    
    std::vector<std::vector<std::vector<double> > > bestlandmarks;
    std::vector<std::vector<std::vector<double> > > bestposes;
    std::vector<std::vector<double>> bestlpd_rerror;
    std::vector<std::vector<std::vector<double> > > landmarks;
    std::vector<std::vector<std::vector<double> > > poses;
    std::vector<ParseOptimizationResults> originPOR;
    
    std::vector<std::vector<double>> isc_rerror;
    std::vector<std::vector<std::vector<int> > > forwardLMap;
    
    void BuildLandmarkSet();
    bool DifferentPoses(std::vector<double>& a, std::vector<double>& b);
    void ConstructFactorGraph(int survey);
    int BinarySearchLandmark(std::vector<LandmarkTrack>& landmarks, int lkey);
    std::vector<gtsam::Point2> CoordsFromCachedSetAndIds(std::vector<LandmarkTrack>& landmarks, std::vector<int>& ids, int ckey);
    
    int GetIndexOfFirstPoint(const std::vector<std::vector<double> >& landmarks, int id);
    std::vector<gtsam::Point3> GetSubsetOf3DPoints(const std::vector<std::vector<double> >& landmarks, const std::vector<int>& ids_subset);
    
    double EvaluateGaussian(int x, double u, double dev);
    std::vector<bool> LPDInlierTest(int s, int l, double LPD_RERROR_THRESHOLD, std::vector<double>& error);
    int EvaluateLPD(std::vector<std::vector<std::vector<double> > >& poseresult, std::vector<std::vector<std::vector<double> > >& landmarks, int s, int j, std::vector<EvaluateRFlow*> perf, std::vector<std::unordered_map<int, double> >& errorcache);
//    void AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, std::vector<gtsam::Point2>& p2d1, std::vector<int>& pids, std::vector<double>& inliers);
    void AddLocalization(int sISC, int sTIME, int survey, int surveyTIME, gtsam::Pose3 offset);
    void AddDirectionalLocalization(int s, int j, int d);
    void AddLocalizations(int survey);
    void SaveResults();
    
    double UpdateErrorIterative();
//    void SaveResults();
    int GetSurveyWithHighestISCError();
    
    enum Direction {
        BACKWARD = 0,
        FORWARD = 1
    };
    
public:
    std::string _origin_dir;
    
    MultiSessionIterativeSmoothingAndRefinement(Camera& cam, std::string results_dir, std::string pftbase, std::string date = "");
    
    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_MULTISESSIONITERATIVESMOOTHINGANDREFINEMENT_HPP_ */
