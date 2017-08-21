/*
 * Anchors.hpp
 *
 *  Created on: Aug 10, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_ANCHORS_HPP_
#define SRC_RFLOWOPTIMIZATION_ANCHORS_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <FileParsing/ParseOptimizationResults.h>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/Camera.hpp>
#include <gtsam/geometry/Pose3.h>

class Anchors: public FileParsing {
private:
    double avgbadthreshold = 15;
    static const std::string _anchorsname;

    Camera& cam;
public:
    std::string _filename;
    std::vector<std::vector<double> > anchors;
    std::vector<int> sections;
    int last;

    Anchors(Camera& cam, std::string base, std::string date);
    
    Anchors(Camera& cam, ParseOptimizationResults& POR, int nanchors, int nposes);

    void ModifyAnchors(const std::vector<std::vector<double> >& landmarks, std::vector<double>& rerrors, ParseOptimizationResults& POR, std::string _pftset);
    void MergeAnchors(ParseOptimizationResults& POR, std::string _pftset, std::vector<bool>& split, const std::vector<std::vector<double> >& landmarks);
    std::vector<bool> SplitAnchors(const std::vector<std::vector<double> >& landmarks, std::vector<double>& rerrors, ParseOptimizationResults& POR, std::string _pftset);
    
    
    void WriteAnchors();
    void LoadAnchors();
    int NumAnchors();
    std::vector<double> ShiftPose(int s, gtsam::Pose3& gtp);
    std::vector<double> ShiftPose(int s, std::vector<double>& p);
    std::vector<std::vector<double> > GetShiftedPoses(std::vector<std::vector<double> >& poses);
    void UpdateAnchors(std::vector<std::vector<double> >& updated);
    int PoseIdxToAnchorIdx(int pidx);
    bool IsTransition(int t);
    gtsam::Pose3 GetAnchorAsPose(int idx);
    void Print();
};



#endif /* SRC_RFLOWOPTIMIZATION_ANCHORS_HPP_ */
