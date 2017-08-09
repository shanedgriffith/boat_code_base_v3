/*
 * Anchors.hpp
 *
 *  Created on: Mar 8, 2017
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


class Anchors: public FileParsing {
private:
    double avgbadthreshold = 15;
    static const string _anchorsname;
    bool ReadAnchors();
    void FindSections(Camera& _cam);

public:
    string _base;
    string _date;
    std::vector<std::vector<double>> anchor;
    std::vector<int> sections;

    Anchors(string base, string date, Camera& _cam): _base(base), _date(date) {
        if(!ReadAnchors()) FindSections(_cam);
    }

    int NumAnchors();
    bool HaveAnchors();
    bool IsValid(int section);
    std::vector<double> GetAnchoredPose(int i, vector<double> p);
    void SetAnchor(int section, std::vector<double> anchor);
    void WriteAnchors();
    int GetSection(int idx);
    std::vector<double> GetAnchor(int idx);
    gtsam::Pose3 GetAnchorAsPose(int idx);
    void Print();
};



#endif /* SRC_RFLOWOPTIMIZATION_ANCHORS_HPP_ */
