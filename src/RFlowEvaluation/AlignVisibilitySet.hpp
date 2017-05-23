/*
 * AlignVisibilitySet.hpp
 *
 *  Created on: Feb 28, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWEVALUATION_ALIGNVISIBILITYSET_HPP_
#define SRC_RFLOWEVALUATION_ALIGNVISIBILITYSET_HPP_

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include <DataTypes/Camera.hpp>
#include <FileParsing/ParseOptimizationResults.h>
#include <ImageAlignment/FlowFrameworks/MachineManager.h>
#include <DataTypes/Map.hpp>
#include "AlignImageMachine.hpp"

extern const std::string base;
extern const std::string optimized_datasets;
extern const std::string query_loc;
extern const std::string poses_loc;

class AlignVisibilitySet {
private:
    int _nthreads = 8;

    int GetIndexFromImageNo(int imageno, ParseOptimizationResults& por);

    MachineManager man;
    std::vector<AlignImageMachine*> ws;
    std::vector<ParseOptimizationResults> por;
    std::vector<Map> maps;

    Camera& _cam;
public:
    std::string _date1, _date2;

    //date1 is the reference survey.
    AlignVisibilitySet(Camera cam, std::string date1, std::string date2):
        _cam(cam), _date1(date1), _date2(date2) {

        maps.push_back(Map(base + "maps/"));
        maps.push_back(Map(base));// + "maps/"
        maps[0].LoadMap(_date1);
        maps[1].LoadMap(_date2);

        por.push_back(ParseOptimizationResults(base + "maps/" + _date1));
        por.push_back(ParseOptimizationResults(base + _date2));// + "maps/"

        for(int i=0; i<_nthreads; i++) {
            ws.push_back(new AlignImageMachine(_cam));
            man.AddMachine(ws[i]);
        }
    }

    ~AlignVisibilitySet() {
        for(int i=0; i<_nthreads; i++) {
            delete(ws[i]);
        }
    }


    void Visibility();
};



#endif /* SRC_RFLOWEVALUATION_ALIGNVISIBILITYSET_HPP_ */
