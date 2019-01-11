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
#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseOptimizationResults.h>
#include <ImageAlignment/FlowFrameworks/MachineManager.h>
#include <DataTypes/Map.hpp>
#include "AlignImageMachine.hpp"

class AlignVisibilitySet: public FileParsing {
private:
    int _nthreads = 8;
    
    std::vector<char> LoadLabelsFile(std::string filepath);
    
    MachineManager man;
    std::vector<AlignImageMachine*> ws;
    std::vector<ParseOptimizationResults> por;
    std::vector<Map> maps;
    
    const Camera& _cam;
public:
    std::string _date1, _date2;
    std::string _pftbase, _query_loc, _results_dir, _visibility_dir;
    
    //date1 is the reference survey.
    AlignVisibilitySet(const Camera& cam, std::string date1, std::string date2, std::string pftbase,
                       std::string query_loc, std::string results_dir, std::string visibility_dir):
        _cam(cam), _date1(date1), _date2(date2), _pftbase(pftbase), _query_loc(query_loc),
        _results_dir(results_dir), _visibility_dir(visibility_dir) {
        
        maps.push_back(Map(_results_dir + "maps/"));
        maps.push_back(Map(_results_dir + "maps/"));
        maps[0].LoadMap(_date1);
        maps[1].LoadMap(_date2);
        
        por.push_back(ParseOptimizationResults(_results_dir + "maps/", _date1));
        por.push_back(ParseOptimizationResults(_results_dir + "maps/", _date2));
        
        for(int i=0; i<_nthreads; i++) {
            ws.push_back(new AlignImageMachine(_cam));
            ws[i]->SetDirs(_pftbase, _query_loc, _results_dir);
            man.AddMachine(ws[i]);
        }
    }

    ~AlignVisibilitySet() {
        for(int i=0; i<_nthreads; i++) {
            delete(ws[i]);
        }
    }


    void Visibility(); //uses the pose/grid-based viewset and finds poses from the other sessions using covisibility
    
    void AlignMapBasedViewset(); //uses the map-based viewset and finds poses from the other sessions using covisibility

    void VisualizeAllLabelsInOneMap();
};



#endif /* SRC_RFLOWEVALUATION_ALIGNVISIBILITYSET_HPP_ */
