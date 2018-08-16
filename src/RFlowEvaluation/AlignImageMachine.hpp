/*
 * AlignImageMachine.hpp
 *
 *  Created on: Feb 28, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWEVALUATION_ALIGNIMAGEMACHINE_HPP_
#define SRC_RFLOWEVALUATION_ALIGNIMAGEMACHINE_HPP_


#include <DataTypes/Map.hpp>
#include <DataTypes/Camera.hpp>

#include <FileParsing/ParseOptimizationResults.h>
#include <ImageAlignment/FlowFrameworks/AlignmentMachine.h>
#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <DataTypes/AlignmentResult.h>

class AlignImageMachine: public AlignmentMachine {
private:
    AlignmentResult RunSFlowWithRF(std::vector<ReprojectionFlow*> rf, std::string image1, std::string image2);
    
    std::vector<Map*> maps;
    std::vector<ParseOptimizationResults*> por;
    std::vector<std::string> dates;
    int poseloc0, poseloc1;
    std::string _saveloc;
    bool basic;
    std::string _image0, _image1;
    
    Camera& _cam;
public:
    std::string _pftbase, _query_loc, _results_dir;
    
    AlignImageMachine(Camera& cam):
        _cam(cam) {}
    void RunRFlow();
    void RunSFlow();
    
    void SetDirs(std::string pftbase, std::string query_loc, std::string results_dir);
    void Setup(int ploc0, std::string saveloc, int ploc1=-1);
    void SetImages(std::string i0, std::string i1, std::string savename);
    void Reset();
    void * Run();
    void LogResults();

    void SetPOR(std::vector<ParseOptimizationResults*> pores){por=pores;}
    void SetDates(std::vector<std::string> d){dates=d;}
    void SetMaps(std::vector<Map*> m){maps = m;}
};

#endif /* SRC_RFLOWEVALUATION_ALIGNIMAGEMACHINE_HPP_ */
