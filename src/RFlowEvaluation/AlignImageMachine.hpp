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
#include <DataTypes/AlignmentResult.h>

class AlignImageMachine: public AlignmentMachine {
private:
    
    ParseFeatureTrackFile LoadFTF(int survey, int time);
    AlignmentResult RunSFlowWithRF(std::vector<ReprojectionFlow*> rf, std::string image1, std::string image2);
    
    std::vector<Map*> maps;
    std::vector<ParseOptimizationResults*> por;
    std::vector<std::string> dates;
    int poseloc0;
    
    Camera& _cam;
public:
    std::string _pftbase, _query_loc;
    
    AlignImageMachine(Camera& cam):
        _cam(cam) {}
    void RunRFlow();
    
    void SetDirs(std::string pftbase, std::string query_loc, std::string results_dir);
    void Setup(int ploc0);
    void Reset();
    void * Run();
    void LogResults();

    void SetPOR(std::vector<ParseOptimizationResults*> pores){por=pores;}
    void SetDates(std::vector<std::string> d){dates=d;}
    void SetMaps(std::vector<Map*> m){maps = m;}
};

#endif /* SRC_RFLOWEVALUATION_ALIGNIMAGEMACHINE_HPP_ */
