/*
* AlignICPImagePairs.hpp
*
*  Created on: Aug 15, 2018
*      Author: shane
*/

#ifndef SRC_RFLOWEVALUATION_ALIGNICPIMAGEPAIRS_HPP_
#define SRC_RFLOWEVALUATION_ALIGNICPIMAGEPAIRS_HPP_

#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <DataTypes/Camera.hpp>
#include <DataTypes/Map.hpp>
#include <ImageAlignment/FlowFrameworks/MachineManager.h>
#include "AlignImageMachine.hpp"
#include <FileParsing/ParseOptimizationResults.h>
#include <DataTypes/Map.hpp>

class AlignICPImagePairs{
private:
    
    int DateToIdx(int date);
    std::vector<std::vector<int> > ReadCSVFile(std::string file, int firstidx, int lastidx);
    
    std::vector<std::string> ParseLineAdv(char * line, std::string separator);
    std::string PaddedInt(int num);
    
    MachineManager man;
    std::vector<AlignImageMachine*> ws;
    
    Camera& _cam;
public:
    std::string _query_loc, _savebase, _maps_dir, _pftbase;
    std::vector<std::string> _dates;
    
    AlignICPImagePairs(Camera& cam, std::string query_loc, std::string results_dir, std::string pftbase, std::vector<std::string> dates, int nthreads=12):
    _cam(cam), _query_loc(query_loc), _pftbase(pftbase), _maps_dir(results_dir + "maps/"), _savebase(results_dir + "aligned_icp_image_pairs/"), _dates(dates)
    {
        for(int i=0; i<nthreads; i++){
            ws.push_back(new AlignImageMachine(cam));
            ws[i]->SetDirs(_pftbase, _query_loc, results_dir);
            man.AddMachine(ws[i]);
        }
    }
    
    ~AlignICPImagePairs(){
        man.WaitForMachine(true);
        for(int i=0; i<ws.size(); i++){
            delete(ws[i]);
        }
    }
    
    void AlignImagesRFlow(std::string file, int firstidx, int lastidx);
    void AlignImagesSFlow(std::string file, int firstidx, int lastidx);
    void AlignImagesWarped();
    void GetResults();
    void GetResultsTimelapse(std::string argnum, std::string argdate);
    
    void AlignTimelapsesRFlow(std::string dirnum);
    void AlignTimelapsesSFlow(std::string dirnum);
};
















#endif /* SRC_RFLOWEVALUATION_ALIGNICPIMAGEPAIRS_HPP_ */
