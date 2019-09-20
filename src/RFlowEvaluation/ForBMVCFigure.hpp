/*
 * ForBMVCFigure.hpp
 *
 *  Created on: May 11, 2016
 *      Author: shane
 */

#ifndef SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_FORBMVCFIGURE_HPP_
#define SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_FORBMVCFIGURE_HPP_

#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <DataTypes/Map.hpp>
#include <ImageAlignment/FlowFrameworks/MachineManager.h>
#include "AlignImageMachine.hpp"

class ForBMVCFigure{
private:
    bool dry_run_;
    std::vector<std::string>& _dates;
    
    MachineManager man;
    std::vector<AlignImageMachine*> ws;
    
    const Camera& _cam;
public:
    std::string _pftbase, _query_loc, _savebase, _visibility_dir, _maps_dir;
    
    ForBMVCFigure(const Camera& cam, std::vector<std::string>& dates, std::string pftbase,
                  std::string query_loc, std::string results_dir, int nthreads=12):
    dry_run_(false), 
    _cam(cam), _dates(dates), _pftbase(pftbase), _query_loc(query_loc), _savebase(results_dir + "aligned_images/"),
    _maps_dir("/Volumes/Untitled/data/maps_only/maps_only_2014/") //results_dir + "maps/")
    {
        for(int i=0; i<nthreads; i++){
            ws.push_back(new AlignImageMachine(cam));
            ws[i]->SetDirs(_pftbase, _query_loc, results_dir);
            man.AddMachine(ws[i]);
        }
    }
    
    ~ForBMVCFigure(){
        man.WaitForMachine(true);
        for(int i=0; i<ws.size(); i++){
            delete(ws[i]);
        }
    }
    
    void setDryRun(){dry_run_ = true;}
    void MakeTimelapse(std::string ref_date, int num, bool viewpoint_variance = false);
    void MakeTimelapse(int d, int num, bool viewpoint_variance, std::vector<ParseOptimizationResults>& por, std::vector<Map>& maps);
    
};
















#endif /* SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_FORBMVCFIGURE_HPP_ */
