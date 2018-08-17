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

    Camera& _cam;
    std::vector<std::string>& _dates;
    
    MachineManager man;
    std::vector<AlignImageMachine*> ws;
public:
    std::string _pftbase, _query_loc, _savebase, _visibility_dir, _maps_dir;
    
    ForBMVCFigure(Camera& cam, std::vector<std::string>& dates, std::string pftbase,
                  std::string query_loc, std::string results_dir, int nthreads=12):
    _cam(cam), _dates(dates), _pftbase(pftbase), _query_loc(query_loc), _savebase(results_dir + "aligned_images/"),
    _maps_dir(results_dir + "maps/")
    {
        for(int i=0; i<nthreads; i++){
            ws.push_back(new AlignImageMachine(cam));
            ws[i]->SetDirs(_pftbase, _query_loc, results_dir);
            man.AddMachine(ws[i]);
        }
    }
    
    void MakeTimelapse(std::string ref_date, int num, bool viewpoint_variance = false);
    
};
















#endif /* SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_FORBMVCFIGURE_HPP_ */
