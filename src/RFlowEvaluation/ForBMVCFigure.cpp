/*
 * ForBMVCFigure.cpp
 *
 *  Created on: May 11, 2016
 *      Author: shane
 */

#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseSurvey.h>
#include <ImageAlignment/DREAMFlow/SFlowDREAM.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <RFlowOptimization/SFlowDREAM2RF.hpp>

#include "ForBMVCFigure.hpp"

using namespace std;

void ForBMVCFigure::MakeTimelapse(std::string ref_date, int num, bool viewpoint_variance){
    string savedir = _savebase + ref_date + "_" + to_string(num) + "/";
    
    FileParsing::MakeDir(savedir);
    FileParsing::MakeDir(savedir + "rf");
    FileParsing::MakeDir(savedir + "sf");
    FileParsing::MakeDir(savedir + "scene");
    FileParsing::MakeDir(savedir + "mappoints");
    FileParsing::MakeDir(savedir + "viewpoint");
    
    std::vector<ParseOptimizationResults> por;
    std::vector<Map> maps;
    int d1 = 0;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults(_maps_dir, _dates[i]));
        maps.push_back(Map(_maps_dir));
        maps[i].LoadMap(_dates[i]);
        if(ref_date == _dates[i]) d1 = i;
    }
    
    vector<int> offset = {-15, -10, -5, 0, 5, 10, 15};
    for(int off=0; off<offset.size(); off++){
        if(!viewpoint_variance && offset[off] != 0) continue;
        for(int i=0; i<_dates.size(); i++){
            if(ref_date == _dates[i]) continue;
            int tidx = man.GetOpenMachine();
            ws[tidx]->Setup(num, savedir);
            ws[tidx]->SetOffset(offset[off]);
            ws[tidx]->SetMaps({&maps[d1], &maps[i]});
            ws[tidx]->SetDates({ref_date, _dates[i]});
            ws[tidx]->SetPOR({&por[d1], &por[i]});
            man.RunMachine(tidx);
        }
    }
    man.WaitForMachine(true);
}
























