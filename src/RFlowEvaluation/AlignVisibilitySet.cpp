/*
 * AlignVisibilitySet.cpp
 *
 *  Created on: Feb 28, 2017
 *      Author: shane
 */


#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <DataTypes/Map.hpp>
#include <FileParsing/ParseVisibilityFile.h>

#include "AlignVisibilitySet.hpp"

void AlignVisibilitySet::Visibility() {
    ParseVisibilityFile vis(_visibility_dir, _date1, _date2);
    
    std::string saveloc = _results_dir + _date1 + "_to_" + _date2 + "/";
    mkdir(saveloc.c_str(), (mode_t) (S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH));
    
    for(int i=0; i<vis.boat1.size(); i++) {
        std::cout << "Iteration: " << i << std::endl;
        
        //spawn a job to handle the alignment.
        int tidx = man.GetOpenMachine();
        ws[tidx]->Setup(vis.boat1[i]);
        ws[tidx]->SetMaps({&maps[0], &maps[1]});
        ws[tidx]->SetDates({_date1, _date2});
        ws[tidx]->SetPOR({&por[0], &por[1]});
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    
    std::cout << "Finished aligning the visibility set for " << _date1 << " to " << _date2 <<". Num images: " << vis.boat1.size() << std::endl;
}


void AlignVisibilitySet::VisualizeAllLabelsInOneMap(){
    
}
