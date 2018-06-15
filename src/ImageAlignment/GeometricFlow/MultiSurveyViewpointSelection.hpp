//
//  MultiSurveyViewpointSelection.hpp
//  boat_code_base
//
//  Created by Shane Griffith on 1/23/17.
//  Copyright © 2017 shane. All rights reserved.
//

#ifndef SRC_GEOMETRICFLOW_MULTISURVEYVIEWPOINTSELECTION_HPP_
#define SRC_GEOMETRICFLOW_MULTISURVEYVIEWPOINTSELECTION_HPP_

#include <stdio.h>
#include "ViewpointMachine.hpp"
#include "ReprojectionFlow.hpp"
#include <ImageAlignment/FlowFrameworks/MachineManager.h>

class MultiSurveyViewpointSelection{
private:
    int SURVEY_TIME_COMPARE_UP_TO = 3;
    bool IsSameScene(double gstatistic);

    typedef struct {
        double g;
        int idx;
    }dubs;

    void SortPoints(std::vector<dubs>& both);

    MachineManager man;
    std::vector<ViewpointMachine*> ws;
public:
    MultiSurveyViewpointSelection(int nthreads=8){
        if(nthreads > 1) {
            for(int i=0; i<nthreads; i++){
                ws.push_back(new ViewpointMachine());
                man.AddMachine(ws[i]);
            }
        }
    }
    
    std::vector<std::vector<double> > TopViewpoints(std::vector<ReprojectionFlow*>& rf, std::vector<std::vector<std::vector<double> > *>& poselists,
                                                       std::vector<std::string>& dates, std::vector<double>& pref);

    bool WithinThreeMonths(std::string date1, std::string date2);
    ~MultiSurveyViewpointSelection(){
        man.WaitForMachine(true);
        for(int i=0; i<ws.size(); i++){
            delete(ws[i]);
        }
    }
};




#endif /* SRC_GEOMETRICFLOW_MULTISURVEYVIEWPOINTSELECTION_HPP_ */
