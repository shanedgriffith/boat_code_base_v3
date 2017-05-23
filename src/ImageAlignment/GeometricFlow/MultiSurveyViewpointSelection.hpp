//
//  MultiSurveyViewpointSelection.hpp
//  boat_code_base
//
//  Created by Shane Griffith on 1/23/17.
//  Copyright © 2017 shane. All rights reserved.
//

#ifndef MultiSurveyViewpointSelection_hpp
#define MultiSurveyViewpointSelection_hpp

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
    vector<ViewpointMachine*> ws;
public:
    MultiSurveyViewpointSelection(int nthreads=8){
        if(nthreads > 1) {
            for(int i=0; i<nthreads; i++){
                ws.push_back(new ViewpointMachine());
                man.AddMachine(ws[i]);
            }
        }
    }
    
    std::vector<std::vector<double> > TopKViewpoints(std::vector<ReprojectionFlow*>& rf, std::vector<std::vector<std::vector<double> > *>& poselists,
                                                       std::vector<string>& dates, std::vector<double>& pref, int k=8);

    bool WithinThreeMonths(string date1, string date2);
    ~MultiSurveyViewpointSelection(){
        man.WaitForMachine(true);
        for(int i=0; i<ws.size(); i++){
            delete(ws[i]);
        }
    }
};




#endif /* MultiSurveyViewpointSelection_hpp */
