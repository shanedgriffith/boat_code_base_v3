//
//  ViewpointMachine.hpp
//  boat_code_base
//
//  Created by Shane Griffith on 1/23/17.
//  Copyright © 2017 shane. All rights reserved.
//

#ifndef SRC_GEOMETRICFLOW_VIEWPOINTMACHINE_HPP_
#define SRC_GEOMETRICFLOW_VIEWPOINTMACHINE_HPP_

#include <stdio.h>
#include "ReprojectionFlow.hpp"
#include <ImageAlignment/FlowFrameworks/AlignmentMachine.h>

class ViewpointMachine: public AlignmentMachine{
private:
    ReprojectionFlow* _rf;
    std::vector<std::vector<double> >* _plist;
    std::vector<double> _pref;
    double g;
    int pidx;
    double * _gres;
    int * _pidxres;
public:
    ViewpointMachine(){}
    void Setup(ReprojectionFlow* rf, std::vector<std::vector<double> >* plist, std::vector<double> pref, double * gres, int * pidxres);
    void Reset();
    void * Run();
    void LogResults();
//    void Terminate();
};



#endif /* SRC_GEOMETRICFLOW_VIEWPOINTMACHINE_HPP_ */
