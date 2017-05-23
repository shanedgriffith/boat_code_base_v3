//
//  ViewpointMachine.cpp
//  boat_code_base
//
//  Created by Shane Griffith on 1/23/17.
//  Copyright © 2017 shane. All rights reserved.
//

#include "ViewpointMachine.hpp"



void ViewpointMachine::Setup(ReprojectionFlow* rf, std::vector<std::vector<double> >* plist, std::vector<double> pref, double * gres, int * pidxres){
    thread_state = state::LOCKED;
    _rf = rf;
    _plist = plist;
    _pref = pref;
    _gres = gres;
    _pidxres = pidxres;
}

void ViewpointMachine::Reset(){
    thread_state = state::OPEN;
    _rf = NULL;
    _plist = NULL;
    _pref = {};
    _gres = NULL;
    _pidxres = NULL;
}

void * ViewpointMachine::Run(){
    thread_state = state::RUNNING;
    
    pidx = _rf->IdentifyClosestPose(*_plist, _pref, &g, false);
    
    thread_state = state::FINISHED;
    return (void *) NULL;
}

void ViewpointMachine::LogResults(){
    *_gres = g;
    *_pidxres = pidx;
}
