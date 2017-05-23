//
//  IRMachine.cpp
//  boat_code_base
//
//  Created by Shane Griffith on 1/6/17.
//  Copyright © 2017 shane. All rights reserved.
//

#include "IRMachine.hpp"

#include <ImageAlignment/DREAMFlow/SFlowDREAM.hpp>
#include <DataTypes/AlignmentResult.h>


void IRMachine::Setup(std::string im1, std::string im2, double * res, double * ver){
    thread_state = state::LOCKED;
    _im1 = im1;
    _im2 = im2;
    _res = res;
    if(ver != NULL) _ver = ver;
}

void IRMachine::Reset(){
    thread_state = state::OPEN;
    _im1 = "";
    _im2 = "";
    _res = NULL;
    _ver = NULL;
}

void * IRMachine::Run() {
    thread_state = state::RUNNING;
    SFlowDREAM sf(_cam);
    sf.SetDryRun();
    sf.SetVerifyAlignment();
    sf.ConstructImagePyramid(_im1, _im2);
    sf.AlignImages();
    AlignmentResult ar = sf.GetAlignmentResult();
    res_async = ar.alignment_energy;
    ver_async = ar.verified_ratio;
    thread_state = state::FINISHED;
    return (void *) NULL;
}

void IRMachine::LogResults(){
    if(_res!=NULL) *_res = res_async;
    if(_ver!=NULL) *_ver = ver_async;
}
