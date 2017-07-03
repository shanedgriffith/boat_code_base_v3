//
//  IRMachine.hpp
//  boat_code_base
//
//  Created by Shane Griffith on 1/6/17.
//  Copyright © 2017 shane. All rights reserved.
//

#ifndef SRC_IMAGEALIGNMENT_DREAMFLOW_IRMACHINE_HPP_
#define SRC_IMAGEALIGNMENT_DREAMFLOW_IRMACHINE_HPP_

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <DataTypes/Camera.hpp>
#include <ImageAlignment/FlowFrameworks/AlignmentMachine.h>

class IRMachine: public AlignmentMachine {
private:
    std::string _im1;
    std::string _im2;
    double res_async;
    double * _res;
    double ver_async;
    double * _ver;
    Camera _cam;
public:
    IRMachine():_cam(0,0,0,0,0,0){}
    void SetCamera(Camera cam){_cam = cam;}
    void Setup(std::string im1, std::string im2, double * res, double * ver = NULL);
    void Reset();
    void * Run();
    void LogResults();
};



#endif /* SRC_IMAGEALIGNMENT_DREAMFLOW_IRMACHINE_HPP_ */
