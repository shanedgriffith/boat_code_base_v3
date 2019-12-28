#pragma once

#include "FileParsing/ParseSurvey.h"
#include "BoatSurvey/ParseBoatSurvey.hpp"



class testAngularVelocity
{
private:
public:
    testAngularVelocity(){}
    
    static void
    compareAngularVelocityToTraj();
    
    static void
    testReprojectionWithYawDifference();
};
