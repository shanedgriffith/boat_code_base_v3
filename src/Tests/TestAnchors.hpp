#pragma once

#include <FileParsing/ParseOptimizationResults.h>
#include <Optimization/SingleSession/FactorGraph.hpp>
#include <RFlowOptimization/LPDInterface.hpp>
#include "Optimization/MultiSession/RFlowFactorGraph.hpp"
#include "Optimization/SingleSession/GTSAMInterface.h"
#include <DataTypes/Camera.hpp>
#include <DataTypes/LandmarkTrack.h>


class TestAnchors
{
private:
    
    ParseOptimizationResults PORj_;
    ParseOptimizationResults PORk_;
    LPDInterface lpdi_;
    std::vector<LandmarkTrack> landmarks;
    
    int num_anchors;
    RFlowFactorGraph* rfFG;
    GTSAMInterface GTS;
    
    void ConstructFactorGraph();
    int SessionToNum(std::string session);
    void AddLocalizations();
    void SessionDates(std::vector<std::string>& dates);
    void BuildLandmarkSet();
    void EvaluateAnchors();
    
    void AddLandmarks(std::vector<LandmarkTrack>& ended_tracks);
    
    const Camera& _cam;
public:
    std::string _results_dir, _maps_dir, _pftbase;
    
    TestAnchors(const Camera& cam);
    
    void Run();
    
    
};



