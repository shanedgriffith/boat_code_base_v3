/*
 * AlignImageMachine.cpp
 *
 *  Created on: Feb 28, 2017
 *      Author: shane
 */


#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>

#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/ParseSurvey.h>
#include <RFlowOptimization/SFlowDREAM2RF.hpp>

#include "AlignImageMachine.hpp"


using namespace std;

void AlignImageMachine::Setup(int ploc0, std::string saveloc) {
    thread_state = state::LOCKED;
    poseloc0 = ploc0;
    _saveloc = saveloc;
}

void AlignImageMachine::SetDirs(std::string pftbase, std::string query_loc, std::string results_dir) {
    _pftbase = pftbase;
    _query_loc = query_loc;
    _results_dir = results_dir;
} 

void AlignImageMachine::Reset(){
    thread_state = state::OPEN;
    poseloc0 = -1;
    por = {};
    dates = {};
    maps = {};
    _saveloc = "";
}

AlignmentResult AlignImageMachine::RunSFlowWithRF(vector<ReprojectionFlow*> rf, string image1, string image2){
    SFlowDREAM2RF sf(_cam);
    sf.SetReprojectionFlow(rf);
    sf.SetEpipolar();
    sf.SetTwoCycleConsistency();
    sf.ConstructImagePyramid(image1, image2);
    sf.AlignImages();
    return sf.GetAlignmentResult();
}

void AlignImageMachine::RunRFlow() {
    vector<ReprojectionFlow*> rf;
    ReprojectionFlow r1(_cam, *maps[0]);
    ReprojectionFlow r2(_cam, *maps[1]);
    rf.push_back(&r1);
    rf.push_back(&r2);

    double gstatistic = 0;
    int poseloc1 = rf[0]->IdentifyClosestPose(por[1]->boat, por[0]->boat[poseloc0], &gstatistic);
    if(poseloc1 == -1)  return;

    std::cout << "aligning: ("<<dates[0] <<"." << poseloc0 << ") to ("<<dates[1] << "."<<poseloc1<<"). Saving to " << _saveloc << std::endl;
    
    ParseFeatureTrackFile pftf0 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[0], por[0]->ftfilenos[poseloc0]);
    ParseFeatureTrackFile pftf1 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[1], por[1]->ftfilenos[poseloc1]);

    rf[0]->ComputeFlow(por[1]->boat[poseloc1], por[0]->boat[poseloc0]); //map points of survey 0 onto pose1_est.
    rf[1]->ComputeFlow(por[0]->boat[poseloc0], por[1]->boat[poseloc1]); //map points of survey 1 onto pose0_est.
    rf[0]->CreateRestrictedSet(stoi(dates[0]), pftf0);
    rf[1]->CreateRestrictedSet(stoi(dates[1]), pftf1);
    
    string _image0 = ParseSurvey::GetImagePath(_query_loc + dates[0], por[0]->cimage[poseloc0]);
    string _image1 = ParseSurvey::GetImagePath(_query_loc + dates[1], por[1]->cimage[poseloc1]);
    AlignmentResult ar = RunSFlowWithRF(rf, _image0, _image1);
    
    ar.Save(_saveloc);
}

void * AlignImageMachine::Run() {
    thread_state = state::RUNNING;

    RunRFlow();

    thread_state = state::FINISHED;
    return (void *) NULL;
}

void AlignImageMachine::LogResults(){

}
