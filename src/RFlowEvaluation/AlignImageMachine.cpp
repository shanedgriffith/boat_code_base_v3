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

void AlignImageMachine::Setup(int ploc0, std::string saveloc, int ploc1) {
    thread_state = state::LOCKED;
    poseloc0 = ploc0;
    poseloc1 = ploc1;
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
    poseloc1 = -1;
    basic = false;
    _image0 = "";
    _image1 = "";
    por = {};
    dates = {};
    maps = {};
    _saveloc = "";
    _offset = 0;
}

AlignmentResult AlignImageMachine::AlignImages(string image1, string image2, vector<ReprojectionFlow*> rf){
    SFlowDREAM2RF sf(_cam);
    if(rf.size() > 0) sf.SetReprojectionFlow(rf);
    sf.SetEpipolar();
    sf.SetTwoCycleConsistency();
    sf.ConstructImagePyramid(image1, image2);
    sf.AlignImages();
    return sf.GetAlignmentResult();
}

void AlignImageMachine::SetImages(std::string i0, std::string i1, std::string savename){
    thread_state = state::LOCKED;
    basic = true;
    _image0 = i0;
    _image1 = i1;
    _saveloc = savename;
}

void AlignImageMachine::SetOffset(int offset){
    _offset = offset;
}

void AlignImageMachine::RunSFlow(){
    AlignmentResult ar = AlignImages(_image0, _image1);
    ar.SaveWarpedImage(_saveloc);
}

void AlignImageMachine::RunRFlow() {
    vector<ReprojectionFlow*> rf;
    ReprojectionFlow r1(_cam, *maps[0]);
    ReprojectionFlow r2(_cam, *maps[1]);
    rf.push_back(&r1);
    rf.push_back(&r2);
    
    if(poseloc1 == -1) {
        double gstatistic = 0;
        int poseloc0WithOffset = std::max(0, std::min(static_cast<int>(poseloc0+_offset), static_cast<int>(por[0]->boat.size()-1)));
        poseloc1 = rf[0]->IdentifyClosestPose(por[1]->boat, por[0]->boat[poseloc0WithOffset], &gstatistic);
        if(poseloc1 == -1)  return;
    }
    
    std::cout << "aligning: ("<<dates[0] <<"." << poseloc0 << ") to ("<<dates[1] << "."<<poseloc1<<"). Saving to " << _saveloc << std::endl;
    
    ParseFeatureTrackFile pftf0 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[0], por[0]->ftfilenos[poseloc0]);
    ParseFeatureTrackFile pftf1 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[1], por[1]->ftfilenos[poseloc1]);
    
    rf[0]->ComputeFlow(por[1]->boat[poseloc1], por[0]->boat[poseloc0]); //map points of survey 0 onto pose1_est.
    rf[1]->ComputeFlow(por[0]->boat[poseloc0], por[1]->boat[poseloc1]); //map points of survey 1 onto pose0_est.
    rf[0]->CreateRestrictedSet(stoi(dates[0]), pftf0);
    rf[1]->CreateRestrictedSet(stoi(dates[1]), pftf1);
    
    _image0 = ParseSurvey::GetImagePath(_query_loc + dates[0], por[0]->cimage[poseloc0]);
    _image1 = ParseSurvey::GetImagePath(_query_loc + dates[1], por[1]->cimage[poseloc1]);
    AlignmentResult ar = AlignImages(_image0, _image1, rf);
    
    if(_saveloc.find(".jpg") != std::string::npos){
        ar.SaveWarpedImage(_saveloc);
    } else {
        string _savename_rf =  _saveloc + "rf/" + dates[1]+"_" +  to_string(poseloc1) + "_" +to_string(_offset)+ "_.jpg";
        string _savename_ref = _saveloc + "ref_" + dates[0]+"_" + to_string(poseloc0) + "_" +to_string(_offset)+"_.jpg";
        string _savename_im2 = _saveloc + "scene/" + dates[1]+"_" + to_string(poseloc1) + "_" +to_string(_offset)+ "_.jpg";
        string _savename_up = _saveloc + "mappoints/" + dates[1]+"_" + to_string(poseloc1) + "_" +to_string(_offset)+ "_.jpg";
        string _savename_refp = _saveloc + "mappoints/ref_" + dates[1]+"_" + to_string(poseloc1) + "_" +to_string(_offset)+ "_.jpg";
        string _visualize_points = _saveloc + "viewpoint/" + dates[1]+"_" + to_string(poseloc1) + ".jpg";
        
        rf[0]->DrawViewset(por[0]->boat[poseloc0], por[1]->boat[poseloc1], _visualize_points);
        ar.SaveWarpedImage(_savename_rf);
        ImageOperations::Save(ar.ref, _savename_ref);
        ImageOperations::Save(ar.im2, _savename_im2);
        rf[0]->DrawFlowPoints(ar.im2);
        ImageOperations::Save(ar.im2, _savename_up);
        rf[0]->DrawMapPoints(ar.ref);
        ImageOperations::Save(ar.ref, _savename_refp);
        
        _saveloc =  _saveloc + "sf/" + dates[1]+"_" +  to_string(poseloc1) + "_" +to_string(_offset)+ "_.jpg";
        RunSFlow();
    }
}

void * AlignImageMachine::Run() {
    thread_state = state::RUNNING;

    if(basic) RunSFlow();
    else RunRFlow();

    thread_state = state::FINISHED;
    return (void *) NULL;
}

void AlignImageMachine::LogResults(){

}
