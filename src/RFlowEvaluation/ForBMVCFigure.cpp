/*
 * ForBMVCFigure.cpp
 *
 *  Created on: May 11, 2016
 *      Author: shane
 */

#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseSurvey.h>
#include <ImageAlignment/DREAMFlow/SFlowDREAM.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <RFlowOptimization/SFlowDREAM2RF.hpp>

#include "ForBMVCFigure.hpp"

using namespace std;


void ForBMVCFigure::AlignSection(int num, string date0, string date1, int offset){
	/*Used to align each survey to survey 1 */
    
    std::vector<Map> maps;
    maps.push_back(Map(_map_base));
    maps.push_back(Map(_map_base));
    maps[0].LoadMap(date0);
    maps[1].LoadMap(date1);
    
    vector<ReprojectionFlow*> rf;
    ReprojectionFlow r1(_cam, maps[0]);
    ReprojectionFlow r2(_cam, maps[1]);
    rf.push_back(&r1);
    rf.push_back(&r2);
    
    ParseOptimizationResults por0(_map_base, date0);
    ParseOptimizationResults por1(_map_base, date1);
    
    double gstatistic = 0;
    int poseloc = rf[0]->IdentifyClosestPose(por1.boat, por0.boat[num], &gstatistic);
    if(poseloc == -1)  {
        std::cout << "no poses between " << date0 << "." <<num <<  " and " << date1 << std::endl;
        return;
    }
    
    ParseFeatureTrackFile pftf0 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + date0, por0.ftfilenos[num]);
    ParseFeatureTrackFile pftf1 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + date1, por1.ftfilenos[poseloc]);
    
    rf[0]->ComputeFlow(por1.boat[poseloc], por0.boat[num]); //map points of survey 0 onto pose1_est.
    rf[1]->ComputeFlow(por0.boat[num], por1.boat[poseloc]); //map points of survey 1 onto pose0_est.
    rf[0]->CreateRestrictedSet(stoi(date0), pftf0);
    rf[1]->CreateRestrictedSet(stoi(date1), pftf1);
    
    string _image0 = ParseSurvey::GetImagePath(_query_loc + date0, por0.cimage[num]);
    string _image1 = ParseSurvey::GetImagePath(_query_loc + date1, por1.cimage[poseloc]);

    string savedir = _savebase + date0 + to_string(num) + "/";
    FileParsing::MakeDir(savedir);
    FileParsing::MakeDir(savedir + "rf");
    FileParsing::MakeDir(savedir + "sf");
    FileParsing::MakeDir(savedir + "scene");
    FileParsing::MakeDir(savedir + "mappoints");
    FileParsing::MakeDir(savedir + "viewpoint");
    
    string _savename_rf =  savedir + "rf/" + date1+"_" +  to_string(poseloc) + "_" +to_string(offset)+ "_.jpg";
    string _savename_sf = savedir + "sf/" + date1+"_" + to_string(poseloc) + "_" +to_string(offset)+ "_.jpg";
    string _savename_im2 = savedir + "scene/" + date1+"_" + to_string(poseloc) + "_" +to_string(offset)+ "_.jpg";
    string _savename_up = savedir + "mappoints/" + date1+"_" + to_string(poseloc) + "_" +to_string(offset)+ "_.jpg";
    string _savename_refp = savedir + "mappoints/ref_" + date1+"_" + to_string(poseloc) + "_" +to_string(offset)+ "_.jpg";
    string _savename_ref = savedir + "ref_" + date0+"_" + to_string(num) + "_" +to_string(offset)+"_.jpg";
    string _visualize_points = savedir + "viewpoint/" + date1+"_" + to_string(poseloc) + ".jpg";
    
//    string _savename_sfepi = _savebase + to_string(num) + "_warped_sf_basicepi_" + date1+"_" +to_string(offset)+ "_.jpg";
//    string _savename_sfcons = _savebase + to_string(num) + "_warped_sf_basiccons_" + date1+"_" +to_string(offset)+ "_.jpg";
//    string _savename_sfbrf = _savebase + to_string(num) + "_warped_sf_basicrf_" + date1+"_" +to_string(offset)+ "_.jpg";

    rf[0]->DrawViewset(por0.boat[num], por1.boat[poseloc], _visualize_points);
    
    //run image alignment
    SFlowDREAM2RF sf(_cam);
    sf.SetReprojectionFlow(rf);
    sf.SetEpipolar();
    sf.SetTwoCycleConsistency();
    sf.ConstructImagePyramid(_image0, _image1);
    sf.AlignImages();
    
	AlignmentResult ar = sf.GetAlignmentResult();
	ar.SaveWarpedImage(_savename_rf);
	ImageOperations::Save(ar.ref, _savename_ref);
	ImageOperations::Save(ar.im2, _savename_im2);
	rf[0]->DrawFlowPoints(ar.im2);
	ImageOperations::Save(ar.im2, _savename_up);
	rf[0]->DrawMapPoints(ar.ref);
	ImageOperations::Save(ar.ref, _savename_refp);
    
//    std::cout << "not running the sift flow version" << std::endl;
//    return;
//    
    SFlowDREAM2RF sfbasic(_cam);
    sfbasic.ConstructImagePyramid(_image0, _image1);
    sfbasic.AlignImages();
    AlignmentResult arbasic = sfbasic.GetAlignmentResult();
    arbasic.SaveWarpedImage(_savename_sf);
//
//    SFlowDREAM2RF sfbasicepi(_cam);
//    sfbasicepi.SetEpipolar();
//    sfbasicepi.ConstructImagePyramid(_image0, _image1);
//    sfbasicepi.AlignImages();
//    AlignmentResult arbasice = sfbasicepi.GetAlignmentResult();
//    arbasice.SaveWarpedImage(_savename_sfepi);
    
//    SFlowDREAM2RF sfbasiccons(_cam);
//    sfbasiccons.SetEpipolar();
//    sfbasiccons.SetTwoCycleConsistency();
//    sfbasiccons.ConstructImagePyramid(_image0, _image1);
//    sfbasiccons.AlignImages();
//    AlignmentResult arbasicc = sfbasiccons.GetAlignmentResult();
//    arbasicc.SaveWarpedImage(_savename_sfcons);
    
//    SFlowDREAM2RF sfbasicrf(_cam);
//    sf.SetReprojectionFlow(rf);
//    sfbasicrf.ConstructImagePyramid(_image0, _image1);
//    sfbasicrf.AlignImages();
//    AlignmentResult arbasicr = sfbasicrf.GetAlignmentResult();
//    arbasicr.SaveWarpedImage(_savename_sfbrf);
}


void ForBMVCFigure::GetAlignmentAtSection(string ref_date, int num, bool viewpoint_variance){
	vector<int> offset = {-15, -10, -5, 0, 5, 10, 15};

	for(int i=0; i<_dates.size(); i++){
        if(ref_date == _dates[i]) continue;
		if(viewpoint_variance){
			for(int j=0; j<offset.size(); j++){
				AlignSection(num+offset[j], ref_date, _dates[i], offset[j]);
			}
		}
		else AlignSection(num, ref_date, _dates[i], 0);
	}
}

























