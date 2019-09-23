//
//  ManualKeypointCorrespondence.cpp
//  boat_code_base_v2
//
//  Created by Shane on 1/11/19.
//  Copyright © 2019 Shane. All rights reserved.
//

#include "ManualKeypointCorrespondence.hpp"
#include <string>
#include <Visualizations/FlickeringDisplay.h>
#include <FileParsing/FileParsing.hpp>
#include <Visualizations/IMDraw.hpp>
#include <FileParsing/ParseSurvey.h>
#include <ImageAlignment/DREAMFlow/ImageOperations.h>
#include <RFlowOptimization/LocalizePose.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>

#include "Optimization/SingleSession/GTSAMInterface.h"

static void
MouseFunc(int event, int x, int y, int flags, void* userdata) {
    ManualKeypointCorrespondence* mic = static_cast<ManualKeypointCorrespondence*>(userdata);
    if  ( event == cv::EVENT_LBUTTONDOWN ) {
        mic->SetPoint(x,y);
        std::cout << "mouse clicked " << x << ", " << y << std::endl;
    }
}

void
ManualKeypointCorrespondence::SetPoint(int x, int y) {
    if(x < w) {
        double md = 100000000;
        int idx = -1;
        for(int i=0; i<_pftf.imagecoord.size(); i++){
            double dist = pow(pow(_pftf.imagecoord[i].x()-x,2)+pow(_pftf.imagecoord[i].y()-y,2),0.5);
            if(dist < md) {
                md = dist;
                idx = i;
            }
        }
        leftpidx = idx;
    }else
        right = cv::Point2i(x-w,y);
    redraw = true;
}

void
ManualKeypointCorrespondence::AddPair() {
    if(leftpidx != -1 and right.x !=-1) {
        indices.push_back(leftpidx);
        correspondences.push_back(right);
    }
    leftpidx = -1;
    right = cv::Point2f(-1,-1);
}

void
ManualKeypointCorrespondence::UpdateProjection(cv::Mat imgr, cv::Mat imgt) {
    ReprojectionFlow rf(_cam, _m);
    rf.ComputeFlow(_lpose, _por0.boat[_p0]); //map points of survey 0 onto pose1_est.
    rf.CreateRestrictedSet(stoi(_d0), _pftf);
    cv::Mat c0 = imgr.clone();
    cv::Mat c1 = imgt.clone();
    rf.DrawMapPoints(c0);
    rf.DrawFlowPoints(c1);
    _preview = FlickeringDisplay::CombinedImage(c0, c1);
}

void
ManualKeypointCorrespondence::UpdateLocalizedPose() {
    if(indices.size() <= 4) {
        std::cout << "Not enough points for localization. Have " << indices.size() << std::endl;
        _show_preview = false;
        return;
    }
    SortPoints();
    std::vector<int> landmark_ids;
    std::vector<gtsam::Point2> p2d;
    for(int j=0; j<correspondences.size(); ++j) {
        landmark_ids.push_back(_pftf.ids[indices[j]]);
        p2d.push_back(gtsam::Point2(correspondences[j].x, correspondences[j].y));
    }
    std::vector<gtsam::Point3> p3d = _por0.GetSubsetOf3DPoints(landmark_ids);
    std::vector<double> inliers(p2d.size(), 1);
    
    LocalizePose lp(_cam);
    lp.debug = true;
    lp.SetErrorThreshold(25.0);
//    std::vector<std::vector<double> > newpose = lp.UseBAIterative(_por1.boat[_p1], p3d, p2d, inliers);
    gtsam::Pose3 estp = lp.P3P(p3d, p2d);
    _lpose = GTSAMInterface::PoseToVector(estp);
    
//    if(newpose.size()==0) {
//        std::cout << "No localization" << std::endl;
//        _show_preview = false;
//        return;
//    }
//    _lpose = newpose[0];
}

bool
ManualKeypointCorrespondence::ProcessLabel(char c) {
    switch(c) {
        case 'a':
            accepted = true;
            std::cout << "accepted localization" << std::endl;
            return false;
        case 'u':
            accepted = false;
            std::cout << "unaccepted localization" << std::endl;
            return false;
        case 'q':
            std::cout << "finished labeling" << std::endl;
            return false;
        case 's':
            AddPair();
            SaveCorrespondences();
            return true;
        case 'n':
            AddPair();
            std::cout << "number of correspondences: " << indices.size() << std::endl;
            return true;
        case 'd':
            if(leftpidx != -1) {
                for(int i=0; i<indices.size(); i++) {
                    if(indices[i] == leftpidx) {
                        indices.erase(indices.begin()+i, indices.begin() + i + 1);
                        correspondences.erase(correspondences.begin()+i, correspondences.begin()+i+1);
                        break;
                    }
                }
            }
            return true;
        case 'p':
            _show_preview = !_show_preview;
            return true;
        default:
            return true;
    }
    return true;
}

void
ManualKeypointCorrespondence::RedrawImages(cv::Mat img) {
    cv::Point2i ro(w,0);
    
    for(int i=0; i<_pftf.imagecoord.size(); i++) {
        CvScalar color = IMDraw::GetLandmarkColor(i);
        cv::Point2f p(_pftf.imagecoord[i].x(), _pftf.imagecoord[i].y());
        circle(img, p, 3, color, -1, 8, 0);
    }
    
    for(int i=0; i<correspondences.size(); ++i) {
        CvScalar color = IMDraw::GetLandmarkColor(i*10);
        cv::Point2f p(_pftf.imagecoord[indices[i]].x(), _pftf.imagecoord[indices[i]].y());
        circle(img, p, 3, color, -1, 8, 0);
        circle(img, correspondences[i] + ro, 3, color, -1, 8, 0);
    }
    
    CvScalar defcolor = CV_RGB(255,255,255);
    if(leftpidx != -1) {
        cv::Point2f p(_pftf.imagecoord[leftpidx].x(), _pftf.imagecoord[leftpidx].y());
        circle(img, p, 5, defcolor, -1, 8, 0);
        circle(img, p, 4, CV_RGB(255,0,0), -1, 8, 0);
    }
    if(right.x != -1) {
        circle(img, right+ro, 5, defcolor, -1, 8, 0);
        circle(img, right+ro, 4, CV_RGB(255,0,0), -1, 8, 0);
    }
}

void
ManualKeypointCorrespondence::ShowImagePair(cv::Mat& im1, cv::Mat& im2) {
    w = im1.cols;
    h = im1.rows;
    
    cv::Mat comb = FlickeringDisplay::CombinedImage(im1, im2);
    
    std::string window_name = "manual correspondence";
    
    cv::namedWindow(window_name);
    cv::moveWindow(window_name, 0, 340);
    ManualKeypointCorrespondence * mic = this;
    cv::setMouseCallback(window_name, MouseFunc, (void *)(mic));
    bool last_preview = false;
    bool cont = true;
    while(cont) {
        cv::Mat displayed;
        if(last_preview != _show_preview and _show_preview) {
//            if(indices.size() >= 15) UpdateLocalizedPose();
            UpdateProjection(im1, im2);
        }
        last_preview = _show_preview;
        if( _show_preview) displayed = _preview;
        else {
            displayed = comb.clone();
            RedrawImages(displayed);
        }
        cv::imshow(window_name, displayed);
        char c = cvWaitKey(30);
        cont = ProcessLabel(c);
    }
    cv::destroyWindow(window_name);
}

void
ManualKeypointCorrespondence::SortPoints() {
    for(int i=0; i<indices.size(); ++i) {
        int md = indices[i];
        int mi = i;
        for(int j=i+1; j<indices.size(); j++) {
            if(indices[j] < md) {
                md = indices[j];
                mi = j;
            }
        }
        std::swap(indices[mi], indices[i]);
        std::swap(correspondences[mi], correspondences[i]);
    }
}

std::string
ManualKeypointCorrespondence::CorrespondencesToString() {
    SortPoints();
    std::string data = "";
    for(int i=0; i<correspondences.size(); ++i) {
        data += std::to_string(indices[i]) + ", " +
        std::to_string(correspondences[i].x) + ", " +
        std::to_string(correspondences[i].y) + "\n";
    }
    return data;
}

std::string
ManualKeypointCorrespondence::GetFilename(std::string d0, int p0, std::string d1, int p1) {
    return d0 + "_" + std::to_string(p0) + "-" + d1 + "_" + std::to_string(p1) + ".txt";
}

void
ManualKeypointCorrespondence::SaveCorrespondences() {
    std::string data = CorrespondencesToString();
    std::string savef = _save_dir + GetFilename(_d0, _p0, _d1, _p1);
    FileParsing::AppendToFile(savef, data, false);
    std::cout << indices.size() << " correspondences saved to : " << savef << std::endl;
}

void
ManualKeypointCorrespondence::RunManualCorrespondence(std::string d0, int p0, std::string d1, int p1) {
    _d0 = d0;
    _d1 = d1;
    _p0 = p0;
    _p1 = p1;
    
    _m.LoadMap(_d0);
    _por0 = ParseOptimizationResults(_map_base, d0);
    _por1 = ParseOptimizationResults(_map_base, d1);
    std::string image0 = ParseSurvey::GetImagePath(_query_loc + d0, _por0.cimage[p0]);
    std::string image1 = ParseSurvey::GetImagePath(_query_loc + d1, _por1.cimage[p1]);
    cv::Mat im0 = ImageOperations::Load(image0);
    cv::Mat im1 = ImageOperations::Load(image1);
    _pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + d0, _por0.ftfilenos[p0]);
    
    _lpose = _por1.boat[p1];
    ShowImagePair(im0, im1);
}

void
ManualKeypointCorrespondence::GetLocalizationList() {
    std::vector<std::string> files = FileParsing::ListFilesInDir(_save_dir, ".txt");
    
    std::string savef = _save_dir + "localization_result.csv";
    for(int i=0; i<files.size(); i++) {
        std::string d0 = files[i].substr(0, 6);
        int p0 = stoi(files[i].substr(7, files[i].find("-")-7));
        std::string d1 = files[i].substr(files[i].find("-")+1, 6);
        int p1 = stoi(files[i].substr(files[i].rfind("_")+1, files[i].rfind(".") - files[i].rfind("_") - 1));
        
        std::string loadf = _save_dir + files[i];
        std::vector<std::vector<std::string> > fs = FileParsing::ReadCSVFile(loadf);
        
        for(int j=0; j<fs.size(); ++j) {
            indices.push_back(stoi(fs[j][0]));
            correspondences.push_back(cv::Point2i(stoi(fs[j][1]), stoi(fs[j][2])));
        }
        
        _show_preview = true;
        RunManualCorrespondence(d0, p0, d1, p1);
        if(indices.size() < 15) {
            std::cout << "not enough correspondences to accept it. Have " << indices.size() << " Check " << files[i] << std::endl;
            accepted = false;
        }
        
        std::string data = std::to_string(accepted) + ", " + d0 + ", " + std::to_string(p0) + ", " + d1 + ", " + std::to_string(p1);
        for(int j=0; j<_lpose.size(); ++j)
            data += ", " + std::to_string(_lpose[j]);
        data += "\n";
        FileParsing::AppendToFile(savef, data, true);
        
        indices.clear();
        correspondences.clear();
    }
    
    std::cout << "files: " << files.size() << std::endl;
}

void
ManualKeypointCorrespondence::testManualCorrespondence(std::string d0, int p0, std::string d1, int p1)
{
    std::string loadf = _save_dir + GetFilename(_d0, _p0, _d1, _p1);
    std::vector<std::vector<std::string> > fs = FileParsing::ReadCSVFile(loadf);
    
    for(int j=0; j<fs.size(); ++j) {
        indices.push_back(stoi(fs[j][0]));
        correspondences.push_back(cv::Point2i(stoi(fs[j][1]), stoi(fs[j][2])));
    }
    
    _show_preview = true;
    RunManualCorrespondence(d0, p0, d1, p1);
}






