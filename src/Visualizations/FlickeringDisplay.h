//
//  FlickeringDisplay.h
//  SIFTFlow
//
//  Created by Shane Griffith on 6/24/15.
//  Copyright (c) 2015 shane. All rights reserved.
//
/*
 Note: Although humans are sensitive to changes in flickering images of a scene, 
 research indicates minor changes in a scene could obfuscate pertinent ones 
 [research on mudsplashes]. Thus, without a user study to demonstrate the 
 effectiveness of this approach, it may look weak for an evaluation.
 */

#ifndef __SIFTFlow__FlickeringDisplay__
#define __SIFTFlow__FlickeringDisplay__

#include <stdio.h>
#include <string.h>
#include <vector>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

extern const std::string base;
extern const std::string optimized_datasets;
extern const std::string query_loc;
extern const std::string poses_loc;

class FlickeringDisplay{
private:
    bool displayscore = false;
    static char const * window_name;
    static const std::string labelfile;
    int waittime = 500; //ms
    int times_shown = 6;
    
    bool finished = false;
    bool side_by_side = false;
    
    void WriteText(cv::Mat& img, string text);
    
    //TODO: in my attempt to create an abstract label space and key input, I think I ended up just making it more complicated.
    // It's not currently worth the time to improve.
    enum LABEL{PRECISE='g', COARSE='c', MISALIGNED='b'};
    const std::unordered_map<char, std::string> KEYS {
        {LABEL::PRECISE, "precise" },
        {LABEL::COARSE, "coarse" },
        {LABEL::MISALIGNED, "misaligned"}
    };
    
    FILE * OpenFile(std::string filename, const char * op);
    void MakeDir(std::string dir);
    bool copyFile(std::string s, std::string d);
    void WriteLabels(std::string savedir);
    void LabelStats();
    void TimingStats();
    
//    void MouseFunc(int event, int x, int y, int flags, void* userdata);
    bool ProcessLabel(char c);
    
    cv::Mat CombinedImage(cv::Mat& im1, cv::Mat& im2);
    void FlickerImagePair(cv::Mat& im1, cv::Mat& im2, cv::Mat& im2_w, std::string text="");
    void DisplaySurveyComparisonDir(string dir, std::vector<int>& dnum, int start=0);
    
    bool display_label_message = false;
    
public:
    std::vector<char> labels;
    std::vector<double> timing;
    std::string _date1, _date2;//_expbase, _surveybase,
    std::string _expbase;
    bool debug = false;
    
    //use these to get a quick result
    FlickeringDisplay();
    ~FlickeringDisplay(){
        cv::destroyWindow(window_name);
    }
    char FlickerImages(cv::Mat& im1, cv::Mat& im2, cv::Mat& im2_w);
    char FlickerImages(cv::Mat& im1, cv::Mat& im2);
    
    //use these to perform survey comparison on saved images.
    FlickeringDisplay(std::string date1, std::string date2): _date1(date1), _date2(date2) {
        _expbase = base + date2 + "-" + date1 + "/";
    }
    
    void SetDisplayScore(){displayscore = true;}
    void DisplaySurveyComparison(bool save = false);
    void DisplaySurveyComparisonCustom(bool save);
    void VideoOfSurveyComparison(std::string dir);
    void CompareFromDir(std::string dir);
    
//    vector<int> FlickerDisplayMany(vector<ParseExperiment>& exps);
};

#endif /* defined(__SIFTFlow__FlickeringDisplay__) */


//TODO: judge the time for each.
