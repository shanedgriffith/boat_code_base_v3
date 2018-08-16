//
//  FlickeringDisplay.cpp
//  SIFTFlow
//
//  Created by Shane Griffith on 6/24/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "FlickeringDisplay.h"
//#include "FileParsing/ParsePixelAlignmentLog.h"
#include "FileParsing/ParseVisibilityFile.h"
#include "FileParsing/ParseSurvey.h"
#include <FileParsing/FileParsing.hpp>
#include <ImageAlignment/DREAMFlow/ImageOperations.h>

using namespace std;
using namespace cv;

char const * FlickeringDisplay::window_name = "Flickering Display";
const string FlickeringDisplay::labelfile = "/labels.txt";

FILE * FlickeringDisplay::OpenFile(string filename, const char * op) {
    FILE * fs = fopen(filename.c_str(), op);
    if(!fs) {cout << "OpenFile: Error. Couldn't open " << filename << "." << endl; exit(-1);}
    return fs;
}

void FlickeringDisplay::MakeDir(string dir) {
    mkdir(dir.c_str(), (mode_t) (S_IRWXU | S_IRWXG | S_IRWXO));
}

bool FlickeringDisplay::copyFile(string s, string d) {
    const char *SRC = s.c_str();
    const char* DEST = d.c_str();
    std::ifstream src(SRC, std::ios::binary);
    std::ofstream dest(DEST, std::ios::binary);
    dest << src.rdbuf();
    return src && dest;
}

void FlickeringDisplay::WriteText(cv::Mat& img, string text) {
    cv::Point location(50,50);
    cv::putText( img, text, location, CV_FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 0, 0), 5, 8 );
}

void MouseFunc(int event, int x, int y, int flags, void* userdata) {
	 static int lx, ly;
	 static bool clear = true;
	 x = x % 704;
     if  ( event == EVENT_LBUTTONDOWN ) {
    	 cout << "clicked ("<<x<<", "<<y<<")"<<endl;
     }
     else if(event == EVENT_RBUTTONDOWN){
    	 cout << "rclicked ("<<x<<", "<<y<<") clear "<<clear<<endl;
    	 if(clear) {
			 lx = x;
			 ly = y;
			 clear = false;
    	 } else {
    		 clear = true;
    		 cout << "Flow: ("<<lx - x << ", " <<  ly-y << ")"<< endl;
    	 }
     }
}

bool FlickeringDisplay::ProcessLabel(char c) {
    /*
     adds the label to the list if it's sufficient.
     returns a bool that specifies whether to continue or not
     */
    
    switch(c) {
        case EOF:
            c=0;
            break;
        case 'q':
            if(debug) cout << "Quitting the visualization." << endl;
            finished = true;
            return true;
        case ' ':
            side_by_side = !side_by_side;
#if defined(__linux__)
            cv::destroyWindow(window_name);
            cv::namedWindow(window_name);
            cv::moveWindow(window_name, 650, 340);
            cv::setMouseCallback(window_name, MouseFunc, NULL);
#endif
            return false;
        case LABEL::PRECISE:
        case LABEL::COARSE:
        case LABEL::MISALIGNED:
            if(debug) cout << KEYS.at(c) << endl;
            labels.push_back(c);
            return true;
        default:
            return false;
    }
    return false;
}

cv::Mat FlickeringDisplay::CombinedImage(cv::Mat& im1, cv::Mat& im2) {
    /*Creates the side-by-side display. */
    cv::Mat img_object = im1;
    cv::Mat img_scene = im2;
    cv::Mat combine(max(img_object.size().height, img_scene.size().height), img_object.size().width + img_scene.size().width, CV_8UC3);
    cv::Mat left_roi(combine, cv::Rect(0, 0, img_object.size().width, img_object.size().height));
    img_object.copyTo(left_roi);
    cv::Mat right_roi(combine, cv::Rect(img_object.size().width, 0, img_scene.size().width, img_scene.size().height));
    img_scene.copyTo(right_roi);
    return combine;
}

void FlickeringDisplay::FlickerImagePair(cv::Mat& im1, cv::Mat& im2, cv::Mat& im2_w, string text) {
    /*
     Flickers a single image pair back and forth.
     */
    
    if(displayscore) {
        WriteText(im1, text);
        WriteText(im2, text);
    }
    
    cv::Mat both_images = FlickeringDisplay::CombinedImage(im1, im2);
    if(debug) cout << "im_sizes ("<<im1.rows <<","<<im1.cols<<") ("<<im2.rows<<","<<im2.cols<<") ("<<im2_w.rows<<","<<im2_w.cols<<") ("<<both_images.rows<<","<<both_images.cols<<")"<<endl;

    int toggle = -1;
    char c = 0;
    bool havelabel = false;
    while(!havelabel && !finished) {
        if(side_by_side) imshow(window_name, both_images);
        else if(toggle==1) imshow(window_name, im1);
        else imshow(window_name, im2_w);
        if(debug) cout << "toggle: " << toggle << ", sbs: " << side_by_side << endl;
        
        c = cvWaitKey(waittime);
        havelabel = ProcessLabel(c);
        toggle*=-1;
    }
}

void FlickeringDisplay::WriteLabels(string savedir) {
    FILE * fp = OpenFile(savedir + labelfile, "w");
    for(int i=0; i<labels.size(); i++) {
        fprintf(fp, "%c, %lf\n", labels[i], timing[i]);
    }
    fclose(fp);
}

void FlickeringDisplay::TimingStats() {
    double mint=10000000000;
    double maxt = -1;
    double sum=0;
    for(int i=0; i<timing.size(); i++)
    {
        if(timing[i] < mint) mint = timing[i];
        if(timing[i] > maxt) maxt = timing[i];
        sum += timing[i];
    }
    
    cout << "Timing stats. Total: " << sum << ", Average: "<<sum/timing.size()<<", Quickest: "<<mint<<", Slowest: "<<maxt<<endl;
}

void FlickeringDisplay::LabelStats() {
    vector<int> counter(KEYS.size(), 0);
    
    for(int i=0; i<labels.size(); i++) {
        switch(labels[i]){
            case LABEL::PRECISE:
                counter[0]++;
                break;
            case LABEL::COARSE:
                counter[1]++;
                break;
            case LABEL::MISALIGNED:
                counter[2]++;
                break;
        }
    }
    cout << "There are " << labels.size() << " labels." << endl;
    cout << "Precise: " << counter[0] << "\tCoarse: " << counter[1] << "\tMisaligned: " << counter[2] << endl;
}

//void FlickeringDisplay::DisplaySurveyComparison(bool save) {
//    //TODO: note base may not be the same for the visibility files and alignment logs.
//    ParsePixelAlignmentLog log(_expbase);
////    ParseVisibilityFile vis(poses_loc, _date1, _date2);
//
//    cv::namedWindow(window_name);
//    for(int i=0; i<log.Size() && !finished; i++) {
//        string image1 = ParseSurvey::GetImagePath(query_loc + _date1, log.imgs[i]);
//        string image2 = ParseSurvey::GetImagePath(query_loc + _date2, log.cimgs[i]);
//        string image2_warped = log.GetWarpedImageFile(i);
//        cv::Mat im1 = cv::imread(image1);
//        cv::Mat im2 = cv::imread(image2);
//        cv::Mat im2_w = cv::imread(image2_warped);
//
//        time_t start, end;
//        time (&start);
//        FlickerImagePair(im1, im2, im2_w, to_string(log.metric[i]));
//        time (&end);
//        timing.push_back(difftime(end, start));
//    }
//
//    cv::destroyWindow(window_name);
//    TimingStats();
//    LabelStats();
//    if(save) WriteLabels(_expbase);
//}

void FlickeringDisplay::CompareFromDir(string dir){
    std::vector<string> dirs = FileParsing::ListDirsInDir(dir);
    int lastsize = 0;
    while(lastsize != dirs.size()){
        std::vector<int> dnum(dirs.size());
        for(int i=0; i<dirs.size(); i++) dnum[i] = stoi(dirs[i]);
        std::sort(dnum.begin(), dnum.end());
        DisplaySurveyComparisonDir(dir, dnum, lastsize);
        dirs = FileParsing::ListDirsInDir(dir);
        lastsize = dirs.size();
    }
}

void FlickeringDisplay::DisplaySurveyComparisonDir(std::string dir, std::vector<int>& dnum, int start) {
    cv::namedWindow(window_name);
    for(int i=start; i<dnum.size() && !finished; i++) {
        string basedir = dir + "/" + to_string(dnum[i]) + "/";
        if(!FileParsing::DirectoryExists(basedir)) continue;
        string image1 = basedir + "ref.jpg";
        string image2 = basedir + "im2.jpg";
        string image2_warped = basedir + "warped.jpg";
        cv::Mat im1 = cv::imread(image1);
        cv::Mat im2 = cv::imread(image2);
        cv::Mat im2_w = cv::imread(image2_warped);
        
        time_t start, end;
        time (&start);
        FlickerImagePair(im1, im2, im2_w, "");
        time (&end);
        timing.push_back(difftime(end, start));
    }
    
    cv::destroyWindow(window_name);
    TimingStats();
    LabelStats();
    WriteLabels(dir);
}

//void FlickeringDisplay::DisplaySurveyComparisonCustom(bool save) {
//    //TODO: note base may not be the same for the visibility files and alignment logs.
//    _expbase = _expbase + "consistent/";//"consistent---first set of results/";//
//    ParsePixelAlignmentLog log(_expbase, "", true);
//    //    ParseVisibilityFile vis(poses_loc, _date1, _date2);
//
////    string ref = "_ref.png";
//////    string im2 ="_min_mapped_im2.png";
////    string im2 ="_consol_im2.png";
//    string ref = "_min_ref.png";
//    //    string im2 ="_min_mapped_im2.png";
//    string im2 ="_min_im2.png";
//
//    cv::namedWindow(window_name);
//    for(int i=0; i<log.Size() && !finished; i++) {
//        string image1 = _expbase + to_string(log.b2[i]) + ref;
//        string image2_warped = _expbase + to_string(log.b2[i]) + im2;
//        cv::Mat im1 = ImageOperations::Load(image1);
//        cv::Mat im2 = im1;
//        cv::Mat im2_w = ImageOperations::Load(image2_warped);
//        ImageOperations::DoubleImage(im1);
//        ImageOperations::DoubleImage(im2);
//        ImageOperations::DoubleImage(im2_w);
////        cv::Mat im1 = cv::imread(image1);
////        cv::Mat im2 = cv::imread(image1);
////        cv::Mat im2_w = cv::imread(image2_warped);
//
//        time_t start, end;
//        time (&start);
//        FlickerImagePair(im1, im2, im2_w, to_string(log.metric[i]));
//        time (&end);
//        timing.push_back(difftime(end, start));
//    }
//
//    cv::destroyWindow(window_name);
//    TimingStats();
//    LabelStats();
//    if(save) WriteLabels(_expbase);
//}

char FlickeringDisplay::FlickerImages(cv::Mat& im1, cv::Mat& im2){
    return FlickerImages(im1, im2, im2);
}

char FlickeringDisplay::FlickerImages(cv::Mat& im1, cv::Mat& im2, cv::Mat& im2_w) {
    FlickerImagePair(im1, im2, im2_w);
    if(labels.size() == 0) exit(-1);
    return labels[labels.size()-1];
}

FlickeringDisplay::FlickeringDisplay() {
    cv::namedWindow(window_name);
    cv::moveWindow(window_name, 650, 340);
    cv::setMouseCallback(window_name, MouseFunc, NULL);
}

//void FlickeringDisplay::VideoOfSurveyComparison(string dir) {
//    ParsePixelAlignmentLog log(_expbase);
//    ParseVisibilityFile vis(poses_loc, _date1, _date2);
//
//    MakeDir(dir);
//
//    int count = 0;
//    for(int i=0; i<log.Size(); i++) {
//        string image1 = ParseSurvey::GetImagePath(query_loc + _date1, log.imgs[i]);
//        string image2_warped = log.GetWarpedImageFile(i);
//
//        for(int j=0; j<times_shown; j++) {
//            string& ref = (j%2==0)?image1:image2_warped;
//            string save = dir + to_string(count) + ".jpg";
//            copyFile(ref, save);
//            count++;
//        }
//    }
//}


/*
TODO: determine the best way to write this if I need to use it again.
 
vector<int> FlickeringDisplay::FlickerDisplayMany(vector<ParseExperiment>& exps) {
//     Meant for tuning the survey comparison method. Toggle back and forth between
//     several different sets.
    vector<int> labels;
 
    int count = 0;//70;
    int toggle = -1;
    int cset = 1;
    char c = 0;
    vector<Mat> curset;
    bool changed = true;
    while(1) {
        if(changed) {
            curset.clear();
            string srv = exps[0].GetSurveyImage(count);
            Mat simg = imread(srv);
            if(simg.rows ==0 ) {cout << "empty image at " << srv << ", survey : " << exps[0].GetLabel() << endl; exit(1);}
            curset.push_back(simg);
            for(int i=0; i<exps.size(); i++)
            {
                string file = exps[i].GetWarpedImage(count);
                Mat inimg = imread(file);
                if(inimg.rows ==0 ) {cout << "empty image at " << file << ", survey : " << exps[i].GetLabel() << endl; exit(1);}
                curset.push_back(inimg);
            }
        }
        
        if(toggle==1) imshow(window_name, curset[0]);
        else imshow(window_name, curset[cset]);
        
        c = cvWaitKey(waittime);
        switch(c){
            case EOF:
                c=0;
                break;
            case 'q':
                cout << "Quitting the visualization." << endl;
                return false;
            case LABEL::PRECISE:
            case LABEL::COARSE:
            case LABEL::MISALIGNED:
                //            if(display_label_message) cout << "precise" << endl;
                labels.push_back(c);
                return true;
                
            case LABEL::PRECISE:
                if(display_label_message) cout << "precise" << endl;
                count++;
                changed = true;
                labels.push_back(cset-1);
                break;
            case LABEL::COARSE:
                if(display_label_message) cout << "coarse" << endl;
                count++;
                changed = true;
                labels.push_back(-1);
                break;
            case LABEL::MISALIGNED:
                if(display_label_message) cout << "bad" << endl;
                count++;
                changed = true;
                labels.push_back(-2);
                break;
            case LABEL::SET1:
                display_set = 1;
                break;
            case LABEL::SET2:
                display_set = 2;
                break;
            case LABEL::SET3:
                display_set = 3;
                break;
            case LABEL::SET4:
                display_set = 4;
                break;
        }
        
        toggle*=-1;
    }
    return labels;
}

void TuneSurveyComparisonMethod() {
    //parse the experiment for 44, 45, 46.
    //45 and 47 use r=28, cost=2.0
    //48 uses r=16, cost=2.0
    //46 r=28, cost=1.5.
    vector<ParseExperiment> exps;
    exps.push_back(ParseExperiment(44));
    exps.push_back(ParseExperiment(46));//updated version in 49.
    exps.push_back(ParseExperiment(48));
    
    //create the display to
    vector<int> labels = FlickerDisplayMany(exps);
    for(int i=0; i<labels.size(); i++) {
        int set = labels[i];
        if(set==-1) cout << i << ", coarse , -1"<< endl;
        if(set==-2) cout << i << ", bad , -1"<< endl;
        else cout << i << ", " << set << ", " << exps[set].GetMetric(i) << endl;
    }
}
*/

