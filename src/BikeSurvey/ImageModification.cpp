/*
 * ImageModification.cpp
 *
 *  Created on: May 1, 2017
 *      Author: shane
 */

#include <BikeRoute/ImageModification.hpp>
#include "FileParsing/FileParsing.hpp"

const std::string ImageModification::_facefile = "/haarcascade_frontalface_alt.xml";
const std::string ImageModification::_profilefile = "/haarcascade_profileface.xml";

void ImageModification::BlurROI(cv::Mat& image, cv::Rect r){
    cv::GaussianBlur(image(r), image(r), cv::Size(0, 0), 20);
}

//reference http://stackoverflow.com/questions/20757147/detect-faces-in-image
void ImageModification::DetectFacesAndBlur(cv::Mat& image){
    cv::Mat frame_gray;
    std::vector<cv::Rect> faces;
    cv::cvtColor(image, frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);

    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    std::vector<cv::Rect> profiles;
    profile_cascade.detectMultiScale(frame_gray, profiles, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    faces.insert(faces.end(), profiles.begin(), profiles.end());

    for(int i=0; i<faces.size(); i++){
        if(i==0) std::cout << "face detected!" << std::endl;
        BlurROI(image, faces[i]);
    }
}

void ImageModification::Test(){
    cv::Mat image = cv::imread("/cs-share/dream/people.jpeg");
    DetectFacesAndBlur(image);
    cv::imwrite("/cs-share/dream/people-blurred.jpeg", image);
}
