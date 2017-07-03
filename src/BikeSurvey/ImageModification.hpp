/*
 * ImageModification.hpp
 *
 *  Created on: May 1, 2017
 *      Author: shane
 */

#ifndef SRC_BIKESURVEY_IMAGEMODIFICATION_HPP_
#define SRC_BIKESURVEY_IMAGEMODIFICATION_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class ImageModification{
private:
    static const std::string _facefile;
    static const std::string _profilefile;

    void BlurROI(cv::Mat& image, cv::Rect r);

    cv::CascadeClassifier face_cascade;
    cv::CascadeClassifier profile_cascade;
public:
    ImageModification(std::string opencv_path){
        if (!face_cascade.load(opencv_path + _facefile)) {
            std::cout << "OpenCV couldn't load the cascade file: " << opencv_path + _facefile << std::endl;
            exit(-1);
        }
        if (!profile_cascade.load(opencv_path + _profilefile)) {
            std::cout << "OpenCV couldn't load the cascade file: " << opencv_path + _profilefile << std::endl;
            exit(-1);
        }
    }

    void DetectFacesAndBlur(cv::Mat& image);

    void Test();
};



#endif /* SRC_BIKESURVEY_IMAGEMODIFICATION_HPP_ */
