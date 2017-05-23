/*
 * MultiManualCorrespondence.hpp
 *
 *  Created on: Nov 18, 2016
 *      Author: shane
 */

#ifndef SRC_EVALUATION_MULTIMANUALCORRESPONDENCE_HPP_
#define SRC_EVALUATION_MULTIMANUALCORRESPONDENCE_HPP_

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <FileParsing/FileParsing.hpp>

class MultiManualCorrespondence: public FileParsing{
private:
	typedef struct{
		cv::Point2f lp;
		cv::Point2f rp;
	}correspondence;

	typedef struct{
		  std::string date1;
		  std::string date2;
		  int im1;
		  int im2;
		  double labeltime;
		  std::vector<correspondence> points;
	}handlabels;

	std::vector<handlabels> correspondenceset;

	int GetImageNumberFromImagePath(std::string imagepath);
	std::string GetDateFromImagePath(std::string imagepath);
	void ProcessLineEntries(int type, std::vector<std::string> lp);
	void ReadDelimitedFile(std::string file, int type);

public:

	MultiManualCorrespondence(std::string file){
		std::cout << "Reading: " << file << std::endl;
		ReadDelimitedFile(file, 0);
		std::cout <<"Number of labeled image pairs: " << GetNumberOfImagePairs() << std::endl;
		std::cout <<"Median number of points per pair: " << MedianNumberOfPointsPerPair() << std::endl;
		PrintHist();
	}

	int GetNumberOfImagePairs();
	int MedianNumberOfPointsPerPair();
	void PrintHist();
};



#endif /* SRC_EVALUATION_MULTIMANUALCORRESPONDENCE_HPP_ */
