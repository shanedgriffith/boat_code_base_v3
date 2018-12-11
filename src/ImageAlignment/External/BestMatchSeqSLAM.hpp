/*
 * BestMatchSeqSLAM.hpp
 *
 *  Created on: Nov 30, 2016
 *      Author: shane
 */

#ifndef SRC_IMAGEALIGNMENT_EXTERNAL_BESTMATCHSEQSLAM_HPP_
#define SRC_IMAGEALIGNMENT_EXTERNAL_BESTMATCHSEQSLAM_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <math.h>
#include <random>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include <FileParsing/ParseSurvey.h>
#include <ImageAlignment/DREAMFlow/ImageOperations.h>
#include <DataTypes/AlignmentResult.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


/*
 * An implementation of SeqSLAM, based on Milford's 2014 paper for environmental and epidermal change monitoring.
 * */
class BestMatchSeqSLAM {
private:
	//inter-frame alignment
	const double delta = 0.55; //Fraction of best matches required within search range for overall match validation
	const double theta = 0.453786; //Search tolerance in radians = 26 degrees.
	const int n = 10; //Search range in frames

	//intra-frame alignment
	const int s = 40; //Width of patch comparison area in pixels
	const int res = 400; //Length of maximum dimension of patch normalized image in pixels
	const int q = 20; //Number of patches used in the intra-frame sequence search
	const int z = 50; //Length of sequence search within a frame in pixels
	const double f = 0.1; //Fractional range of image over which patch correspondences are sought

	//dataset specific parameters
	int CAM_SKIP = 5;
    
	//SeqSlam params.
	const int Rx = 48;
	const int Ry = 32;
	const int P = 8;
	const int Rwindow = 10;
    
	//Intra params
	const int NumGen = 20;
    
	void NormalizePatch(cv::Mat& image);
	cv::Mat PatchNormalizedImage(cv::Mat& image);
	double SumOfAbsDifference(cv::Mat& image1, cv::Mat& image2);
	std::pair<double, double> ComputeMeanAndStd(std::vector<double>& D, int s, int e);
	std::vector<double> LocalConstrastEnhancement(std::vector<double>& D);
	std::vector<std::vector<double> > BuildDifferenceMatrix(ParseSurvey ps1, ParseSurvey ps2);
	std::vector<int> FindTopRankedMatches(std::vector<std::vector<double> >& dm);
	std::vector<int> FillInBestMatches(std::vector<bool>& usable, std::vector<int>& topmatches);
	std::vector<int> CountBestMatchCandidates(int idx, int n, int m, std::vector<int>& topmatches);
	std::vector<std::vector<std::string> > GetMatchedImageList(ParseSurvey& ps1, ParseSurvey& ps2, std::vector<int>& best_matches);
    
	typedef struct{
		double angle;
        cv::Point2f origin;
		cv::Point2f end;
	}match_vector;

	typedef struct{
		std::vector<cv::Point2d> pref;
		std::vector<cv::Point2d> pnew;
	}correspondences;

	bool InsideImage(cv::Point2f p, int width, int height);
	match_vector GenerateRandomVector(int width, int height);
	std::vector<std::vector<std::vector<double> > > GetDifferenceMatrices(match_vector mv, cv::Mat pn1, cv::Mat pn2);
	cv::Point2d GetMatch(std::vector<std::vector<std::vector<double> > > difference_vectors);
	void AddToCorrespondences(correspondences& cs, cv::Point2d p, match_vector mv, int height, int width);
	cv::Mat FlowFieldFromHomography(cv::Mat homography, int height, int width);
    std::vector<cv::Point2d> GenerateMatchVector(int idx1, int idx2, cv::Point2d p1, cv::Point2d p2);
    std::vector<cv::Point2d> GetBestMatchVector(std::vector<std::vector<std::vector<double> > >& difference_vectors);
    void AddSeqToCorrespondences(correspondences& cs, std::vector<cv::Point2d> ps, match_vector mv, int height, int width);

public:
	std::string _date1, _date2;
    std::string _query_loc, _pftbase;
    BestMatchSeqSLAM(std::string date1, std::string date2, std::string pftbase, std::string query_loc):
		_date1(date1), _date2(date2), _query_loc(query_loc), _pftbase(pftbase)
	{}

	std::vector<std::vector<std::string> > InterSurveyAlignment();

    AlignmentResult IntraSurveyAlignment(std::string fim1, std::string fim2);
};


#endif /* SRC_IMAGEALIGNMENT_EXTERNAL_BESTMATCHSEQSLAM_HPP_ */
