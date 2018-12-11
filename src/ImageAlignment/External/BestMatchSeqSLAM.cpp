/*
 * BestMatchSeqSLAM.cpp
 *
 *  Created on: Nov 30, 2016
 *      Author: shane
 */


#include "BoatSurvey/ParseBoatSurvey.hpp"
#include "BestMatchSeqSLAM.hpp"



void BestMatchSeqSLAM::NormalizePatch(cv::Mat& image){
	int minv = 255;
	int maxv = 0;
	unsigned char* data = (unsigned char *) image.data;
	for(int i=0; i<image.rows; i++){
		for(int j=0; j<image.cols; j++){
			int idx = i*image.cols + j;
			cv::Scalar s = data[idx];
			maxv = std::max(maxv, (int) s.val[0]);
			minv = std::min(minv, (int) s.val[0]);
		}
	}

	for(int i=0; i<image.rows; i++){
		for(int j=0; j<image.cols; j++){
			int idx = i*image.cols + j;
			data[idx] = (data[idx] - minv)/(maxv-minv);
		}
	}
}

cv::Mat BestMatchSeqSLAM::PatchNormalizedImage(cv::Mat& image){
	cv::Mat gray_image;

	cv::Mat pnimage(Ry, Rx, CV_8UC1, cv::Scalar(0,0,0));
	cv::cvtColor( image, gray_image, CV_BGR2GRAY );
	cv::resize(gray_image, pnimage, pnimage.size(), 0,0,CV_INTER_AREA);

	for(int i=0; i<Ry/P; i++){
		for(int j=0; j<Rx/P; j++){
			cv::Rect roi(j, i, P, P);
			cv::Mat ref = pnimage(roi);
			NormalizePatch(ref);
		}
	}

	return pnimage;
}

double BestMatchSeqSLAM::SumOfAbsDifference(cv::Mat& image1, cv::Mat& image2){
	//The input images are patch-normalized.
	double sum=0;
	unsigned char* data1 = (unsigned char *) image1.data;
	unsigned char* data2 = (unsigned char *) image2.data;
	for(int i=0; i<image1.rows; i++){
		for(int j=0; j<image1.cols; j++){
			int idx = i*image1.cols + j;
			sum += std::abs((int) data1[idx] - (int) data2[idx]);
		}
	}
	return sum/(Rx*Ry);
}

std::pair<double, double> BestMatchSeqSLAM::ComputeMeanAndStd(std::vector<double>& D, int s, int e){
    /* //TODO check this.
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance> > acc;
    std::for_each(D.begin()+s, D.begin()+e, std::bind<void>(std::ref(acc), _1));
	return std::pair<double, double>(boost::accumulators::mean(acc), std::sqrt((double)boost::accumulators::variance(acc)));*/
    return std::pair<double, double>();
}

std::vector<double> BestMatchSeqSLAM::LocalConstrastEnhancement(std::vector<double>& D){
	std::vector<double> Dp(D.size(), 0);
	int h = std::ceil(Rwindow/2.0);
	for(int i=0; i<D.size(); i++){
		int s=i-h;
		int e=i+h;
		if(s<0){
			e += abs(s);
			s += abs(s);
		}
		if(e>D.size()){
            s = std::max(0, (int)D.size() - s); //s - (e - D.size())
			e = D.size(); //e - (e - D.size());
		}
		std::pair<double, double> ms = ComputeMeanAndStd(D, s, e);
		Dp[i] = (D[i] - ms.first)/ms.second;
	}
	return Dp;
}

std::vector<std::vector<double> > BestMatchSeqSLAM::BuildDifferenceMatrix(ParseSurvey ps1, ParseSurvey ps2){
	int max1 = ps1.NumPoses()/CAM_SKIP;
	int max2 = ps2.NumPoses()/CAM_SKIP;
    
	std::vector<std::vector<double> > difference_matrix(max1, std::vector<double>(max2, 0.0));
    
	for(int i=0; i<max1; i=i+CAM_SKIP){
        std::string fim1 = ParseSurvey::GetImagePath(ps1._base, ps1.GetImageNumber(i));
		cv::Mat Im1 = ImageOperations::Load(fim1);
		cv::Mat pn1 = PatchNormalizedImage(Im1);
		std::vector<double> difference_vector(max2, 0.0);
        
		for(int j=0; j<max2; j=j+CAM_SKIP){
            std::string fim2 = ParseSurvey::GetImagePath(ps2._base, ps2.GetImageNumber(j));
			cv::Mat Im2 = ImageOperations::Load(fim2);
			cv::Mat pn2 = PatchNormalizedImage(Im2);

			//compare
			difference_vector[j] = SumOfAbsDifference(pn1, pn2);
		}

		difference_vector = LocalConstrastEnhancement(difference_vector);
		difference_matrix.push_back(difference_vector);
	}
	return difference_matrix;
}

std::vector<int> BestMatchSeqSLAM::FindTopRankedMatches(std::vector<std::vector<double> >& dm){
	std::vector<int> maxes(dm.size(), 0);
	for(int i=0; i<(int)dm.size(); i++){
		maxes[i] = std::distance(std::begin(dm[i]), std::max_element(std::begin(dm[i]), std::end(dm[i])));
	}
	return maxes;
}

std::vector<int> BestMatchSeqSLAM::CountBestMatchCandidates(int idx, int n, int m, std::vector<int>& topmatches){
	//assume measurements are off the diagonal.

	//estimate two lines. (0,0) -> (m,n)
	int x = idx;
	int y = topmatches[idx];
	double dangle = std::atan2(n, m);
	double mb = m * std::cos(dangle - theta);
	double nb = n * std::sin(dangle-theta);
	double ab = (nb-y)/(mb-x);
	double bb = y -  ab* x;

	double mt = m * std::cos(dangle+theta);
	double nt = n * std::sin(dangle+theta);
	double at = (nt-y)/(mt-x);
	double bt = y - at * x;

	std::vector<int> correct;
	int num_to_check = std::max((int)topmatches.size(), n+idx+1) - (idx+1);
	for(int i=idx+1; i<topmatches.size() && i<n+idx+1; i++){
		int xp = i;
		int yp = topmatches[i];
		int yb_bound = ab*xp + bb;
		if(yb_bound > yp) continue;
		int yt_bound = at*xp + bt;
		if(yt_bound < yp) continue;
		correct.push_back(i);
	}

	if(1.0*num_to_check/correct.size() <= delta) correct.clear();
	correct.push_back(idx);
	return correct;
}

std::vector<int> BestMatchSeqSLAM::FillInBestMatches(std::vector<bool>& usable, std::vector<int>& topmatches){
	std::vector<int> best_matches(topmatches.size(), -1);
	int last_good = 0;
	std::vector<int> to_interp;
	for(int i=0; i<best_matches.size(); i++){
		if(usable[i]) {
			double inc = (1.0*topmatches[i] - topmatches[last_good])/(to_interp.size()+1);
			for(int j=1; j<to_interp.size()+1; j++){
				best_matches[i-to_interp.size()] = topmatches[last_good] + inc * j;
			}
			best_matches[i] = topmatches[i];
			last_good = i;
		}
	}
	for(int j=0; j<to_interp.size(); j++){
		int idx = best_matches.size()-to_interp.size()+j;
		best_matches[idx] = topmatches[last_good];
	}
    return best_matches;
}

std::vector<std::vector<std::string> > BestMatchSeqSLAM::GetMatchedImageList(ParseSurvey& ps1, ParseSurvey& ps2, std::vector<int>& best_matches){
	std::vector<std::vector<std::string> > ret;
	for(int i=0; i<best_matches.size(); i++){
        std::string fim1 = ParseSurvey::GetImagePath(ps1._base, ps1.GetImageNumber(i*5));
        std::string fim2 = ParseSurvey::GetImagePath(ps2._base, ps2.GetImageNumber(best_matches[i]*5));
		ret.push_back({fim1, fim2});
	}
	return ret;
}

std::vector<std::vector<std::string> > BestMatchSeqSLAM::InterSurveyAlignment(){
	//Note: the set of images to use may be different
	//i.e., I may want to use only the set that's identified in the aux files as being part of the optimized set.
	//e.g., run optimization on all those surveys.
	ParseBoatSurvey ps1(_query_loc, _pftbase, _date1); //pftbase is wrong here, but it's unused.
	ParseBoatSurvey ps2(_query_loc, _pftbase, _date2); //pftbase is wrong here, but it's unused.
    
	std::vector<std::vector<double> > dm = BuildDifferenceMatrix(ps1, ps2);
	std::vector<int> topmatches = FindTopRankedMatches(dm);

	//identify usable best matches.
	std::vector<bool> usable(dm.size(), false);
	for(int i=0; i<topmatches.size(); i++){
		std::vector<int> topusable = CountBestMatchCandidates(i, dm.size(), dm[0].size(), topmatches);
		for(int j=0; j<topusable.size(); j++){
			usable[topusable[j]] = true;
		}
		if(topusable.size()>2) i+=topusable[topusable.size()-2]-1;
	}

	//interpolate the unusable matches
	std::vector<int> best_matches = FillInBestMatches(usable, topmatches);

	//return the image paths that correspond to each image.
	return GetMatchedImageList(ps1, ps2, best_matches);
}

bool BestMatchSeqSLAM::InsideImage(cv::Point2f p, int width, int height){
	return p.x >= 0 && p.x < width && p.y >= 0 && p.y < height;
}

BestMatchSeqSLAM::match_vector BestMatchSeqSLAM::GenerateRandomVector(int width, int height){
    match_vector mv = {0, cv::Point2f(0,0), cv::Point2f(-1,-1)};
    int h = f*std::min(width, height)/2;
	mv.angle = 2 * M_PI * rand() / (1.0+RAND_MAX);
	bool invalid = true;
	while(invalid){
		mv.origin.x = width * rand() / (1.0+RAND_MAX);
		mv.origin.y = height * rand() / (1.0+RAND_MAX);
		mv.end.x = mv.origin.x + z * cos(mv.angle);
		mv.end.y = mv.origin.y + z * sin(mv.angle);
        invalid = !InsideImage(cv::Point2f(mv.origin.x - h, mv.origin.y - h), width, height);
		invalid |= !InsideImage(cv::Point2f(mv.origin.x + h, mv.origin.y + h), width, height);
		invalid |= !InsideImage(cv::Point2f(mv.end.x - h, mv.end.y - h), width, height);
		invalid |= !InsideImage(cv::Point2f(mv.end.x + h, mv.end.y + h), width, height);
	}
	return mv;
}

std::vector<std::vector<std::vector<double> > > BestMatchSeqSLAM::GetDifferenceMatrices(match_vector mv, cv::Mat pn1, cv::Mat pn2){
	int hypspace = f*std::min(pn2.rows, pn2.cols);
	std::vector<std::vector<std::vector<double> > > difference_vectors(q, std::vector<std::vector<double> >(hypspace-s, std::vector<double>(hypspace-s, 0)));
	double inc = z/(q-1.0);
	for(int i=0; i<q; i++){
		double cx = mv.origin.x + inc*i * cos(mv.angle);
		double cy = mv.origin.y + inc*i * sin(mv.angle);
		cv::Rect r1(cx-s/2, cy-s/2, s, s);
		cv::Mat pn1small = pn1(r1);
		std::vector<std::vector<double> > difference_vector(hypspace-s, std::vector<double>(hypspace-s, 0));
		for(int j=0; j<hypspace-s; j++){
			for(int k=0; k<hypspace-s; k++){
				cv::Rect r2(cx-hypspace/2+k, cy-hypspace/2+j, s, s);
				cv::Mat pn2small = pn2(r2);
				difference_vectors[i][j][k] = SumOfAbsDifference(pn1small, pn2small);
			}
		}
	}
	return difference_vectors;
}

std::vector<cv::Point2d> BestMatchSeqSLAM::GenerateMatchVector(int idx1, int idx2, cv::Point2d p1, cv::Point2d p2){
    std::vector<cv::Point2d> mvec(q, cv::Point2d(0,0));
    mvec[idx1] = p1;
    mvec[idx2] = p2;
    double deltax = (p2.x-p1.x)/(idx2-idx1);
    double deltay = (p2.y-p1.y)/(idx2-idx1);

    for(int i=0;i<q;i++)
    {
        mvec[i].x = mvec[idx1].x + deltax*(i-idx1);
        mvec[i].y = mvec[idx1].y + deltay*(i-idx1);
    }
    return mvec;
}

std::vector<cv::Point2d> BestMatchSeqSLAM::GetBestMatchVector(std::vector<std::vector<std::vector<double> > >& difference_vectors){
	//select a random pair of best matches, find the score if the matching function was contiguous in delta,
	//iterate and take the combination with the lowest score. (better. I should consider implementing this
	//to see how it compares.)
	std::vector<cv::Point2d> best_matches(q);
	for(int i=0; i<difference_vectors.size(); i++){
		double mind = 100000000;
		for(int j=0; j<difference_vectors[0].size(); j++){
			for(int k=0; k<difference_vectors[0][0].size(); k++){
				if(mind > difference_vectors[i][j][k]){
					mind = difference_vectors[i][j][k];
                    best_matches[i] = cv::Point2d(k,j);
				}
			}
		}
	}

    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0,q-1);
    std::vector<cv::Point2d> min_mvec;
    double minv=10000000000;
	//generate NumGen lines across the best matches, get their score, and save the best line.
	for(int i=0; i<NumGen; i++){
        int idx1 = dist(rng);
        int idx2 = idx1;
        while(idx1 == idx2) idx2 = dist(rng);
        std::vector<cv::Point2d> mvec = GenerateMatchVector(idx1, idx2, best_matches[idx1], best_matches[idx2]);
        double sum = 0;
        for(int j=0; j<mvec.size(); j++){
            sum += difference_vectors[j][mvec[j].y][mvec[j].x];
        }
        if(sum<minv){
            minv = sum;
            min_mvec = mvec;
        }
	}

    return min_mvec;
}

cv::Point2d BestMatchSeqSLAM::GetMatch(std::vector<std::vector<std::vector<double> > > difference_vectors){
	//The approach described in the paper, which is suboptimal. It assumes the points are distributed
	//equally across z. (an inaccurate assumption).
	std::vector<std::vector<double> > cum_difference(difference_vectors[0].size(), std::vector<double>(difference_vectors[0][0].size(), 0));
	for(int i=0; i<difference_vectors.size(); i++){
		for(int j=0; j<difference_vectors[0].size(); j++){
			for(int k=0; k<difference_vectors[0][0].size(); k++){
				cum_difference[j][k] += difference_vectors[i][j][k];
			}
		}
	}

    cv::Point2d minp(0,0);
	double minval = 1000000000;
	for(int j=0; j<difference_vectors[0].size(); j++){
		for(int k=0; k<difference_vectors[0][0].size(); k++){
			if(minval > cum_difference[j][k]){
				minval = cum_difference[j][k];
				minp = cv::Point2d(k,j);
			}
		}
	}
	return minp;
}

void BestMatchSeqSLAM::AddSeqToCorrespondences(correspondences& cs, std::vector<cv::Point2d> ps, match_vector mv, int height, int width){
    int hypspace = f*std::min(height, width);
    double inc = z/(q-1.0);
    for(int i=0; i<ps.size(); i++){
        int cx = mv.origin.x + inc*i * cos(mv.angle);
        int cy = mv.origin.y + inc*i * sin(mv.angle);
        cs.pref.push_back(cv::Point2d(cx, cy));
        cv::Point2d off = cv::Point2d(ps[i].x-(hypspace-s)/2, ps[i].y-(hypspace-s)/2);
        cs.pnew.push_back(cv::Point2d(cx+off.x, cy+off.y));
    }
}

void BestMatchSeqSLAM::AddToCorrespondences(correspondences& cs, cv::Point2d p, match_vector mv, int height, int width){
	int hypspace = f*std::min(height, width);
	cv::Point2d off = cv::Point2d(p.x-(hypspace-s)/2, p.y-(hypspace-s)/2);
	double inc = z/(q-1.0);
	for(int i=0; i<q; i++){
		int cx = mv.origin.x + inc*i * cos(mv.angle);
		int cy = mv.origin.y + inc*i * sin(mv.angle);
		cs.pref.push_back(cv::Point2d(cx, cy));
		cs.pnew.push_back(cv::Point2d(cx+off.x, cy+off.y));
	}
}

cv::Mat BestMatchSeqSLAM::FlowFieldFromHomography(cv::Mat homography, int height, int width){
	//TODO: check that the flow field is consistent with the one ouput by SFlowDREAM.
	cv::Mat flow(height, width, CV_64FC2, cv::Scalar::all(0));
	double * fmap = (double *) flow.data;
	for(int i=0; i<height; i++){
		for(int j=0; j<width; j++){
			cv::Mat pix = (cv::Mat_<double>(3,1) << j, i, 1);
			cv::Mat res = homography * pix;
            double * pres = (double*)res.data;
			int x = pres[0]/pres[2];
			int y = pres[1]/pres[2];
			if(x <0 || x >= width || y <0 || y >= height) continue;
			int idx = i * width + j;
			fmap[idx*2] = x;
			fmap[idx*2+1] = y;
		}
	}
	return flow;
}

AlignmentResult BestMatchSeqSLAM::IntraSurveyAlignment(std::string fim1, std::string fim2){
	cv::Mat Im1 = ImageOperations::Load(fim1);
	cv::Mat pn1 = PatchNormalizedImage(Im1);
	cv::Mat Im2 = ImageOperations::Load(fim2);
	cv::Mat pn2 = PatchNormalizedImage(Im2);

	correspondences cs;
	while(cs.pref.size() < 50){
		//generate a vector of length z in a random direction
		match_vector mv = GenerateRandomVector(pn1.cols, pn1.rows);

		std::vector<std::vector<std::vector<double> > > difference_vectors = GetDifferenceMatrices(mv, pn1, pn2);

		cv::Point2d bestp = GetMatch(difference_vectors);

		AddToCorrespondences(cs, bestp, mv, pn1.rows, pn1.cols);
	}

	//generate homography from the correspondences.
	cv::Mat homography = findHomography(cs.pref, cs.pnew);

	//get the flow field.
	cv::Mat flow = FlowFieldFromHomography(homography, pn1.rows, pn1.cols);

	AlignmentResult ar(pn1.rows, pn1.cols);
	ar.SetReferenceImage(Im1.data);
	ar.SetSecondImage(Im2.data);
	ar.SetFlow((double*)flow.data);

	return ar;
}

















































