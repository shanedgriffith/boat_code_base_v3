/*
 * SFlowDREAM.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: shane
 */

#include "SFlowDREAM.hpp"

#include <ctime>
#include <sys/time.h>

using namespace cv;
using namespace std;

void SFlowDREAM::SetHeterogeneousHypothesisSpace(BPFlow& bpflow, SIFTImageLayer& il, int imset){
    std::cout << "SFlowDREAM::SetHeterogeneousHypothesisSpace() Not implemented." << std::endl;
    exit(-1);
}

void SFlowDREAM::ApplyRFlowConstraints(BPFlow& bpflow, SIFTImageLayer& il, double scale, int imset){
	bpflow.UseRConstraints();
	int nc = rf->GetNumberOfConstraints(imset);
	rf->OutputSize(il.width(), il.height());
	for(int i=0; i<nc; i++){
		vector<double> c = rf->GetConstraint(imset, i);
		bpflow.AddConstraint(round(c[0]), round(c[1]), c[2], c[3], c[4]/scale);
	}
}

cv::Mat SFlowDREAM::AddOffset(cv::Mat im, int x, int y){
    //uses a hard offset of 1,1
    Mat imoff(im.rows,im.cols,CV_8UC(im.channels()), Scalar::all(0));
    Rect prev(0,0,im.cols-x,im.rows-y);
    Rect test(x,y,im.cols-x,im.rows-y);
    im(prev).copyTo(imoff(test));
    Rect row(0,0,im.cols,y);
    Rect col(0,0,x,im.rows);
    cv::Mat tempr, tempc;
    im(row).copyTo(tempr);
    cv::flip(im(row), tempr, 0);
    cv::flip(im(col), tempc, 1);
    tempr.copyTo(imoff(row));
    tempc.copyTo(imoff(col));
    return imoff;
}

double SFlowDREAM::ComputeVerificationRatio(SIFTImageLayer& sil, SIFTImageLayer& woff, int x, int y) {
    //assumes a hard offset of 1,1
    //rf[i,j] + (1,1) =?= wrf[i,j]
    int height = sil.rows;
    int width = sil.cols;
    double * rf = (double *) sil.f.data;
    double * wrf = (double *) woff.f.data;
    int count = 0;
    int numzeros = 0;
    for(int i=0; i<height-y; i++) {
        for(int j=0; j<width-x; j++) {
            count++;
            int idx1 = (i*width + j)*2;
            int idx2 = ((i+y)*width + (j+x))*2;
            if(rf[idx1]+x != wrf[idx2] || rf[idx1+1]+y != wrf[idx2+1]) continue;
            numzeros++;
        }
    }
    if(debug) cout << "VerificationRatio()  " << 1.0*numzeros/count <<endl;
    return 1.0*numzeros/count;
}

void SFlowDREAM::VerifiedBP(SIFTImageLayer& sil, double gamma, int h, int nHierarchy, double scale, int imset, vector<vector<double> >* two_cycle_hypspace){
    //NOTE: this can be parallelized. (may gain up to 0.8 seconds per full alignment attempt)
    RunBP(sil, gamma, h, nHierarchy, scale, imset, two_cycle_hypspace);
    if(reprojection_flow || ranverification || !verifyalignment) return;

    int x=3, y=3;
    SIFTImageLayer woff(sil.im);
    woff.im[1] = AddOffset(woff.im[1], x, y); //put the offset on im2
    RunBP(woff, gamma, h, nHierarchy, scale, imset, two_cycle_hypspace);
    sil.verified_ratio = ComputeVerificationRatio(sil, woff, x, y);
    ranverification = true;
}

void SFlowDREAM::RunBP(SIFTImageLayer& il, double gamma, int h, int nHierarchy, double scale, int imset, vector<vector<double> >* two_cycle_hypspace) {
    //order here is important.
    if(debug) cout << "Run BP for ("<<il.width()<<", "<< il.height()<<")"<<endl;
    std::clock_t start, construct, end;

    //set the offset.
    double * offset = (double *) il.f.data;
    //if reprojection_flow and we're at the top layer.
    if(reprojection_flow && offsets[imset].rows == il.height())
        offset = (double *) offsets[imset].data;

    //set the input and parameters
    BPFlow bpflow;
    bpflow.LoadImages(il.width(), il.height(), il.channels(), il.ImageRefs(imset));
    bpflow.SetOffset(offset);
    bpflow.setPara(alpha,d);
    bpflow.setDataTermTruncation(true);
    bpflow.setHomogeneousMRF(h);
    if(reprojection_flow && il.width() == 44){
        SetHeterogeneousHypothesisSpace(bpflow, il, imset);
    }

    //set the constraints
    if(twocycle && two_cycle_hypspace != NULL) bpflow.AddHypSpaceWeights(two_cycle_hypspace);
    if(epipolar && hypspace.size() > imset) bpflow.AddEpiWeights(&hypspace[imset]);
    if(reprojection_flow && il.width() < 320) {
    	//Only set reprojection flow constraints in the higher layers of the pyramid.
    	ApplyRFlowConstraints(bpflow, il, scale, imset);
    }

    start = std::clock();

    //construct the MRF
    bpflow.ComputeDataTerm();
    bpflow.ComputeRangeTerm(gamma);

    construct = std::clock();

    //perform belief propagation
    if(debug) bpflow.setDisplay(true);
    bpflow.MessagePassing(nIterations, nHierarchy, NULL);
    end = std::clock();

    //get results
    bpflow.ComputeVelocity((double*) il.f.data);
    bpflow.GetEnergyImage((double*) il.nrg.data);
    double e_a = bpflow.GetEnergy(true);
    double e_d = bpflow.GetEnergy(false);
    il.SetAlignmentEnergy(e_a, e_d);

    if(debug) {
        cout << "mrf construction time: " << (construct - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << endl;
        cout << "inference time: " << (end - construct) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << endl;
        cout << "energy: " << e_a << endl;
    }
}


void SFlowDREAM::ComputeCycleConsistencyWeights(vector<vector<double> >& hypspace, SIFTImageLayer& il, int h) {
	int winsize = 2*h+1;
	double * f  = (double *) il.f.data;

	//compute average flow.
	double av_x=0.0, av_y=0.0;
	for(int i=0; i<il.rows; i++){
		for(int j=0; j<il.cols; j++){
			int idx = (i*il.cols + j)*2;
			av_x += f[idx];
			av_y += f[idx+1];
		}
	}
	av_x /= il.rows * il.cols;
	av_y /= il.rows * il.cols;

	//assign the flow weights.
	for(int i=0; i<il.rows; i++){
		for(int j=0; j<il.cols; j++){
			int idx = i*il.cols + j;

			for(int k=-h; k<=h; k++){
				for(int l=-h; l<=h; l++){
					int hyp = (k+h)*winsize+(l+h);
					int sp_x = j+l;
					int sp_y = i+k;
					int sp = (sp_y*il.cols + sp_x)*2;

					//use the average flow to get a default score if it is otherwise unknown.
					int ret_x = av_x + sp_x;
					int ret_y = av_y + sp_y;
					//the flow is known.
					if(sp_y < il.rows && sp_x < il.cols && sp_y >= 0 && sp_x >= 0){
						ret_x = sp_x+f[sp];
						ret_y = sp_y+f[sp+1];
					}

					double dist = _cycle_weight*pow(pow(j - ret_x, 2.0) + pow(i - ret_y, 2.0), 0.5);
					hypspace[idx][hyp] = dist;
				}
			}

		}
	}
}

void PrintFlow(SIFTImageLayer& il) {
    double * f  = (double *) il.f.data;
    std::cout<<"Flow result---------------------------------------------------------\n";
    for(int i=0; i<il.rows; i++){
        for(int j=0; j<il.cols; j++){
            int idx = (i*il.cols + j)*2;
            std::cout<<f[idx]<<","<<f[idx+1]<<"|";
        }
        std::cout<<std::endl;
    }
}

double SFlowDREAM::MeasureConsistency(vector<SIFTImageLayer>& vec, int ref){
	int width = vec[ref].cols;
	int height = vec[ref].rows;

	double * rf = (double *) vec[ref].f.data;
	double sum=0.0;
	int count = 0;
    int numzeros = 0;
    int numbig = 0;
    double sumx=0.0, sumy=0.0;
	for(int i=0; i<height; i++){
		for(int j=0; j<width; j++){
			int idx = (i*width + j)*2;

			sumx += rf[idx];
			sumy += rf[idx+1];

			int sp_x = j + rf[idx];
			int sp_y = i + rf[idx+1];
			int sp = (sp_y*width + sp_x)*2;
			if(sp_y >= height || sp_x >= width || sp_y < 0 || sp_x < 0) continue;
			for(int k=0; k<vec.size(); k++){
				if(k==ref) continue;
				double * f = (double *) vec[k].f.data;
				int back_x = sp_x + f[sp];
				int back_y = sp_y + f[sp+1];
				count++;
				int dist = pow(pow(back_x-j, 2.0) + pow(back_y-i,2.0), 0.5);
				if(dist <= 1) numzeros++;
				if(dist > 5) numbig++;
				sum += dist;
			}
		}
	}
	if(debug) cout << "MeasureConsistency() percent big: " << 1.0*numbig/count << ", percent zeros:  " << 1.0*numzeros/count << ", dist: " << sum/count << ", avg: ("<<sumx/(height*width)<<","<<sumy/(height*width)<<")" <<endl;
	return 1.0*numzeros/count;
}


/*Computes epipolar constraints from the set of consistent pixels.
 * */
bool SFlowDREAM::ComputeEpipolarWeights(SIFTImageLayer& ref, SIFTImageLayer& comp, int imset, int h){
	int width = ref.cols;
	int height = ref.rows;
    bool print = false;

	double * rf = (double *) ref.f.data;
	double * f = (double *) comp.f.data;


	if(consistent_set.rows==0){
		//one channel for each set.
		consistent_set = cv::Mat(height, width, CV_8UC2, cv::Scalar::all(0));
	}

	//for storing the set of consistent pixels.
	vector<cv::Point2f> orig_sparse;
	vector<cv::Point2f> mapped_sparse;

	//get the set of consistent pixels.
	for(int i=0; i<height; i++){
		for(int j=0; j<width; j++){
			int idx = (i*width + j)*2;

			int sp_x = j + rf[idx];
			int sp_y = i + rf[idx+1];
			int sp = (sp_y*width + sp_x)*2;
			if(sp_y >= height || sp_x >= width || sp_y < 0 || sp_x < 0) {
				continue;
			}

			int back_x = sp_x + f[sp];
			int back_y = sp_y + f[sp+1];

			int dist = pow(pow(back_x-j, 2.0) + pow(back_y-i,2.0), 0.5);
			bool val = false;
			if(dist == 0) {
			    val = true;
				orig_sparse.push_back(cv::Point2f(j,i));
				mapped_sparse.push_back(cv::Point2f(sp_x, sp_y));
			}

			//the imset values here are the channel value.
            ((unsigned char *) consistent_set.data)[idx + imset] = val;
            ((unsigned char *) consistent_set.data)[sp+(imset+1) % 2] = val;
			if(print) cout << ", " << ((unsigned char *) consistent_set.data)[idx + imset];
		}
		if(print) cout << endl;
	}

	cv::Mat off;
	if(imset < offsets.size()) off = offsets[imset];
	RFlowFME fme(_cam, width, height);
	fme.setSTD(epi_std);
	return fme.IdentifyHypothesisSpace(hypspace[imset], orig_sparse, mapped_sparse, off, h);
}


void SFlowDREAM::AdaptiveOffsetConstraint(int height, int width){
	/*Use RF to 1) center the hypothesis space around the correct alignment; and 2) size the hypothesis space.
	 *
	 * Generate an offset image that's in the middle of min and max of the constraints, then set h
	 * to be HYP_SPACE_PADDING outside of this range.
	 * */
	int hmax = 0;
	rf->OutputSize(width, height);
	for(int i=0; i<2; i++){
		vector<double> mcur = rf->GetConstraintBounds(i);
		offsets.push_back(cv::Mat(height, width, CV_64FC2, cv::Scalar(mcur[2], mcur[5])));

		//size the hypothesis space.
		int hx = max((double) abs(mcur[1]-mcur[2]), (double) abs(mcur[2]-mcur[0]));
		int hy = max((double) abs(mcur[4]-mcur[5]), (double) abs(mcur[5]-mcur[3]));
		hmax = max(hmax, max(hx, hy));
		if(debug)
            cout << "Image Alignment " << i<< " offset: ["<<mcur[2] << "," <<mcur[5] << "] bounds: [" << hx <<", " << hy << "] actual bound: " << hmax << " + " << HYP_SPACE_PADDING << endl;
	}

	//set the hypothesis space size
    topwsize = hmax + HYP_SPACE_PADDING;
}

void SFlowDREAM::CyclicAlignment(SIFTImageLayer& sil, double g, int h, int nHierarchy){
	/*Iteratively aligns the image pair in the forward and reverse directions until the alignment consistency is high*/
	int imset = 0;
	int winsize = h*2+1;
	vector<vector<double> > two_cycle_hypspace = vector<vector<double> >(sil.height()*sil.width(), vector<double>(winsize*winsize, 0.0));
	vector<SIFTImageLayer> res;
	for(int i=0;i<2;i++) res.push_back(SIFTImageLayer(sil.height(), sil.width()));
	SIFTImageLayer best = SIFTImageLayer(sil.height(), sil.width());
	if(hypspace.size()==0 && epipolar) hypspace = vector<vector<vector<double> > >(2, vector<vector<double> >(sil.height()*sil.width(), vector<double>(winsize*winsize, 0.0)));

	for(; cycle_iter<two_cycle_iterations; cycle_iter++){
		if(debug) cout << "iter " << cycle_iter<< endl;
		VerifiedBP(sil, g, h, nHierarchy, ip.layers[0].width()/sil.width(), imset, &two_cycle_hypspace);

		res[imset].CopyResult(sil);
		if(cycle_iter > 0) res[0].consistency = MeasureConsistency(res, 0);
		if(res[0].consistency > best.consistency) best.CopyResult(res[0]);
		if(debug) cout << ""<<cycle_iter<< "|"<<imset << ", energy: " <<res[imset].alignment_energy << ", consistency: " << res[0].consistency << endl;

		//stop looping if any flow is consistent with all the other flows.
		if(res[0].consistency > consistency_threshold)  {
			if(debug) cout << "break due to consistency success: "<<res[0].consistency << " is above " << consistency_threshold << endl;
			break;
		}
		if(verifyalignment && res[0].verified_ratio < verification_threshold) {
		    if(debug) cout << "break due to verification ratio fail: "<<res[0].verified_ratio << " is below " << verification_threshold << endl;
            break;
		}

		//two-cycle consistency constraints
		ComputeCycleConsistencyWeights(two_cycle_hypspace, res[imset], h);

		//epipolar constraints
		if(cycle_iter>0 && !reprojection_flow && epipolar){
			//Compute the epipolar constraints from the set of consistent pixels.
			//The constraints are in the form of hypspace weights for the next iteration.
			bool haveconsistent = ComputeEpipolarWeights(res[(imset+1)%2], res[imset], imset, h);
			if(!haveconsistent) break;
		}

		imset = (imset+1)%2;
	}

	sil.CopyResult(best);
}

void SFlowDREAM::EpipolarConstraintsFromRF(){
	RFlowFME fme(_cam, rf->outw, rf->outh);
	fme.setSTD(epi_std);
	for(int j=0; j<2; j++){
		vector<cv::Point2f> orig_sparse = rf->GetRestrictedSetOrig(j);
		vector<cv::Point2f> mapped_sparse = rf->GetRestrictedSetMapped(j);
		bool suc = fme.IdentifyHypothesisSpace(hypspace[j], orig_sparse, mapped_sparse, offsets[j], topwsize);
		if(!suc){
			std::cout << "SFlowDREAM::EpipolarConstraintsFromRF() Error: Failed to generate epipolar constraints from RF points. Unexpected behavior." << std::endl;
			exit(-1);
		}
	}
}

void SFlowDREAM::AlignImages(){
	/*With each image layer, the parameters change as follows:
	 * nhierarchy: 2, 1, 2, 3. (specifies a within-bpflow matching hierarchy)
	 * window size: 11, 3, 2, 1. (The hypothesis space around a pixel. Square size: (2*h+1)^2)
     */
	if(ip._nlayers == 0){
		cout << "SFlowDREAM::AlignImages() Error: Define the image pyramid first." << endl;
		exit(-1);
	}
	if(reprojection_flow && rf == NULL){
		cout << "Reprojection flow is set, but the map wasn't specified. Unsetting." << endl;
		reprojection_flow = false;
	}

	struct timeval start, end;
	gettimeofday(&start, NULL);

    

    if(reprojection_flow) {
        if(debug) cout << "SFlowDREAM::AlignImages() using RF" << endl;
		SIFTImageLayer& sil = ip.layers[ip.TopLayer()];

	    //adaptive offset and window size constraints from RF.
    	//NOTE: This constraint is always applied if RF is used.
    	AdaptiveOffsetConstraint(sil.height(), sil.width());

    	//epipolar constraints from RF.
    	int winsize = topwsize*2+1;
    	hypspace = vector<vector<vector<double> > >(2, vector<vector<double> >(sil.height()*sil.width(), vector<double>(winsize*winsize, 0.0)));
    	EpipolarConstraintsFromRF();
    } else if(debug) cout << "SFlowDREAM::AlignImages() without RF" << endl;

	//align images from the top of the image pyramid to the bottom.
    for(int i=ip.start_layer; i>=ip.stop_layer; i--) {
        int num_imsets = (twocycle && cycle_down_to_layer <= i)?2:1; //  || reprojection_flow)
        double g = gamma*pow(2,i-1);
        int dubmult = topwsize;
        int nHierarchy=2;

        if(i!=ip.TopLayer()){
            Mat doubled = ip.layers[i+1].DoubleFlow();
            ip.layers[i].SetFlow((double *) doubled.data);
            dubmult = i;
            nHierarchy = ip.TopLayer()-i;
            epi_std = 10.0/dubmult;

            if(epipolar){
            	//compute the epipolar constraint from the previous flow result
                int winsize = dubmult*2+1;
				hypspace = vector<vector<vector<double> > >(num_imsets, vector<vector<double> >(ip.layers[i].height()*ip.layers[i].width(), vector<double>(winsize*winsize, 0.0)));
				RFlowFME fme(_cam, ip.layers[i].width(), ip.layers[i].height());
                fme.setSTD(epi_std);//if not top layer?
				fme.IdentifyHypothesisSpace(hypspace[0], ip.layers[i].f, dubmult);
            }
        }

        if(num_imsets < 2) VerifiedBP(ip.layers[i], g, dubmult, nHierarchy, ip.layers[0].width()/ip.layers[i].width());
        else CyclicAlignment(ip.layers[i], g, dubmult, nHierarchy);

        term_layer = i;
    	if(i < ip.start_layer)  {
    	    ip.layers[i].consistency = ip.layers[i+1].consistency;
    	    ip.layers[i].verified_ratio = ip.layers[i+1].verified_ratio;
    	}
        
    	if(mandatory_consistency_condition && ip.layers[i].consistency < consistency_threshold)  break;
    	if(verifyalignment && ip.layers[ip.start_layer].verified_ratio < verification_threshold) break;
    }

	gettimeofday(&end, NULL);
	computation_time = (((end.tv_sec - start.tv_sec) * 1000000) + (end.tv_usec - start.tv_usec))/1000000.0;
}

AlignmentResult SFlowDREAM::GetAlignmentResult(){
	if(computation_time == 0) {
		cout << "No AlignmentResult. Align the images first." << endl;
		exit(-1);
	}
	int l = (term_layer == stop_layer)?res_layer:term_layer;
	AlignmentResult res = ip.GetAlignmentResult(l);
	res.SetComputationTime( computation_time );
	if(consistent_set.rows > 0) {
	    //std::cout << "consistent_set sizes: " << consistent_set.rows << ", "<<consistent_set.cols<<", "<<consistent_set.channels()<<std::endl;
	    res.SetConsistentSet(consistent_set.data, consistent_set.rows, consistent_set.cols);
	}
	res.alignment_energy_lowres = ip.layers[term_layer].alignment_energy;
    res.verified_ratio = ip.layers[term_layer].verified_ratio;
    res.consistency = ip.layers[term_layer].consistency;
	return res;
}

void SFlowDREAM::SetDryRun(bool b){
	if(b) {
		stop_layer = nlayers-1;
		res_layer = nlayers-1;
		ip.SetStopLayer(stop_layer);
	} else {
		stop_layer =     1;
		res_layer  =     0;
		ip.SetStopLayer(stop_layer);
	}
}

void SFlowDREAM::SetVerifyAlignment(bool set){
    verifyalignment = set;
    if(reprojection_flow){
        std::cout << "SFlowDREAM::SetVerifyAlignment() Can't verify alignment quality if RF is used." << std::endl;
        exit(1);
    }
}

void SFlowDREAM::SetTwoCycleConsistency(bool set){
	twocycle = set;
}

void SFlowDREAM::SetLastCycleConsistencyLayer(int l){
	/*By default, two-cycle consistency is only run on the top layer when it is enabled.
	 * This option allows it to be run at lower levels of the image pyramid.
	 * */
	cout << "Not currently supported. The ImagePyramid only stores the flow result for one flow. The cycle consistency needs to be rigorous before that." << endl;
	exit(-1);
//	cycle_down_to_layer = l;
}

void SFlowDREAM::ContinueOnConsistency(bool set){
	/*Check the top level alignment for consistency.
	 * Proceed with the full resolution alignment if it's consistent.
	 * */
	mandatory_consistency_condition = set;
}

void SFlowDREAM::SetEpipolar(bool set){
	epipolar = set;
}

bool SFlowDREAM::SetReprojectionFlow(ReprojectionFlow* rflow){
	if(!rflow->HaveTwoSets()){
		return false;
	}
	reprojection_flow = true;
	rf = rflow;
	verifyalignment = false;
	return true;
}


void SFlowDREAM::ConstructImagePyramid(string fim1, string fim2){
    //NOTE: previously applied undistort here, but that creates border effects, which would affect the alignment.
    //      Undistort is only needed for computing the fundamental or the essential matrix. 
    //
    //Input: Camera& _cam,
//    ImageOperations::Undistort(_cam, Im1); //DON'T DO THIS (see above). actually, maybe I should. Either way, the border effect will have an effect on the alignment.
//    ImageOperations::Undistort(_cam, Im2);
    Mat Im1 = ImageOperations::Load(fim1);
    Mat Im2 = ImageOperations::Load(fim2);
    ConstructImagePyramid(Im1, Im2);
}


void SFlowDREAM::ConstructImagePyramid(cv::Mat& im1, cv::Mat& im2){
	vector<cv::Mat> ims = {im1, im2};
	ip = ImagePyramid(nlayers, stop_layer, ims);
}


void SFlowDREAM::SetImagePyramid(ImagePyramid& pyr){
	ip = pyr;
}


void SFlowDREAM::SetTopWsize(int t){
	topwsize = t;
}

void SFlowDREAM::Reset(){
	alpha=      1.0*255;
	d=          40*255;
	gamma=      0.001*255;
	topwsize=   11;
	nIterations=100;

	nlayers	   =     5;
	stop_layer =     1;
	res_layer  =     0;
	term_layer =     5;

	debug = false;
	timed = false;

	ip = ImagePyramid();

	verifyalignment = false;
	ranverification = false;
	verification_threshold = 0.4;
	twocycle = false;
	two_cycle_iterations = 19;
	cycle_down_to_layer = 4;
	_cycle_weight = 16;
	consistency_threshold = 0.95;
	mandatory_consistency_condition = false;

	epipolar = false;
	hypspace = {};

	reprojection_flow = false;
	rf = {};

	computation_time = 0.0;
}















