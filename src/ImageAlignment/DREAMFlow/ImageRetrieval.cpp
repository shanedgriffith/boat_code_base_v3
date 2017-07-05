/*
 * ImageRetrieval.cpp
 *
 *  Created on: Aug 19, 2016
 *      Author: shane
 */


#include "ImageRetrieval.hpp"
#include <ImageAlignment/DREAMFlow/SFlowDREAM.hpp>
#include <DataTypes/AlignmentResult.h>
#include <FileParsing/ParseSurvey.h>
#include <math.h>


using namespace std;

void ImageRetrieval::InitializeMachine(int nthreads){
    if(nthreads>1){
        man.check_users = true;
        for(int i=0; i<nthreads; i++){
            ws.push_back(new IRMachine());
            ws[i]->SetCamera(_cam);
            man.AddMachine(ws[i]);
        }
    }
}

/*The following are from ICRAAlignmentMachine for image retrieval and the first alignment using SIFT Flow*/
bool ImageRetrieval::DistanceCriterion(std::vector<double>& pose1, std::vector<double>& pose2){
    double dist = pow(pow(pose1[0] - pose2[0], 2) + pow(pose1[1] - pose2[1],2), 0.5);
    if (dist > pose_distance_threshold) {
        // Don't try to match points more than 20m away, GPS is not
        // that bad.
        return false;
    }
    if (fabs(remainder(pose1[5]-pose2[5],2*M_PI)) > pose_angle_threshold*M_PI/180) {
        // If we're not looking remotely in the same direction, no
        // point trying to match
        return false;
    }
    return true;
}


std::vector<int> ImageRetrieval::IdentifyNeighborPoses(std::vector<std::vector<double> >& poses, std::vector<int>& imageno, std::vector<double>& p2){
	//return a list of the nearest neighbor poses.
	//how is it returned?
	//  in the order it's found.
	std::vector<int> indices;
	 for(int i=0; i<poses.size(); i++){
	    	if(CAM_SKIP > 0 && imageno[i]%CAM_SKIP != 0) continue;
	    	std::vector<double> p1 = poses[i];
	    	if(!DistanceCriterion(p1, p2)){
	    		if(indices.size() > 5) break;
				else continue;
			}
	    	indices.push_back(i);
	 }
	 return indices;
}


std::vector<double> ImageRetrieval::NaiveSearch(std::string base, std::vector<int>& imageno, std::string image2, std::vector<int>& neighbor_poses){
	SFlowDREAM sf(_cam);
	sf.SetDryRun();
	sf.SetVerifyAlignment();
    std::vector<double> min_vals = {-1, 1000000000};
	for(int i=0; i<neighbor_poses.size(); i++){
		int idx = neighbor_poses[i];

        std::string image1 = ParseSurvey::GetImagePath(base, imageno[idx]);

		sf.ConstructImagePyramid(image1, image2);
		sf.AlignImages();
		AlignmentResult ar = sf.GetAlignmentResult();
		if(ar.alignment_energy < min_vals[1]){
			min_vals[1] = ar.alignment_energy;
			min_vals[0] = i;
		}
		//if(debug) cout << "Aligning " << im1 << ": "<<_ps2->imageno[i]<<" has " << ar.alignment_energy << ". min: " << min_nrg << endl;
	}

    return min_vals;
}


/*Perform image retrieval. Find the best match for image2 in survey 1.
 * @param neighbor_poses The list of poses from survey 1 with a similar GPS location and compass heading.
 * @param: DELTA. Used with the alignment energy to identify when the images have become dissimilar.
 * 				  Experimentally determined.
 * 				  For efficiency purposes and not necessary for the correct function of the algorithm.
 * */
std::vector<double> ImageRetrieval::DirectionalSearch(std::string base, std::vector<int>& imageno, std::string image2, std::vector<int>& neighbor_poses){
	SFlowDREAM sf(_cam);
	sf.SetDryRun();
	sf.SetVerifyAlignment();
	std::vector<double> min_vals = {-1, 1000000000};

    bool left = true, right = true;
    //start the search from the middle, going left and right.
    for(int i=0; i<neighbor_poses.size()/2; i++){

    	int neg = 1;
    	for(int j=0; j<2; j++){
    		if(neg < 0 && !left) continue;
    		if(neg > 0 && !right) continue;
    		int off = neighbor_poses.size()/2 + i*neg;
    		if(off < 0 || off > neighbor_poses.size()-1) continue;
    		int idx = neighbor_poses[off];
            
            std::string image1 = ParseSurvey::GetImagePath(base, imageno[idx]);
    		sf.ConstructImagePyramid(image1, image2);
    		sf.AlignImages();
    		AlignmentResult ar = sf.GetAlignmentResult();
			if(ar.alignment_energy < min_vals[1]){
				min_vals[1] = ar.alignment_energy;
				min_vals[0] = idx;
			}

			if(min_vals[0]!= -1 && ar.alignment_energy-min_vals[1] > DELTA){
				if(neg < 0) left = false;
				else right = false;
			}

			neg *= -1;
    	}
    }

    return min_vals;
}

int ImageRetrieval::IdentifyClosestPose(std::string base, std::vector<std::vector<double> >& poses, std::vector<int>& imageno, std::vector<double> pose2, string image2, double * ae) {
	std::vector<int> neighbor_poses = IdentifyNeighborPoses(poses, imageno, pose2);
    std::vector<double> minvals = {-1, 1000000000};
    if(neighbor_poses.size()>0){
        if(ws.size()>0) minvals = MultiThreadedSearch(base, imageno, image2, neighbor_poses);
        else minvals = DirectionalSearch(base, imageno, image2, neighbor_poses);
    }
	if(ae!=NULL) *ae = minvals[1];
	return minvals[0];
}

int ImageRetrieval::GetMin(std::vector<double>& res){
    //find the minval that is greater than 0
    if(res.size() == 0) return -1;
    double minv = 100000000000;
    int mini = -1;
    for(int i=0; i<res.size(); i++){
        if(res[i]<minv && res[i] > 0){
            minv = res[i];
            mini = i;
        }
    }
    return mini;
}

int ImageRetrieval::GetMax(std::vector<double>& ver){
    if(ver.size() == 0) return -1;
    std::vector<double>::iterator result = std::max_element(ver.begin(), ver.end());
    return  std::distance(ver.begin(), result);
}

bool ImageRetrieval::ContinueInDirection(int off, int direction, std::vector<double>& res){
    //assumes off starts off in bounds.
    int idx = GetMin(res);
    if(idx==-1) return true;
    if(off==0 || off==res.size()) return false; //shouldn't this be for the opposite direction?
    while(res[off]<1) off -= direction; //stop at the middle of res.
    if(res[off]-res[idx]>DELTA) return false;
    return true;
}

bool ImageRetrieval::ArrayMatchesSignal010(std::vector<double>& res){
    //returns false for signals like 0101 and 0000.
    std::vector<bool> signal={false,true,false};
    int idx=0;
    int num_true = 0;
    for(int i=0; i<res.size(); i++){
        bool criteria = res[i]>1;
//        std::cout<<i<<","<<idx<<","<<res[i]<<","<<signal[idx]<<std::endl;
        if(criteria) num_true++;
        if(criteria==signal[idx]) continue;
        else{
            idx++;
            if(idx>=signal.size()) return false;
        }
    }
    if(res.size() > 0 && num_true==0) return false;
    return true;
}

bool ImageRetrieval::HaveVerified(int i, std::vector<double>& ver){
    if(i<ws.size()) return false;
    int vidx = GetMax(ver);
    if(vidx >=0 && ver[vidx] > verification_threshold) {
        return true;
    }
    return false;
}

std::vector<double> ImageRetrieval::MultiThreadedSearch(std::string base, std::vector<int>& imageno, string image2, std::vector<int>& neighbor_poses){
    //Something like 3.2x faster than sequential directional search (when all the threads can be used).
    bool left = true, right = true;
    std::vector<double> res(neighbor_poses.size(), 0);
    std::vector<double> ver(neighbor_poses.size(), 0);
    std::vector<int> sign={1,-1};
    int idx = res.size()/2;
    bool verified = false;
    int vidx = -1;
    for(int i=0; i<res.size(); i++){
        int dir = sign[i&0x1];
        idx += i*dir;

        int tidx = man.GetOpenMachine();

        //check progress to stop if we can.
//        bool verified = HaveVerified(i, ver);
//        if(verified) break;
        
        if(dir<0 && left) left = ContinueInDirection(idx, dir, res);
        else if(dir>0 && right) right = ContinueInDirection(idx, dir, res);
        if(!left && !right) break;
        if(dir<0 && !left) continue;
        else if(dir>0 && !right) continue;
        
        int sidx = neighbor_poses[idx];
        string image1 = ParseSurvey::GetImagePath(base, imageno[sidx]);
        ws[tidx]->Setup(image1, image2, &res[idx], &ver[idx]);
        man.RunMachine(tidx);
    }
    
    //check whether we can stop as soon as each thread finishes.
    if(left || right){//this approach is ~0.5 seconds faster than man.WaitForMachine(true);
        while(man.WaitForAnotherMachine()){
//            while(!verified && man.WaitForAnotherMachine()){
//            verified = HaveVerified(ws.size(), ver);
            if(left) left = ContinueInDirection(0, -1, res); //looks wrong.
            else if(right) right = ContinueInDirection(res.size()-1, 1, res);
            else break;
        }
    }
    
    //given all the boundaries, now check that the points between the boundaries are filled in.
    while(!ArrayMatchesSignal010(res) && man.WaitForAnotherMachine());
//    while(!verified && !HaveVerified(ws.size(),ver) && !ArrayMatchesSignal010(res) && man.WaitForAnotherMachine());
    
    man.TerminateMachines();
    
    int nnz = 0;
    for(int i=0; i<res.size(); i++) if(res[i] > 0) nnz++;
    std::cout << "ImageRetrieval searched " << nnz << " images of " << res.size() << ". Base: " << base << std::endl;

    std::vector<double> min_vals = {-1, 1000000000};
    if(neighbor_poses.size()>0) {
        int idx_max_ver = GetMax(ver); //why not choose the one with the max verification?
        if(ver[idx_max_ver] < verification_threshold) {
            std::cout << "ImageRetrieval::MultiThreadedSearch() Failed alignment verification. Value " << ver[idx_max_ver] << std::endl;
            return {-1, 1000000000};
        }
        else std::cout << "ImageRetrieval::MultiThreadedSearch() Passed alignment verification. Value " << ver[idx_max_ver] << std::endl;
        if(idx_max_ver >= 0) min_vals[1] = ver[idx_max_ver]; //instead of saving ae, save the verification value.
        min_vals[0] = neighbor_poses[idx_max_ver];
    }
    return min_vals;
}

std::vector<double> ImageRetrieval::ParallelIRWithKnownInput(std::vector<std::string> refimages, string image1){
    std::vector<double> res(refimages.size(), 0);

    for(int i=0; i<res.size(); i++){
        int tidx = man.GetOpenMachine();
        ws[tidx]->Setup(refimages[i], image1, &res[i]);
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    std::vector<double> min_vals = {-1, 1000000000};
    if(refimages.size()>0){
        min_vals[0] = GetMin(res);
        if(min_vals[0] >=0) min_vals[1] = res[(int)min_vals[0]];
    }
    return min_vals;
}



