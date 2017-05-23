//
//  ImagePose.h
//  SIFTFlow
//
//  Created by Shane Griffith on 2/18/16.
//  Copyright © 2016 shane. All rights reserved.
//

#ifndef ImagePose_h
#define ImagePose_h
#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseSurvey.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <FileParsing/ParseFeatureTrackFile.h>

#include <DataTypes/Camera.hpp>

#include <string>
#include <iostream>

class img_pose{
private:
public:
    static const int image_skip = 5;
    
    int image;
    std::vector<double> pose;
    int ftfileno;
    double av;
    double time;
    int auxidx;
    int camera_key;
    
    img_pose(int i, std::vector<double> p, int f, double v, double t, int x, int k):
    image(i), pose(p), ftfileno(f), av(v), time(t), auxidx(x), camera_key(k){}
    
    static std::vector<double> GetIntermediatePose(std::vector<double> p1, std::vector<double> p2, double weight){
        /*Weight the poses equally. Due to the camera transitions, however, if the camera flipped, use the yaw of p1.*/
        //alpha * p1 + (1-alpha)*p2;
        std::vector<double> res;
        for(int i=0; i<p1.size(); i++){
            double comb = weight*p1[i] + (1-weight)*p2[i];
            res.push_back(comb);
        }
        return res;
    }
    
    static int FindFeatureTrackFile(Camera& _cam, std::string base, int idx, double time=0){
        int count = idx;
        ParseFeatureTrackFile PFT(_cam, base, count);
        while(PFT.time<time) { //pft.time is -1 if the file is bad, so a default time of 0 uses the first available file from idx.
            PFT.Next(++count);
        }
        return count;
    }
    
    static double PoseDistance(std::vector<double>& pose1, std::vector<double>& pose2){
        /*compute the distance between the poses
         check that the camera is initialized.
         check that the yaw of both is within 10 degrees
         */
        double projx1 = pose1[0] + 10*cos(pose1[3]);
        double projy1 = pose1[1] + 10*sin(pose1[3]);
        
        double projx2 = pose2[0] + 10*cos(pose2[3]);
        double projy2 = pose2[1] + 10*sin(pose2[3]);
        double euc = pow((projx2-projx1),2) + pow((projy2-projy1),2);
        double euc2 = pow(pose2[0]-pose1[0],2) + pow(pose2[1]-pose1[1],2);
        
        return euc+euc2; //uncomment for both shore+pose.
    }
    
    static int FindClosestPose(std::vector<std::vector<double> >& poses, std::vector<double> pose){
        double mind = 10000000000;
        int center=-1;
        for(int i=0; i<poses.size(); i++) {
            double dist = PoseDistance(poses[i], pose);
            if(dist == -1) continue;
            if(dist < mind){
                mind = dist;
                center = i;
            }
        }
        
        if(center == -1) {
            perror("failed to initialize the sequence.");
            exit(-1);
        }
        return center;
    }
    
    static std::vector<int> HandleEdgeCase(int sidx, int eidx, ParseSurvey& ps){
    	/*The edge case that sidx > eidx (The survey started and ended in the section)
		*  Between the sequences [sidx,end) and (start, eidx], find the two closest poses that overlap least.
		*   e.g., check all the poses in seq1 for the closest pose in seq2. Use the pair with min distance.
		*/
    	int mini = sidx;
    	int minj = 0;
    	double mind = 10000000000;
    	for(int i=sidx; i<ps.NumPoses(); i++) {
    		std::vector<double> posei = ps.GetPose(i);
    		for(int j=0; j<eidx; j++){
    			std::vector<double> posej = ps.GetPose(j);
				double dist = PoseDistance(posej, posei);
				if(dist < mind) {
					mind = dist;
					mini = i;
					minj = j;
				}
    		}
    	}
        
    	return {sidx, mini, minj, eidx};
    }

    static std::vector<img_pose> GetDataBetweenIndices(int sidx, int eidx, ParseSurvey& ps){
    	int limage = -1;
    	std::vector<img_pose> res;
		for(int i=sidx; i<eidx; i++){
			int image = ps.GetImageNumber(i);
			if(image%image_skip !=0)continue; //rather than incrementing by image_skip, this method ensures the images are all, e.g., divisible by 5.
			if(image == limage) continue; //only add the image once. (in case there are duplicates in the aux file)
			limage = image;

			std::vector<double> pose = ps.GetPose(i);
			int pftno = ps.GetImageNumber(i);
			double av = ps.GetAvgAngularVelocity(max(i-20,0), i);
			double image_time = ps.timings[i];
			img_pose ip(image, pose, pftno, av, image_time, i, -1);
			res.push_back(ip);
		}
        
		return res;
    }
	
    static img_pose GetTransition(img_pose a, img_pose b){
    	std::vector<double> p;
    	for(int i=0; i<a.pose.size(); i++)
			p.push_back(a.pose[i]);
		p[5] += M_PI; //marks the transition.
		img_pose n(-1, p, -1, 0, a.time, a.auxidx, -1);
    	return n;
    }

    static std::vector<img_pose> GetDataBetweenPoses(std::vector<double> start, std::vector<double> end, ParseSurvey& ps){
        std::vector<std::vector<double> >& poses = ps.Poses();
		int sidx = FindClosestPose(poses, start);
		int eidx = FindClosestPose(poses, end);
		std::vector<img_pose> res;

		if(sidx == -1 || eidx ==-1){
			/*If 1 is negative and the other isn't, that's an edge case, which could be handled, but isn't here.*/
			cout <<"warning: no images between the two poses" << endl;
			return res;
		}

		if(sidx > eidx){
			std::vector<int> indices = HandleEdgeCase(sidx, eidx, ps);
			std::vector<img_pose> sec1 = GetDataBetweenIndices(indices[0], indices[1], ps);
			std::vector<img_pose> sec2 = GetDataBetweenIndices(indices[2], indices[3], ps);
			img_pose transition = GetTransition(sec1[sec1.size()-1], sec2[sec2.size()-1]);
			sec1.push_back(transition);
			sec1.insert(sec1.end(), sec2.begin(), sec2.end());
			return sec1;
		}

		return  GetDataBetweenIndices(sidx, eidx, ps);
	}
    
    static std::vector<img_pose> GetDataBetweenPoses(Camera& _cam, std::vector<double> start, std::vector<double> end, ParseOptimizationResults& por, ParseSurvey& ps, string base){
        //Note: this version may not match BruteForceAlignment.cpp. Cross-check the code and implement a single version later for consistency.
        std::vector<int>& images = por.cimage;
        std::vector<vector<double> >& poses = por.boat;
        
        int sidx = FindClosestPose(poses, start);
        int eidx = FindClosestPose(poses, end);
        std::vector<img_pose> res;
        
        if(sidx == -1 || eidx ==-1 || sidx > eidx){
            cout <<"warning: no images between the two poses" << endl;
            return res;
        }
        
        int lastftfileno = por.ftfilenos[sidx]-1;
        int lastauxidx = por.auxidx[sidx]-1;
        for(int i=sidx; i<eidx; i++){
            int n_between = images[i+1] - images[i];
            for(int j=0; j<n_between; j++){
                
                int image = images[i]+j;
                if(image%image_skip !=0)continue;
                
                std::vector<double> pose = GetIntermediatePose(poses[i+1], poses[i], (1.0*j)/n_between);
                while(lastauxidx < ps.timings.size() && ps.GetImageNumber(++lastauxidx) < image);
                if(lastauxidx >= ps.timings.size()){
                    std::cout << "image: " << image << std::endl;
                    std::cout << "reached the end of the aux: " << lastauxidx<<std::endl;
                    exit(1);
                }
                double image_time = ps.timings[lastauxidx];
                lastftfileno = FindFeatureTrackFile(_cam, base, ++lastftfileno, image_time);
                double av = ps.GetAvgAngularVelocity(lastauxidx-20, lastauxidx);
                img_pose ip = {image, pose, lastftfileno, av, image_time, lastauxidx, -1};
                res.push_back(ip);
            }
        }
        
        return res;
    }
    
    static std::vector<int> GetRangeOfMapPoints(Camera& _cam, std::vector<double> start, std::vector<double> end, ParseOptimizationResults& por, std::string base){
        int sidx = FindClosestPose(por.boat, start);
        int ftfilenostart = FindFeatureTrackFile(_cam, base, por.ftfilenos[sidx], 0);
        ParseFeatureTrackFile PFTs(_cam, base, ftfilenostart);
        int first = PFTs.ids[0];

        int eidx = FindClosestPose(por.boat, end);
        int ftfilenoend = FindFeatureTrackFile(_cam, base, por.ftfilenos[eidx], 0);
        ParseFeatureTrackFile PFTe(_cam, base, ftfilenoend);
        int last = PFTe.ids[PFTe.ids.size()-1];

        return {first, last};
    }
};

#endif /* ImagePose_h */
