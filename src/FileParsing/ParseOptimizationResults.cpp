//
//  ParseOptimizationResults.cpp
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "ParseOptimizationResults.h"
#include "Optimization/SingleSession/GTSAMInterface.h"
#include <cmath>


using namespace std;

void ParseOptimizationResults::LoadOptimizationResult() {
    ReadDelimitedFile(_map_base + _date + boatfile, LINETYPE::pose);
    if(Exists(_map_base + _date  + velocityfile)) ReadDelimitedFile(_map_base + _date  + velocityfile, LINETYPE::vels);
    ReadDelimitedFile(_map_base + _date  + correspondencefile, LINETYPE::corres);
    ReadDelimitedFile(_map_base + _date  + pointsfile, LINETYPE::point);
    RemoveTransitionEntries();
    
    if(verbose) printf("ParseOptimizationResults::LoadOptimizationResult() Read %d points.\n", (int) p.size());
    
    SortPoints();
}

void ParseOptimizationResults::ProcessPoseEntries(vector<string> lp, vector<vector<double> >& poses) {
    if(lp.size()<7) return;
    vector<double> pose(lp.size()-1, 0.0);
    for(int i=1; i<lp.size(); i++) pose[i-1] = stod(lp[i]);
    poses.push_back(pose);
}

void ParseOptimizationResults::ProcessCorrespondenceEntries(vector<string> lp) {
    if(lp.size()<4) return;
    ftfilenos.push_back(stoi(lp[1]));
    auxidx.push_back(stoi(lp[2]));
    cimage.push_back(stoi(lp[3]));
    if(stoi(lp[3]) == -1) to_remove.push_back(auxidx.size()-1);
    
    //using if statements for backwards compatibility (newer files have six entries)
    if(lp.size()>4)timings.push_back(stod(lp[4]));
}

void ParseOptimizationResults::RemoveTransitionEntries() {
	for(int i=to_remove.size()-1; i>=0; i--){
		int idx = to_remove[i];
		boat.erase(boat.begin()+idx);
		cimage.erase(cimage.begin()+idx);
		ftfilenos.erase(ftfilenos.begin()+idx);
		auxidx.erase(auxidx.begin() + idx);
		if(timings.size()>0)timings.erase(timings.begin() + idx);
	}
}

void ParseOptimizationResults::ProcessPointEntries(vector<string> lp) {
    if(lp.size() < 4) return;
    point_obj po;
    po.p_id = stoi(lp[0]);
    po.p = gtsam::Point3(stod(lp[1]), stod(lp[2]), stod(lp[3]));
    p.push_back(po);
}

void ParseOptimizationResults::ProcessLineEntries(int type, vector<string> lp) {
    switch(type){
        case LINETYPE::pose:
            ProcessPoseEntries(lp, boat);
            break;
        case LINETYPE::vels:
            ProcessPoseEntries(lp, velocities);
            break;
        case LINETYPE::corres:
            ProcessCorrespondenceEntries(lp);
            break;
        case LINETYPE::point:
            ProcessPointEntries(lp);
            break;
    }
}

void ParseOptimizationResults::ReadDelimitedFile(string file, int type) {
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp))
    {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

void ParseOptimizationResults::SortPoints() {
    if(debug) cout << "Sorting the optimized points." << endl;
    /*note: sorting would be faster while loading.
     */
    
    std::qsort(&p[0], p.size(), sizeof(point_obj), [](const void* a, const void* b) {
        point_obj arg1 = *static_cast<const point_obj*>(a);
        point_obj arg2 = *static_cast<const point_obj*>(b);
        
        if(arg1.p_id < arg2.p_id) return -1;
        if(arg1.p_id > arg2.p_id) return 1;
        return 0;
    });
    
    landmarks = std::vector<std::vector<double> >(p.size(), std::vector<double>(4, 0.0));
    for(int i=0; i<p.size(); i++){
        landmarks[i][0] = p[i].p.x();
        landmarks[i][1] = p[i].p.y();
        landmarks[i][2] = p[i].p.z();
        landmarks[i][3] = p[i].p_id;
    }
}

vector<double> ParseOptimizationResults::LoadReprojectionErrorFile(string evalfile) {
    FILE * fe = OpenFile(evalfile, "r");
    vector<double> coarseeval;
    char bigline[100000];
    while(fgets(bigline, 100000, fe))
    {
        vector<double> pt;
        char * tmp = bigline;
        vector<string> vec = ParseLineAdv(tmp, ",");
        for(int i=2; i<vec.size(); i++)
        {
            coarseeval.push_back(stod(vec[i]));
        }
    }
    fclose(fe);
    return coarseeval;
}

int ParseOptimizationResults::GetIndexOfFirstPoint(int id) {
  int bot = 0;
  int top = p.size();
  while(bot < top) {
    int mid = bot + (top-bot)/2;
    if(p[mid].p_id > id) top = mid;
    else if(p[mid].p_id < id) bot = mid+1;
    else return mid;
  }
  return -1;
}

vector<gtsam::Point3> ParseOptimizationResults::GetSubsetOf3DPoints(vector<int>& ids_subset) {
    vector<gtsam::Point3> pset;
    if(ids_subset.size() == 0) return pset;

    int iter = -1;
    int countfound = 0;
    for(int i=0; i < ids_subset.size(); i++) {
        bool found = false;
        if(iter==-1) iter = GetIndexOfFirstPoint(ids_subset[i]);

        if(iter>=0){
            for(int j=iter; j < p.size(); j++) {
                if(p[j].p_id==ids_subset[i]) {
                    pset.push_back(p[j].p);
                    iter = j;
                    countfound++;
                    found = true;
                    break;
                }
                else if(p[j].p_id > ids_subset[i]){
                    break;
                }
            }
        }

        if(!found) pset.push_back(gtsam::Point3(0,0,0));
    }
    if(debug) cout << "Found " << countfound << " of " << ids_subset.size() << " points." << endl;
    return pset;
}

gtsam::Pose3 ParseOptimizationResults::CameraPose(int idx){
    return GTSAMInterface::VectorToPose(boat[idx]);
}

double DistanceFunction(vector<double> pose1, vector<double> pose2) {
    double projx1 = pose1[0] + 10*cos(pose1[3]);
    double projy1 = pose1[1] + 10*sin(pose1[3]);
    
    double projx2 = pose2[0] + 10*cos(pose2[3]);
    double projy2 = pose2[1] + 10*sin(pose2[3]);
    double euc = pow((projx2-projx1),2) + pow((projy2-projy1),2);
    double euc2 = pow(pose2[0]-pose1[0],2) + pow(pose2[1]-pose1[1],2);
    
    return euc+euc2; //uncomment for both shore+pose.
}

vector<int> GetNearestShore(vector<double> pose1, vector<vector<double> > bps2, vector<double>& dist_res) {
    /*
     Get indices for the two smallest poses and their distances.
     */
    
    double mind1 = 100000000;
    double mind2 = 100000000;
    vector<int> mini;
    mini.push_back(-1);
    mini.push_back(-1);
    
    for(int i=0; i<bps2.size(); i++)
    {
        vector<double> pose2 = bps2[i];
        double dist = DistanceFunction(pose1, pose2);
        if(dist > 2000) i = i+20+5*dist/2000;
        if(dist < mind1)
        {
            mind2 = mind1;
            mini[1] = mini[0];
            mind1 = dist;
            mini[0] = i;
        }
        else if(dist < mind2)
        {
            mind2 = dist;
            mini[1] = i;
        }
        if(mind1<90 && dist>150) break; //stop iterating if the nearest boat pose has already been found.
    }
    dist_res.push_back(mind1);
    dist_res.push_back(mind2);
    return mini;
}

int IdentifyBestLikelyImage(vector<double> q, vector<double> p1, vector<double> p2, int n, double * distres=NULL) {
    //p2-p1/n
    vector<double> inc_amt;
    for(int i=0; i<p1.size(); i++)
    {
        inc_amt.push_back((p2[i]-p1[i])/n);
    }
    
    double mini = 0;
    double mind = 1000;
    //n is the difference between image indices.
    for(int i=0; i<n; i++)
    {
        vector<double> newp;
        for(int j=0; j<p1.size(); j++)
        {
            newp.push_back(p1[j] + i*inc_amt[j]);
        }
        
        double dist = DistanceFunction(q, newp);
        if(dist < mind)
        {
            mind = dist;
            mini = i;
        }
    }
    
    if(distres!= NULL) *distres = mind;
    
    return mini;
}

int ParseOptimizationResults::GetImageIndexGivenPose(vector<double> ref_pose, double* boat_pose){
    vector<double> dist_res;
    
    //identify the nearest two poses
    vector<int> idxs = GetNearestShore(ref_pose, boat, dist_res);
    if(debug) cout << "distance: " << dist_res[0] << ", idx: " << idxs[0] << endl;
    
    int nearest_idx = -1;
    if(dist_res[0] < 100) {
        //extra logic that may be unnecessary. This catches cases in which the nearest pose fell just out of range
        int botidx = max(idxs[0]-1, 0);
        int topidx = min(idxs[1]+1, GetNumberOfPoses()-1);
        
        int im1idx1 = cimage[botidx];
        int im1idx2 = cimage[topidx];
        int best = IdentifyBestLikelyImage(ref_pose, boat[botidx], boat[topidx], im1idx2-im1idx1);
        
        //nearest image
        nearest_idx = best + im1idx1;
        
        //nearest boat pose
        if(boat_pose != NULL){
            double b2idx = idxs[0];
            if(im1idx2-im1idx1!=0) b2idx = botidx + ((1.0*topidx-botidx)*best)/(im1idx2-im1idx1);
            *boat_pose = b2idx;
            if(debug) cout << "top: " << topidx << ", bot: " << botidx << endl;
        }
    }
    
    return nearest_idx;
}

int ParseOptimizationResults::GetNearestPoseToImage(int image){
    int s=0;
    int e=cimage.size();
    int med;
    while(e-s>1){
        med = s + (e-s)/2;
        if(cimage[med] < image) s = med;
        else if(cimage[med] > image) e = med;
        else return med;
    }
    if(cimage[med] == image) return med;
    if(med > cimage.size()-2 || abs(image-cimage[med]) < abs(image-cimage[med+1])) return med;
    return med+1;
}


