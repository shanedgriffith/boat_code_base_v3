//
//  PointMap.cpp
//  SIFTFlow
//
//  Created by Shane Griffith on 2/20/16.
//  Copyright © 2016 shane. All rights reserved.
//

#include "PointMap.hpp"


using namespace std;

void PointMap::ProcessLineEntries(int type, vector<string> lp){
    ids.push_back(stoi(lp[0]));
    variance.push_back(stod(lp[1]));
    double x = stod(lp[2]);
    double y = stod(lp[3]);
    double z = stod(lp[4]);
    landmark.push_back(gtsam::Point3(x, y, z));
}


void PointMap::ReadDelimitedFile(string file, int type)
{
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    fgets(line, LINESIZE-1, fp);
    while (fgets(line, LINESIZE-1, fp))
    {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}


void PointMap::SetIDs(int first, int last, ParseOptimizationResults& por){
    for(int i=first; i<=last; i++){
        ids.push_back(por.landmarks[i][3]);
        landmark.push_back(gtsam::Point3(por.landmarks[i][0], por.landmarks[i][1], por.landmarks[i][2]));
    }
}


double PointMap::ComputeVariance(vector<double>& list){
    
    if(list.size() == 0) return -1;
    double sum = 0.0;
    for(int i=0; i<list.size(); i++){
        sum += list[i];
        //printf(", %.1lf", list[i]);
    }
    sum /= list.size(); //subtract 1 here?
    return sum;
}


void PointMap::SetVariances(vector<vector<double> >& variance_list){
    variance.clear();
    double sum = 0;
    int countpos=0;
    for(int i=0; i<variance_list.size(); i++){
//        cout << ids[i];
        double v = ComputeVariance(variance_list[i]);
        variance.push_back(v);
       // cout << ", " << v << endl;
        if(v >= 0){
            sum += v;
            countpos++;
        }
        //if(i>100){cout<<"debug statement, to remove. setvariances of pointmap."<<endl;exit(1);}
    }
    //cout << "  Average point variance (for one date): " << sum/countpos << " = " << sum << "/"<<countpos<<endl;
}


void PointMap::WritePoints(){
    if(ids.size() != landmark.size()){
        cout << "Error. There are "<<ids.size() << " ids and " << landmark.size() << " landmarks."<<endl;
        exit(1);
    }
    FILE * bts = OpenFile(_pointmapfile, "w");
    for(int i=0; i<ids.size(); i++){
        gtsam::Point3 p = landmark[i];
        fprintf(bts, "%d,%lf,%lf,%lf,%lf\n", ids[i], variance[i], p.x(), p.y(), p.z());
    }
    fflush(bts);
    fclose(bts);
}



















