//
//  EvaluateSLAM.cpp
//  SIFTFlow
//
//  Created by Shane Griffith on 1/28/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "EvaluateSLAM.h"
#include "FileParsing/ParseOptimizationResults.h"
#include "FileParsing/ParseFeatureTrackFile.h"
#include <stdio.h>

#include <gtsam/geometry/Pose3.h>

using namespace gtsam;
using namespace std;

const std::string EvaluateSLAM::reprofile = "/reprojection_error.csv";

vector<double> EvaluateSLAM::LoadRerrorFile(){
    std::string fname = _results_dir + _date + reprofile;
    std::vector<double> rerrors;
    FILE * fp = OpenFile(fname, "r");
    int LINESIZE = 10000;
    char line[LINESIZE];
    fgets(line, LINESIZE-1, fp);
    while(!feof(fp)){
        double val=0;
        fgets(line, LINESIZE-1, fp);
        if(sscanf(line, "%lf",&val)!=1) break;
        rerrors.push_back(val);
    }
    fclose(fp);
    // std::cout << "Rerror file nentries: " << rerrors.size() << ", first and last entries: " << rerrors[0] << ", " << rerrors[rerrors.size()-1] << std::endl;
    return rerrors;
}

double EvaluateSLAM::GetAverageRerror(vector<double> rerrors){
    static double avg_rerror=0.0;
    if(avg_rerror > 0.0001) return avg_rerror;
    double sum=0;
    int count=0;
    for(int i=0; i<rerrors.size(); i++){
        if(rerrors[i]<=0.0001) continue;
        count++;
        sum += rerrors[i];
    }
    if(count == 0){
        std::cout << "EvaluateSLAM::GetAverageRerror() Something went wrong with rerror file processing."<<std::endl;
        exit(1);
    }
    avg_rerror = sum/count;
    return avg_rerror;
}

double EvaluateSLAM::UpdateTots(vector<double>& stats){
    double ret = 0.0;
    if(stats[1] > 0) {
        ret = stats[0]/stats[1];
        tots[0] += ret;
        tots[1]++;
        if(ret > avgbadthreshold) tots[3]++;
        else if(tots.size()>4) {
            tots[4] += ret;
            tots[5]++;
        }
    }
    else tots[3]++;
    tots[2] += stats[2];
    return ret;
}

void EvaluateSLAM::PrintTots(string name){
    if(debug) {
        if(tots[1] == 0) {
            cout << "Average " + name + " Rerror: No data" << std::endl;
            return;
        }
        cout << "Average " + name + " Rerror: " << tots[0]/tots[1] << std::endl;
        if(tots.size()>4) cout << "  " << tots[4]/tots[5] << " average " + name + " rerror for inliers" << endl;
        cout << "  " << tots[2] << " unacceptable landmarks (not so high)" << endl;
        cout << "  " << tots[3] << " average unacceptable (should be nearly zero)" << endl;
    }
}

double EvaluateSLAM::MeasureReprojectionError(std::vector<double>& boat, std::vector<gtsam::Point2>& orig_imagecoords, std::vector<gtsam::Point3>& p, vector<double>& rerror) {
    gtsam::Pose3 tf(gtsam::Rot3::ypr(boat[5],boat[4],boat[3]), gtsam::Point3(boat[0],boat[1],boat[2]));
    
    double total_error = 0;
    double count = 0;
    double num_bad=0;
    std::string string_data = "";
    for(int j=0; j<orig_imagecoords.size(); j++) {
        if(rerror.size()>0 && (rerror[j]<0.0001 || rerror[j]>6.0)) continue;//skip points that are likely mismatches.
        if(p[j].x()==0.0 && p[j].y()==0.0 && p[j].z()==0.0) continue;
        
        gtsam::Point3 res = tf.transform_to(p[j]);
        gtsam::Point2 twodim = _cam.ProjectToImage(res);
        if(!_cam.InsideImage(twodim)) continue; //different.
        gtsam::Point2 orig = orig_imagecoords[j];
        
        double error = pow(pow(twodim.x() - orig.x(), 2)+pow(twodim.y() - orig.y(), 2),0.5);
        if(error>badthreshold) num_bad++;
        count++;
        total_error += error;
    }
    std::vector<double> stats = {total_error, count, num_bad};
    return UpdateTots(stats);
}

double EvaluateSLAM::OfflineRError(ParseOptimizationResults& POR, int idx, std::string _pftbase){
    ParseFeatureTrackFile PFT(_cam, _pftbase, POR.ftfilenos[idx]);
    std::vector<gtsam::Point3> p_subset = POR.GetSubsetOf3DPoints(PFT.ids);
    return MeasureReprojectionError(POR.boat[idx], PFT.imagecoord, p_subset);
}

std::vector<double> EvaluateSLAM::ErrorForSurvey(std::string _pftbase, bool save){
    time_t start,interm,end;
    time (&start);
    ParseOptimizationResults POR(_results_dir + _date);

    std::vector<double> result(POR.boat.size(), 0.0);
    for(int i=0; i<POR.boat.size(); i++) {
        result[i] = OfflineRError(POR, i, _pftbase);
    }
    if(save) SaveEvaluation(result);
    return result;
}

void EvaluateSLAM::SaveEvaluation(std::vector<double> evaluation, std::string altname){
    std::string filepath = _results_dir + _date + reprofile;
    if(altname.length()>0) filepath = _results_dir + _date + altname;
    FILE * fp = OpenFile(filepath, "w");
    fprintf(fp, "%s,0\n", _date.c_str());
    for(int i=0; i<evaluation.size(); i++){
        fprintf(fp, "%lf\n", evaluation[i]);
        fflush(fp);
    }
    fprintf(fp, "\n");
    fclose(fp);
}



