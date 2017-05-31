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
    char line[LINESIZE]="";
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

std::string EvaluateSLAM::formattime(double seconds) {
    int hours = seconds/3600;
    int minutes = (seconds-hours*3600)/60;
    seconds = seconds-hours*3600- minutes*60;
    return to_string(hours) + ":"+to_string(minutes)+":"+to_string(seconds);
}

std::vector<double> EvaluateSLAM::MeasureReprojectionError(std::vector<double>& boat, std::vector<gtsam::Point2>& orig_imagecoords, std::vector<gtsam::Point3>& p) {
    gtsam::Pose3 tf(gtsam::Rot3::ypr(boat[5],boat[4],boat[3]), gtsam::Point3(boat[0],boat[1],boat[2]));
    
    double total_error = 0;
    double count = 0;
    double num_bad=0;
    std::string string_data = "";
    for(int j=0; j<orig_imagecoords.size(); j++) {
        if(p[j].x()==0.0 && p[j].y()==0.0 && p[j].z()==0.0) continue;
        
        gtsam::Point3 res = tf.transform_to(p[j]);
        gtsam::Point2 twodim = _cam.ProjectToImage(res);
        gtsam::Point2 orig = orig_imagecoords[j];
        
        double error = pow(pow(twodim.x() - orig.x(), 2)+pow(twodim.y() - orig.y(), 2),0.5);
        if(error>badthreshold) num_bad++;
        count++;
        total_error += error;
    }
    std::vector<double> res = {total_error, count, num_bad};
    return res;
}

std::vector<double> EvaluateSLAM::ErrorForSurvey(std::string _pftbase){
    time_t start,interm,end;
    time (&start);
    ParseOptimizationResults POR(_results_dir + _date);

    std::vector<double> result(POR.boat.size(), 0.0);
    int countbad = 0;
    int avgbadness = 0;
    double sum=0;
    int count=0;
    for(int i=0; i<POR.boat.size(); i++) {
        ParseFeatureTrackFile PFT(_cam, _pftbase + _date, POR.ftfilenos[i]);
        std::vector<gtsam::Point3> p_subset = POR.GetSubsetOf3DPoints(PFT.ids);

        std::vector<double> stats = MeasureReprojectionError(POR.boat[i], PFT.imagecoord, p_subset);
        if(stats[1] > 0) {
            result[i] = stats[0]/stats[1];
            sum += result[i];
            count++;
            if(result[i] > avgbadthreshold) avgbadness++;
        }
        else avgbadness++;
        countbad += stats[2];

//        if(debug && i%100==0) {
//            cout << "ITERATION: " << i << endl;
//            cout << "# unacceptable landmarks (not so high): " << countbad << endl;
//            cout << "# average unacceptable (should be nearly zero): " << avgbadness << endl;
//            cout << "Average Rerror: " << sum/count << std::endl;
//            time (&interm);
//            double dif = difftime (interm,start);
//            cout << "HH:MM:SS of run time so far: " << formattime(dif) << endl;
//        }
    }
    if(debug) {
        cout << "Average Rerror: " << sum/count << std::endl;
        cout << "  " << countbad << " unacceptable landmarks (not so high)" << endl;
        cout << "  " << avgbadness << " average unacceptable (should be nearly zero)" << endl;
    }
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

void EvaluateSLAM::Evaluate() {
    std::vector<double> evaluation  = ErrorForSurvey(std::string _pftbase);
    SaveEvaluation(evaluation);
}



