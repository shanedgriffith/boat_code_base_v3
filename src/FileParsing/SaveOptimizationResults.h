//
//  SaveOptimizationResults.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/10/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SRC_FILEPARSING_SAVEOPTIMIZATIONRESULTS_H_
#define SRC_FILEPARSING_SAVEOPTIMIZATIONRESULTS_H_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include "OptimizationResults.hpp"

class SaveOptimizationResults: public OptimizationResults {
private:
    bool started_correspondence;
    bool filter_null_points;
    
    std::string _base;
    time_t beginning,optstart,end;
    
    void SaveLandmarks(std::vector<std::vector<double> >& landmarks);
    void SavePoses(std::string file, std::vector<std::vector<double> >& traj);
    void SaveVisualization(std::vector<std::vector<double> >& landmarks, std::vector<std::vector<double> >& traj, std::vector<double> drawscale);
    
public:
    bool created_base_dir = false;
    bool save_status;
    bool draw_map;

    SaveOptimizationResults(std::string base):
    filter_null_points(true), started_correspondence(false), save_status(false), draw_map(false), _base(base)
    {
        MakeDir(_base);
        time(&beginning);
    }
    
    void TimeOptimization();
    void SetSaveDir(std::string dir){_base = dir; MakeDir(_base); started_correspondence = false;}
    void SetDrawMap() {MakeDir(_base + drawdir); draw_map = true;}
    void SetSaveStatus() {FILE * bts = OpenFile(_base + statusfile, "w"); fclose(bts); save_status = true;}
    
    void StatusMessage(int iteration, double percent_completed);
    
    void PlotAndSaveCurrentEstimate(std::vector<std::vector<double> >& landmarks, std::vector<std::vector<double> >& traj, std::vector<std::vector<double> >& vels, std::vector<double> drawscale);
    
    void SaveDataCorrespondence(int camera_key, int sift_file_no, int aux_file_idx, int imageno, double timestamp);
};


#endif /* SRC_FILEPARSING_SAVEOPTIMIZATIONRESULTS_H_ */
