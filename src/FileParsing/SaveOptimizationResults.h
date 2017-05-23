//
//  SaveOptimizationResults.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/10/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __BundleAdjustOneDataset__SaveOptimizationResults__
#define __BundleAdjustOneDataset__SaveOptimizationResults__

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
    std::string _base;
    time_t start,end;
    
public:
    bool created_base_dir = false;
    bool started_correspondence = false;
    bool filter_null_points;
    bool save_status;
    bool draw_map;

    SaveOptimizationResults(std::string base):
    filter_null_points(true), save_status(false), draw_map(false), _base(base)
    {
        MakeDir(_base);
        time (&start);
    }
    
    void SetSaveDir(std::string dir){_base = dir; MakeDir(_base); started_correspondence = false;}
    void SetDrawMap() {MakeDir(_base + drawdir); draw_map = true;}
    void SetSaveStatus() {FILE * bts = OpenFile(_base + statusfile, "w"); fclose(bts); save_status = true;}
    
    void StatusMessage(int iteration, double percent_completed);
    
    void SaveLandmarks(std::vector<std::vector<double> >& landmarks);
    void SavePoses(std::string file, std::vector<std::vector<double> >& traj);
    
    void SaveVisualization(std::vector<std::vector<double> >& landmarks, std::vector<std::vector<double> >& traj);
    
    void PlotAndSaveCurrentEstimate(std::vector<std::vector<double> >& landmarks, std::vector<std::vector<double> >& traj, std::vector<std::vector<double> >& vels);
    
    void SaveDataCorrespondence(int camera_key, int sift_file_no, int aux_file_idx, int imageno, double timestamp);
    
    std::string GetParmsFileName(){return _base + parms;}
};


#endif /* defined(__BundleAdjustOneDataset__SaveOptimizationResults__) */
