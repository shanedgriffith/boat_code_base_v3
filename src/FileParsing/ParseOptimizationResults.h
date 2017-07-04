//
//  ParseOptimizationResults.h
//  VisualizationCode
//
//  Created by Shane Griffith on 6/9/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SRC_FILEPARSING_PARSEOPTIMIZATIONRESULTS_H_
#define SRC_FILEPARSING_PARSEOPTIMIZATIONRESULTS_H_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include "OptimizationResults.hpp"

class ParseOptimizationResults: public OptimizationResults{
private:
    enum LINETYPE{pose, vels, corres, point};
    void ProcessPoseEntries(std::vector<std::string> lp, std::vector<std::vector<double> >& poses);
    void ProcessPointEntries(std::vector<std::string> lp);
    void ProcessCorrespondenceEntries(std::vector<std::string> lp);
    
    std::vector<double> LoadReprojectionErrorFile(std::string evalfile);
    void SortPoints();
    int GetIndexOfFirstPoint(int id);
    void LoadOptimizationResult(bool sorted);

    void RemoveTransitionEntries();
    std::vector<int> to_remove;
    std::vector<double> rerror;//reprojection error for each entry in cimage.
protected:
    void ProcessLineEntries(int type, std::vector<std::string> lp);
    void ReadDelimitedFile(std::string file, int type);
public:
    typedef struct{
        gtsam::Point3 p;
        int p_id;
    } point_obj;

    bool debug = false;
    std::string _base;
    std::vector<std::vector<double> > boat;//optimized camera poses (misnomer)
    std::vector<std::vector<double> > velocities;//optimized boat velocities
    std::vector<int> cimage;//image correspondences
    std::vector<int> auxidx;//aux file index correspondences
    std::vector<int> ftfilenos;//feature track file correspondences (in the sift dir)
    std::vector<double> timings;
    std::vector<point_obj> p;
    
    ParseOptimizationResults(){};
    
    ParseOptimizationResults(std::string base, bool sorted = true):
    _base(base) {
        LoadOptimizationResult(sorted);
    }
    
    int GetNumberOfPoses(){return (int) boat.size();}
    
    //meant to be called with the feature tracking ids
    std::vector<gtsam::Point3> GetSubsetOf3DPoints(std::vector<int>& ids_subset);
    
    std::vector<double> ReprojectionError(){
        //this just loads the reprojection error file.
        if(rerror.size() == 0)
            rerror = LoadReprojectionErrorFile(_base + reprofile);
        return rerror;
    }
    
    int GetImageIndexGivenPose(std::vector<double> ref_pose, double* boat_pose=NULL);
    
    gtsam::Pose3 CameraPose(int idx);
};


#endif /* SRC_FILEPARSING_PARSEOPTIMIZATIONRESULTS_H_ */
