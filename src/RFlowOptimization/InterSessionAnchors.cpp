#include "InterSessionAnchors.hpp"

#include "Optimization/SingleSession/GTSAMInterface.h"

const std::string InterSessionAnchors::anchorpath_ = "_anchors.csv";

std::string
InterSessionAnchors::
getFileName()
{
    return sessionj_ + "_" + sessionk_ + anchorpath_;
}

void
InterSessionAnchors::
writeAnchors(std::string directory)
{
    std::string filepath = directory + getFileName();
    FILE * fp = OpenFile(filepath,"w");
    
    fprintf(fp, "%s, %s\n", sessionj_.c_str(), sessionk_.c_str());
    
    char line[100];
    for(int i=0; i<anchorkj.size(); i++)
    {
        std::vector<double> pose_vec = GTSAMInterface::PoseToVector(anchorkj[i]);
        
        fprintf(fp, "%d, %d, %lf, %lf, %lf, %lf, %lf, %lf\n",
                jindices[i], kindices[i],
                pose_vec[0], pose_vec[1], pose_vec[2],
                pose_vec[3], pose_vec[4], pose_vec[5]);
    }
    fflush(fp);
    fclose(fp);
}

void
InterSessionAnchors::
readAnchors(std::string directory)
{
    std::string filepath = directory + getFileName();
    FILE * fp = OpenFile(filepath,"r");
    
    char line[LINESIZE];
    fgets(line, LINESIZE-1, fp);
    int s0time, s1time;
    sscanf(line, "%d, %d\n", &s0time, &s1time);
    
    while (fgets(line, LINESIZE-1, fp)) {
        int d0, d1;
        std::vector<double> pose_vec(6);
        
        int ret = sscanf(line, "%d, %d, %lf, %lf, %lf, %lf, %lf, %lf\n",
                     &d0, &d1,
                     &pose_vec[0], &pose_vec[1], &pose_vec[2],
                     &pose_vec[3], &pose_vec[4], &pose_vec[5]);
        
        if(ret != 6)
        {
            break;
        }
        
        gtsam::Pose3 pose = GTSAMInterface::VectorToPose(pose_vec);
        addAnchor(pose, d0, d1);
    }
    fclose(fp);
}

void
InterSessionAnchors::
addAnchor(gtsam::Pose3 anchor, int j, int k)
{
    anchorkj.push_back(anchor);
    jindices.push_back(j);
    kindices.push_back(k);
}

gtsam::Pose3
InterSessionAnchors::
getNearestAnchor(int index, bool sessionj)
{
    std::vector<int>& indices = jindices;
    if(not sessionj)
    {
        indices = kindices;
    }
    
    int closestindx = 0;
    double closestdist = 1000000;
    for(int i=0; i<indices.size(); i++)
    {
        int dist = fabs(indices[i] - index);
        if(dist < closestdist)
        {
            closestdist = dist;
            closestindx = i;
        }
    }
    
    return anchorkj[closestindx];
}








