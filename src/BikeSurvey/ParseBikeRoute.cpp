
#include "ParseBikeRoute.hpp"
#include <VisualOdometry/VisualOdometry.hpp>
#include <chrono>

#include "Optimization/SingleSession/GTSamInterface.h"

using namespace std;

void ParseBikeRoute::ProcessLineEntries(int type, vector<string>& lp){
    if(lp.size()<8)return;
    timings.push_back(stod(lp[0]));
    vector<double> p = {stod(lp[1]), stod(lp[2]), stod(lp[3]), stod(lp[4]), stod(lp[5]), stod(lp[6])};
    poses.push_back(p);
    omega.push_back(stod(lp[7]));
}

void ParseBikeRoute::ReadDelimitedFile(string file, int type) {
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

int ParseBikeRoute::GetImageNumber(int auxidx){
    return auxidx;
}

double ParseBikeRoute::GetAvgAngularVelocity(int sidx, int eidx){
    /* The average angular velocity between two indices [sidx, eidx].
     * */
    sidx = max(0,sidx);
    eidx = min((int)omega.size()-1, eidx);
    double sum = 0.0;
    for(int i=sidx; i<=eidx; i++) {
        sum += omega[i];
    }
    if(eidx-sidx==0) return omega[eidx];
    return sum/(eidx-sidx);
}

Camera ParseBikeRoute::GetCamera(){
    Camera nexus(1206.41699, 1205.09164, 636.766777, 371.147712, 1280, 720);
    nexus.SetDistortion(0.0817643033, 0.682168738, 0.00118562419, -0.00100627619, 0.0); //assume k3 is unused???
    //0.0817643033, 0.682168738, 0.00118562419, -0.00100627619, -7.05688254
    return nexus;
}

vector<double> ParseBikeRoute::GetDrawScale(){
    return {-2000,400,-500,1500};
}

bool ParseBikeRoute::DistanceCriterion(std::vector<double>& pose1, std::vector<double>& pose2){
    double pose_distance_threshold = 5.0; //meters
    double pose_angle_threshold = 20.0; //degrees.
    double dist = pow(pow(pose1[0] - pose2[0], 2) + pow(pose1[1] - pose2[1],2), 0.5);
    if (dist > pose_distance_threshold) {
        // Don't try to match points more than 20m away, GPS is not
        // that bad.
        return false;
    }
    for(int i=3; i<6; i++){
        //if(i==5) std::cout << "got yaw remainder "<< fabs(remainder(pose1[i]-pose2[i],2*M_PI))<<std::endl;
        if (fabs(remainder(pose1[i]-pose2[i],2*M_PI)) > pose_angle_threshold*M_PI/180) {
            // If we're not looking remotely in the same direction, no
            // point trying to match
            return false;
        }
    }
    
    return true;
}

std::vector<double> ParseBikeRoute::InterpolatePoses(int idx, int a, int b, vector<double> pa, vector<double> pb){
    std::vector<double> p(pa.size(), 0);
    for(int i=0; i<pa.size(); i++){
        double w2 = (idx - 1.0*a)/(b-a);
        double w1 = 1 - w2;
        p[i] = pa[i] * w1 + pb[i] * w2;
    }
    return p;
}

void ParseBikeRoute::ModifyPoses(){
    
    Camera nexus = ParseBikeRoute::GetCamera();
    gtsam::Cal3_S2::shared_ptr gtcam = nexus.GetGTSAMCam();
    gtsam::Matrix gtmat = gtcam->matrix();
    
    VisualOdometry vo(nexus);
    vector<double> lastp;
    vector<vector<double>> filtered;
    vector<int> indices = {0};
    ParseFeatureTrackFile PFT0(nexus, _base + _date, 0);
    
    vector<double> curpose;
    for(int i=2; i<poses.size(); i=i+2){
        ParseFeatureTrackFile PFT1(nexus, _base + _date, i);
        std::pair<gtsam::Pose3, int> vop = vo.PoseFromEssential(PFT0, PFT1);
        vector<double> vp = GTSamInterface::PoseToVector(vop.first);
        if(i>2){
            bool smooth = DistanceCriterion(vp, lastp);;
            while(!smooth){
                std::cout << "skipped " << i << std::endl;
                PFT1.Next(++i);
                if(PFT1.time==-1) break; //this will add a bad pose to the end. problem?
                vop = vo.PoseFromEssential(PFT0, PFT1);
                vp = GTSamInterface::PoseToVector(vop.first);
                smooth = DistanceCriterion(vp, lastp);
            }
        }
        printf("pose %d from vo (%lf,%lf,%lf,%lf,%lf,%lf)\n",i,vp[0],vp[1],vp[2],vp[3],vp[4],vp[5]);
        if(i==2) curpose = vp;
        else {
            gtsam::Pose3 lip = GTSamInterface::VectorToPose(lastp);
            gtsam::Pose3 vip = GTSamInterface::VectorToPose(vp);
            gtsam::Pose3 cip = lip.compose(vip);
            curpose = GTSamInterface::PoseToVector(cip);
        }
        filtered.push_back(curpose);
        indices.push_back(i);
        PFT0 = PFT1;
        lastp = vp;
    }
    
    vector<vector<double>> newposes(poses.size(), vector<double>());
    for(int i=0, c=0; i<poses.size(); i++){
        while(indices[c] < i && indices.size() > c) c++;
        if(indices.size() <= c){
            //hmm, but this may cause issues like before with the duplicated GPS.
            std::cout << i << " using duplicated pose" << std::endl;
            newposes.push_back(newposes[newposes.size()-1]);
        }else newposes[i] = InterpolatePoses(i, c-1, c, poses[c-1], poses[c]);
    }
    
    poses = newposes;
}











































