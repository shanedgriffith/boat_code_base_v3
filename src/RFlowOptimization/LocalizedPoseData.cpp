/*
 * LocalizedPoseData.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: shane
 */

#include "LocalizedPoseData.hpp"
#include "Optimization/SingleSession/GTSAMInterface.h"

#include "RFlowEvaluation/AlignImageMachine.hpp"
#include "ImageAlignment/GeometricFlow/ReprojectionFlow.hpp"
#include "RFlowOptimization/SFlowDREAM2RF.hpp"
#include "FileParsing/ParseOptimizationResults.h"

#include "FileParsing/ParseSurvey.h"


using namespace std;

const string LocalizedPoseData::lpath = "/localizations/";

string LocalizedPoseData::GetPath(string toppath, string altpath){
    string fname = to_string(s1time) + "_" + date0 + ".loc";
    string filepath = toppath + lpath + fname;
    if(altpath.length()>0) filepath = toppath + altpath + fname;
    return filepath;
}

void LocalizedPoseData::Save(string toppath, string altpath){
    string filepath = GetPath(toppath, altpath);
    //std::cout << "Saving LPD " << to_string(s1time) + "_" + to_string(s0) + ".loc" << std::endl;

    try{
    FILE * fp = OpenFile(filepath,"w");
    fprintf(fp, "%s, %s, %lf, %lf, %lf, %lf, %lf, %lf\n", date0.c_str(), date1.c_str(),
            tf_p0_to_p1frame0[0], tf_p0_to_p1frame0[1], tf_p0_to_p1frame0[2],
            tf_p0_to_p1frame0[3], tf_p0_to_p1frame0[4], tf_p0_to_p1frame0[5]);
    fprintf(fp, "%d, %d\n", s0time, s1time);
    fprintf(fp, "%lf, %lf\n", perc_dc, avg_rerror_inl);
    fprintf(fp, "%lf, %lf, %lf, %lf, %lf, %lf\n",
            p1frame0[0], p1frame0[1], p1frame0[2], p1frame0[3], p1frame0[4], p1frame0[5]);
    fprintf(fp, "%lf, %lf, %lf, %lf, %lf, %lf\n",
            p0frame1[0], p0frame1[1], p0frame1[2], p0frame1[3], p0frame1[4], p0frame1[5]);
    fprintf(fp, "%d, %d, %d, %d\n", (int) p3d.size(), (int)p3d0.size(), (int)b3d.size(), (int)b3d1.size());
    char allpoints[30000];
    int lastl=0;
    for(int i=0; i<p3d.size(); i++){
        sprintf(allpoints+lastl, "%lf, %lf, %lf\n", p3d[i].x(), p3d[i].y(), p3d[i].z());
        lastl += strlen(allpoints+lastl);
    }
    fprintf(fp, "%s", allpoints);
    fflush(fp);
    memset(allpoints, 0, lastl);
    lastl=0;
    for(int i=0; i<p3d0.size(); i++) {
        sprintf(allpoints+lastl, "%d, %lf, %lf, %lf, %lf, %lf, %lf\n", pids[i], p3d0[i].x(), p3d0[i].y(), p3d0[i].z(), p2d1[i].x(), p2d1[i].y(), rerrorp[i]);
        lastl += strlen(allpoints+lastl);
    }
    fprintf(fp, "%s", allpoints);
    fflush(fp);
    memset(allpoints, 0, lastl);
    lastl=0;
    for(int i=0; i<b3d.size(); i++) {
        sprintf(allpoints+lastl, "%lf, %lf, %lf\n", b3d[i].x(), b3d[i].y(), b3d[i].z());
        lastl += strlen(allpoints+lastl);
    }
    fprintf(fp, "%s", allpoints);
    fflush(fp);
    memset(allpoints, 0, lastl);
    lastl=0;
    for(int i=0; i<b3d1.size(); i++) {
        sprintf(allpoints+lastl, "%d, %lf, %lf, %lf, %lf, %lf, %lf\n", bids[i], b3d1[i].x(), b3d1[i].y(), b3d1[i].z(), b2d0[i].x(), b2d0[i].y(), rerrorb[i]);
        lastl += strlen(allpoints+lastl);
    }
    fprintf(fp, "%s", allpoints);
    fflush(fp);
    fclose(fp);
    } catch (const std::exception& e){
        std::cout << "Exception saving the LPD.\n"<<e.what()<<std::endl;
        exit(1);
    }
}

void LocalizedPoseData::CheckLine(string& filepath, int numr, int expected){
    if(numr != expected){
        std::cout << "LocalizedPoseData Read Error. During read, got "<<numr<<", expected "<< expected<<". Check file "<<filepath<<std::endl;
        exit(1);
    }
}

LocalizedPoseData LocalizedPoseData::Read(string filepath){
    LocalizedPoseData l;

    char line[LINESIZE];
    int d0, d1;
    std::vector<int> expectedperline = {8, 2, 2, 6, 6, 4, 3, 7, 3, 7};
    std::vector<int> numofline = {1, 1, 1, 1, 1, 1, -1, -1, -1, -1};
    int countofline = 0;
    int nump=0;
    int curset=0;
    double x, y, z, p1, p2, r;
    int id;
    FILE * fp = OpenFile(filepath,"r");
    while (fgets(line, LINESIZE-1, fp)) {
        int ret = 0;
        int expected = 0;

        switch(curset) {
        case 0: ret = sscanf(line, "%d, %d, %lf, %lf, %lf, %lf, %lf, %lf\n", &d0, &d1,
                                 &l.tf_p0_to_p1frame0[0], &l.tf_p0_to_p1frame0[1], &l.tf_p0_to_p1frame0[2],
                                 &l.tf_p0_to_p1frame0[3], &l.tf_p0_to_p1frame0[4], &l.tf_p0_to_p1frame0[5]); break;
        case 1: ret = sscanf(line, "%d, %d\n", &l.s0time, &l.s1time); break;
        case 2: ret = sscanf(line, "%lf, %lf\n", &l.perc_dc, &l.avg_rerror_inl); break;
        case 3: ret = sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf\n",
                    &l.p1frame0[0], &l.p1frame0[1], &l.p1frame0[2], &l.p1frame0[3], &l.p1frame0[4], &l.p1frame0[5]);
                break;
        case 4: ret = sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf\n",
                    &l.p0frame1[0], &l.p0frame1[1], &l.p0frame1[2], &l.p0frame1[3], &l.p0frame1[4], &l.p0frame1[5]);
               break;
        case 5: ret = sscanf(line, "%d, %d, %d, %d\n", &numofline[6], &numofline[7], &numofline[8], &numofline[9]); break;
        case 6: ret = sscanf(line, "%lf, %lf, %lf\n", &x, &y, &z); break;
        case 7: ret = sscanf(line, "%d, %lf, %lf, %lf, %lf, %lf, %lf\n", &id, &x, &y, &z, &p1, &p2, &r); break;
        case 8: ret = sscanf(line, "%lf, %lf, %lf\n", &x, &y, &z); break;
        case 9: ret = sscanf(line, "%d, %lf, %lf, %lf, %lf, %lf, %lf\n", &id, &x, &y, &z, &p1, &p2, &r); break;
        case 10: std::cout << "LocalizedPoseData Error. Too many lines. Or some other formatting error that lead to this.\n File "<<filepath << std::endl;
                exit(-1);
        }

        CheckLine(filepath, ret, expectedperline[curset]);

        switch(curset){
        case 5: l.p3d = std::vector<gtsam::Point3>(numofline[6]);
                l.pids = std::vector<int>(numofline[7]);
                l.p3d0 = std::vector<gtsam::Point3>(numofline[7]);
                l.p2d1 = std::vector<gtsam::Point2>(numofline[7]);
                l.rerrorp = std::vector<double>(numofline[7]);
                l.bids = std::vector<int>(numofline[8]);
                l.b3d = std::vector<gtsam::Point3>(numofline[8]);
                l.b3d1 = std::vector<gtsam::Point3>(numofline[9]);
                l.b2d0 = std::vector<gtsam::Point2>(numofline[9]);
                l.rerrorb = std::vector<double>(numofline[9]); break;
        case 6: l.p3d[nump] = gtsam::Point3(x, y, z); break;
        case 7: l.pids[nump] = id;
                l.p3d0[nump] = gtsam::Point3(x, y, z);
                l.p2d1[nump] = gtsam::Point2(p1, p2);
                l.rerrorp[nump] = r; break;
        case 8: l.b3d[nump] = gtsam::Point3(x, y, z); break;
        case 9: l.bids[nump] = id;
                l.b3d1[nump] = gtsam::Point3(x, y, z);
                l.b2d0[nump] = gtsam::Point2(p1, p2);
                l.rerrorb[nump] = r; break;
        default : break;
        }

        countofline++;
        nump++;
        if(countofline==numofline[curset]) {
            curset++;
            countofline=0;
            nump=0;
        }
    }
    fclose(fp);
    if(l.p3d.size()==0){
        std::cout << "LocalizedPoseData Error. Too many lines. Or some other formatting error that lead to this.\n File "<<filepath << std::endl;
        exit(-1);
    }
    l.date0 = to_string(d0);
    l.date1 = to_string(d1);
    return l;
}

std::vector<LocalizedPoseData> LocalizedPoseData::LoadAll(std::string toppath, std::string altpath, std::vector<std::string> dates){
    string dirpath = toppath + lpath;
    if(altpath.length()>0) dirpath = toppath + altpath;
    std::vector<LocalizedPoseData> res;
    if(!DirectoryExists(dirpath)) MakeDir(dirpath);
    else {
        std::vector<string> files = ListFilesInDir(dirpath, ".loc");
        for(int i=0; i<files.size(); i++) {
            if(dates.size() > 0){
                for(int j=0; j<dates.size(); j++) {
                    if(files[i].find(dates[j]) != string::npos) {
                        res.push_back(Read(dirpath + files[i]));
                        break;
                    }
                }
            } else res.push_back(Read(dirpath + files[i]));
        }
        std::sort(res.begin(), res.end()); //necessary. The file name lacks leading zeros, which makes the file order different.
        std::cout << "Found " << res.size() << " localizations.";
        if(res.size()>0) std::cout << " Last one: " << res[res.size()-1].s1time << std::endl;
        else std::cout<<std::endl;
        return res;
    }
    return {};
}

void swap(LocalizedPoseData& first, LocalizedPoseData& second){
    //see http://stackoverflow.com/questions/5695548/public-friend-swap-member-function
    //for an explanation of the syntax, and use of using, without specifying std::swap below.
    using std::swap;

    swap(first.date0, second.date0);
    swap(first.date1, second.date1);
    swap(first.tf_p0_to_p1frame0, second.tf_p0_to_p1frame0);
    swap(first.s0time, second.s0time);
    swap(first.s1time, second.s1time);
    swap(first.p1frame0, second.p1frame0);
    swap(first.p0frame1, second.p0frame1);
    swap(first.p3d, second.p3d);
    swap(first.b3d, second.b3d);
    swap(first.pids, second.pids);
    swap(first.bids, second.bids);
    swap(first.p3d0, second.p3d0);
    swap(first.p2d1, second.p2d1);
    swap(first.rerrorp, second.rerrorp);
    swap(first.b3d1, second.b3d1);
    swap(first.b2d0, second.b2d0);
    swap(first.rerrorb, second.rerrorb);
    swap(first.perc_dc, second.perc_dc);
    swap(first.avg_rerror_inl, second.avg_rerror_inl);
}

LocalizedPoseData& LocalizedPoseData::operator=(LocalizedPoseData other){
    //see http://stackoverflow.com/questions/3279543/what-is-the-copy-and-swap-idiom
    //for an explanation of why this is the right way to define an assignment operator.
    //This assignment isn't passed by reference, because we don't want to
    //copy the function arguments. See
    //https://web.archive.org/web/20140113221447/http://cpp-next.com/archive/2009/08/want-speed-pass-by-value/
    swap(*this, other);
    return *this;
}

bool LocalizedPoseData::operator<(const LocalizedPoseData& str) const {
    return ( (date1.compare(str.date1) < 0) ||
            (date1.compare(str.date1)==0 && s1time < str.s1time) ||
            (date1.compare(str.date1)==0 && s1time==str.s1time && date0.compare(str.date0) < 0));
}

void LocalizedPoseData::SetPoints(std::vector<unsigned char>& inliers, vector<gtsam::Point2>& p2d, vector<gtsam::Point3>& p3d, vector<int> ids, int bot, int top){
    int nin=0;
    for(int i=bot; i<top; i++)
        nin += (int) inliers[i];

    std::vector<gtsam::Point3>* points3d;
    vector<gtsam::Point2>* points2d;
    vector<int>* pointids;
    if(bot==0) {
        pids = std::vector<int>(nin);
        p3d0 = std::vector<gtsam::Point3>(nin);
        p2d1 = std::vector<gtsam::Point2>(nin);
        rerrorp = std::vector<double>(nin, ACCEPTABLE_RERROR);
        points3d = &p3d0;
        points2d = &p2d1;
        pointids = &pids;
    } else {
        bids = std::vector<int>(nin);
        b3d1 = std::vector<gtsam::Point3>(nin);
        b2d0 = std::vector<gtsam::Point2>(nin);
        rerrorb = std::vector<double>(nin, ACCEPTABLE_RERROR);
        points3d = &b3d1;
        points2d = &b2d0;
        pointids = &bids;
    }

    for(int i=bot, idx=0; i<top; i++){
        if(!inliers[i]) continue;
        (*pointids)[idx] = ids[i];
        (*points2d)[idx] = p2d[i];
        (*points3d)[idx++] = p3d[i];
    }
}

void LocalizedPoseData::SetPoses(std::vector<double> p0_, std::vector<double> p1frame0_, std::vector<double> p0frame1_){
    gtsam::Pose3 p0 = GTSAMInterface::VectorToPose(p0_);
    gtsam::Pose3 p1f0 = GTSAMInterface::VectorToPose(p1frame0_);
    gtsam::Pose3 res = p0.between(p1f0);
    tf_p0_to_p1frame0 = GTSAMInterface::PoseToVector(res);
    swap(p1frame0, p1frame0_);
    swap(p0frame1, p0frame1_);
}

void LocalizedPoseData::SetLocalizationQuality(double pdc, double rerror){
    perc_dc = pdc;
    avg_rerror_inl = rerror;
}

void PrintVec(std::vector<double> p){
    for(int i=0; i<p.size(); i++){
        std::cout << p[i]<<",";
    }
    std::cout << std::endl;
}

double LocalizedPoseData::VerifyWith(const Camera& _cam, LocalizedPoseData& lpd, gtsam::Pose3 p1_t, gtsam::Pose3 p1_tm1){
    //given the last flow, verify the next one.
    //use the estimate of the next pose (i.e., the odom.) to predict where the points should project.
    //and then verify that they actually do project to those locations.
    if(date0.length()==-1 || lpd.date0.length() ==-1) return false;
    int count_in_both=0;
    double sum_rerror=0;
    int count_inliers=0;
    
    gtsam::Pose3 p1frame0_t = GTSAMInterface::VectorToPose(lpd.p1frame0);
    gtsam::Pose3 p1frame0_tm1 = GTSAMInterface::VectorToPose(p1frame0);
//    gtsam::Pose3 p1frame0_t_est = p1frame0_tm1.compose(p1_tm1.between(p1_t)); //TODO: check this vs. the one below.
    gtsam::Pose3 p1frame0_t_est = p1frame0_tm1.compose(p1frame0_tm1.between(p1_tm1)*p1_tm1.between(p1_t)*p1_tm1.between(p1frame0_tm1));
    
    //check lpd.p3d onto p1frame0_tm1_est matches lpd.p3d onto p1frame0_tm1. (Tests points from t to tm1.)
    //NOTE: b3d isn't tested because s0 could be the same in both surveys. Thus odometry would be zero.
    for(int i=0; i<lpd.b3d.size(); i++){
        gtsam::Point3 p3t_est = p1frame0_t_est.transform_to(lpd.b3d[i]);
        gtsam::Point2 p2t_est = _cam.ProjectToImage(p3t_est);
        gtsam::Point3 p3t = p1frame0_t.transform_to(lpd.b3d[i]);
        gtsam::Point2 p2t = _cam.ProjectToImage(p3t);
//        if(i<10) std::cout << "point coordinates: ("<<p2t.x() << "," << p2t.y() << ") vs. (" <<p2t_est.x() << ", " << p2t_est.y() << ")"<<std::endl;
        if(!_cam.InsideImage(p2t_est) || !_cam.InsideImage(p2t)) continue;
        double dist = p2t_est.dist(p2t);
        sum_rerror += dist;
        count_in_both++;
        if(dist<=ACCEPTABLE_RERROR) count_inliers++;
    }
    
    bool verified = true;
    if(count_in_both == 0 || sum_rerror/count_in_both>ACCEPTABLE_RERROR) verified = false;
    if(1.0*count_in_both/lpd.b3d.size() < ACCEPTABLE_OVERLAP) verified = false;
    
    if(count_in_both > 0) // && debug
        std::cout<<"LPD verification stats (" << s1time << "." << date0 << " to " << lpd.s1time << "." <<lpd.date0<< ") " <<
        100.0*count_in_both/lpd.b3d.size()<<"% overlap, "<<100.0*count_inliers/count_in_both <<
        "% inliers, "<<sum_rerror/count_in_both<<" average rerror. VERIFIED? " << verified <<std::endl;
    else
        std::cout<<"LPD verification stats (" << s1time << "." << date0 << " to " << lpd.s1time << "." <<lpd.date0<< ") No overlap. VERIFIED? " << verified <<std::endl;
    
    if(verified) return sum_rerror/count_in_both;
    else return -1;
}

bool LocalizedPoseData::IsSet(){
    return date0.length() > 0;
}

void LocalizedPoseData::Print(string opt){
    printf("LocalizedPoseData----------%s\n", opt.c_str());
    printf("reference date: %s, date of current set: %s\n", date0.c_str(), date1.c_str());
    printf("s0time: %d, s1time: %d\n", s0time, s1time);
    printf("perc_dc: %lf, avg_rerror_inl: %lf\n", perc_dc, avg_rerror_inl);
    printf("tf_p0_to_p1frame0: "); PrintVec(tf_p0_to_p1frame0);
    printf("p1frame0: "); PrintVec(p1frame0);
    printf("p0frame1: "); PrintVec(p0frame1);
    printf("sizes. p3d: %d, b3d: %d, p3dsub: %d, b3dsub: %d\n", (int) p3d.size(), (int)b3d.size(), (int)p3d0.size(), (int)b3d1.size());
//    for(int i=0; i<5; i++){
//        p3d[i].print();
//        p3d0[i].print();
//        b3d[i].print();
//        b3d1[i].print();
//    }
}

gtsam::Pose3 LocalizedPoseData::GetP0frame1(){
    return GTSAMInterface::VectorToPose(p0frame1);
}

gtsam::Pose3 LocalizedPoseData::GetP1frame0(){
    return GTSAMInterface::VectorToPose(p1frame0);
}

gtsam::Pose3 LocalizedPoseData::GetTFP0ToP1F0(){
    return GTSAMInterface::VectorToPose(tf_p0_to_p1frame0);
}

void LocalizedPoseData::CheckLPD(const Camera& _cam, std::string _pftbase, std::string _results_dir, std::string _query_loc) {
    //a debugging tool.
    
    std::vector<ParseOptimizationResults> por;
    std::vector<Map> maps;
    
    maps.push_back(Map(_results_dir + "maps/"));
    maps.push_back(Map(_results_dir + "localized_maps/"));
    maps[0].LoadMap(date0);
    maps[1].LoadMap(date1);
    
    por.push_back(ParseOptimizationResults(_results_dir + "maps/", date0));
    por.push_back(ParseOptimizationResults(_results_dir + "localized_maps/", date1));
    
    ReprojectionFlow r0(_cam, maps[0]);
    ReprojectionFlow r1(_cam, maps[1]);
    std::vector<ReprojectionFlow*> rf = {&r0, &r1};
    
    ParseFeatureTrackFile pftf0 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + date0, por[0].ftfilenos[s0time]);
    ParseFeatureTrackFile pftf1 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + date1, por[1].ftfilenos[s1time]);
    
    rf[0]->ComputeFlow(p1frame0, por[0].boat[s0time]); //map points of survey 0 onto pose1_est.
    rf[1]->ComputeFlow(p0frame1, por[1].boat[s1time]); //map points of survey 1 onto pose0_est.
    rf[0]->CreateRestrictedSet(stoi(date0), pftf0);
    rf[1]->CreateRestrictedSet(stoi(date1), pftf1);
    
    string image0 = ParseSurvey::GetImagePath(_query_loc + date0, por[0].cimage[s0time]);
    string image1 = ParseSurvey::GetImagePath(_query_loc + date1, por[1].cimage[s1time]);
    
    SFlowDREAM2RF sf(_cam);
    sf.SetReprojectionFlow(rf);
    sf.SetEpipolar();
    sf.SetTwoCycleConsistency();
    sf.ConstructImagePyramid(image0, image1);
    sf.AlignImages();
    AlignmentResult ar = sf.GetAlignmentResult();
    //~/Documents/Research
    std::string dir = "/Volumes/SAMSUNG/Data/Debug/" + date0 + "-" + to_string(s0time) + "_" + date1 + "-" + to_string(s1time) + "/";
    ar.Save(dir);
}






