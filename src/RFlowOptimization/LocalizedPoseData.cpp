/*
 * LocalizedPoseData.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: shane
 */

#include "LocalizedPoseData.hpp"

using namespace std;

const string LocalizedPoseData::lpath = "/localizations/";

string LocalizedPoseData::GetPath(string toppath, string altpath){
    string filepath = toppath + lpath + to_string(s1time) + ".loc";
    if(altpath.length()>0) filepath = toppath + altpath + to_string(s1time) + ".loc";
    return filepath;
}

void LocalizedPoseData::Save(string toppath, string altpath){
    string filepath = toppath + lpath + to_string(s1time) + ".loc";
    if(altpath.length()>0) filepath = toppath + altpath + to_string(s1time) + ".loc";

    try{
    FILE * fp = OpenFile(filepath,"w");
    fprintf(fp, "%s, %s, %lf, %lf, %lf, %lf, %lf, %lf\n", date0.c_str(), date1.c_str(),
            tf_p0_to_p1frame0[0], tf_p0_to_p1frame0[1], tf_p0_to_p1frame0[2],
            tf_p0_to_p1frame0[3], tf_p0_to_p1frame0[4], tf_p0_to_p1frame0[5]);
    fprintf(fp, "%d, %d, %d, %d\n", s0, s1, s0time, s1time);
    fprintf(fp, "%lf, %lf\n", perc_dc, avg_rerror_inl);
    fprintf(fp, "%lf, %lf, %lf, %lf, %lf, %lf\n",
            p1frame0[0], p1frame0[1], p1frame0[2], p1frame0[3], p1frame0[4], p1frame0[5]);
    fprintf(fp, "%lf, %lf, %lf, %lf, %lf, %lf\n",
            p0frame1[0], p0frame1[1], p0frame1[2], p0frame1[3], p0frame1[4], p0frame1[5]);
    fprintf(fp, "%d, %d, %d, %d\n", (int) p3d.size(), (int)p3d0.size(), (int)b3d.size(), (int)b3d1.size());
    char allpoints[20000];
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

    char line[LINESIZE]="";
    int d0, d1;
    std::vector<int> expectedperline = {8, 4, 2, 6, 6, 4, 3, 7, 3, 7};
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
        case 1: ret = sscanf(line, "%d, %d, %d, %d\n", &l.s0, &l.s1, &l.s0time, &l.s1time); break;
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
                l.p3d0 = std::vector<gtsam::Point3>(numofline[7]);
                l.p2d1 = std::vector<gtsam::Point2>(numofline[7]);
                l.rerrorp = std::vector<double>(numofline[7]);
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

std::vector<LocalizedPoseData> LocalizedPoseData::LoadAll(string toppath, string altpath){
    string dirpath = toppath + lpath;
    if(altpath.length()>0) dirpath = toppath + altpath;
    std::vector<LocalizedPoseData> res;
    if(!DirectoryExists(dirpath)) MakeDir(dirpath);
    else {
        std::vector<string> files = ListFilesInDir(dirpath, ".loc");
        for(int i=0; i<files.size(); i++)
            res.push_back(Read(dirpath + files[i]));
        //std::sort(res.begin(), res.end()); //the file list is now sorted. this should be unnecessary.
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
    swap(first.s0, second.s0);
    swap(first.s1, second.s1);
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
    return (s1time < str.s1time);
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
    gtsam::Pose3 p0 = VectorToPose(p0_);
    gtsam::Pose3 p1f0 = VectorToPose(p1frame0_);
    gtsam::Pose3 res = p0.between(p1f0);
    tf_p0_to_p1frame0 = PoseToVector(res);
    swap(p1frame0, p1frame0_);
    swap(p0frame1, p0frame1_);
}

void LocalizedPoseData::SetLocalizationQuality(double pdc, double rerror){
    perc_dc = pdc;
    avg_rerror_inl = rerror;
}

gtsam::Pose3 LocalizedPoseData::VectorToPose(std::vector<double>& p){
    return gtsam::Pose3(gtsam::Rot3::ypr(p[5], p[4], p[3]), gtsam::Point3(p[0], p[1], p[2]));
}

void PrintVec(std::vector<double> p){
    for(int i=0; i<p.size(); i++){
        std::cout << p[i]<<",";
    }
    std::cout << std::endl;
}

std::vector<double> LocalizedPoseData::PoseToVector(gtsam::Pose3& cam){
    return {cam.x(), cam.y(), cam.z(), cam.rotation().roll(), cam.rotation().pitch(), cam.rotation().yaw()};
}

bool LocalizedPoseData::VerifyWith(Camera& _cam, LocalizedPoseData& lpd, gtsam::Pose3 p1_t, gtsam::Pose3 p1_tm1){
    //given the last flow, verify the next one.
    //use the estimate of the next pose (i.e., the odom.) to predict where the points should project.
    //and then verify that they actually do project to those locations.
    if(s0==-1 || lpd.s0==-1) return false;
    int count_in_both=0;
    double sum_rerror=0;
    int count_inliers=0;

    gtsam::Pose3 p1frame0_t = VectorToPose(lpd.p1frame0);
    gtsam::Pose3 p1frame0_tm1 = VectorToPose(p1frame0);
    gtsam::Pose3 p1frame0_t_est = p1frame0_tm1.compose(p1_tm1.between(p1_t));
    
    //check lpd.p3d onto p1frame0_tm1_est matches lpd.p3d onto p1frame0_tm1. (Tests points from t to tm1.)
    //NOTE: b3d isn't tested because s0 could be the same in both surveys. Thus odometry would be zero.
    for(int i=0; i<lpd.p3d.size(); i++){
        gtsam::Point3 p3t_est = p1frame0_t_est.transform_to(lpd.p3d[i]);
        gtsam::Point2 p2t_est = _cam.ProjectToImage(p3t_est);
        gtsam::Point3 p3t = p1frame0_t.transform_to(lpd.p3d[i]);
        gtsam::Point2 p2t = _cam.ProjectToImage(p3t);
        if(!_cam.InsideImage(p2t_est) || !_cam.InsideImage(p2t)) continue;
        double dist = p2t_est.dist(p2t);
        sum_rerror += dist;
        count_in_both++;
        if(dist<=ACCEPTABLE_RERROR) count_inliers++;
    }
    
    if(count_in_both > 0) // && debug
        std::cout<<"LPD verification stats (" << s1time << " to " << lpd.s1time << ")" <<
        100.0*count_in_both/lpd.p3d.size()<<"% overlap, "<<100.0*count_inliers/count_in_both <<
        "% inliers, "<<sum_rerror/count_in_both<<" average rerror"<<std::endl;

    if(count_in_both == 0 || sum_rerror/count_in_both>ACCEPTABLE_RERROR) return false;
    if(1.0*count_in_both/lpd.p3d.size() < ACCEPTABLE_OVERLAP) return false;
    return true;
}

bool LocalizedPoseData::IsSet(){
    return s0>=0;
}

void LocalizedPoseData::Print(string opt){
    printf("LocalizedPoseData----------%s\n", opt.c_str());
    printf("reference date: %s, date of current set: %s\n", date0.c_str(), date1.c_str());
    printf("s0: %d, s1: %d, s0time: %d, s1time: %d\n", s0, s1, s0time, s1time);
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
    return VectorToPose(p0frame1);
}

gtsam::Pose3 LocalizedPoseData::GetP1frame0(){
    return VectorToPose(p1frame0);
}


