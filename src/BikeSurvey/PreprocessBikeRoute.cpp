/*
 * PreprocessBikeRoute.cpp
 *
 *  Created on: May 1, 2017
 *      Author: shane
 */

#include <VisualOdometry/VisualOdometry.hpp>
#include <VisualOdometry/KLT.hpp>
#include <Visualizations/IMDraw.hpp>
#include <Visualizations/SLAMDraw.h>
#include <FileParsing/ParseSurvey.h>
#include "Optimization/SingleSession/GTSAMInterface.h"
#include "ParseBikeRoute.hpp"
#include "PreprocessBikeRoute.hpp"

#include <limits>

typedef std::numeric_limits< double > dbl;

using namespace std;

void PreprocessBikeRoute::ProcessLineEntries(int type, vector<string>& lp){
    if(lp.size()<11) return;
    timings.push_back(stod(lp[0]));
    for(int i=1; i<lp.size(); i++){
        if(arrs.size()<i)
            arrs.push_back(std::vector<double>());
        if(i==1) arrs[i-1].push_back(stod(lp[i])-default_start.x());
        else if(i==2) arrs[i-1].push_back(stod(lp[i])-default_start.y());
        else if(i==7) arrs[i-1].push_back(stod(lp[i])*M_PI/180);
        else arrs[i-1].push_back(stod(lp[i]));
    }
}

void PreprocessBikeRoute::ReadDelimitedFile(string file, int type) {
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];

    //skip the first line.
    fgets(line, LINESIZE-1, fp);
    first_line = line;
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

void PreprocessBikeRoute::LowPassFilter(std::vector<double>& arr, double std, int interval){
    //cedric pointed out this more elegant moving average.
    double alpha = (move_avg_win - 1.)/move_avg_win;
    double yn = arr[0];
    for(int i=0; i<arr.size(); i+=interval){
        yn = alpha * yn + (1-alpha)*arr[i];
        arr[i] = (std::abs(arr[i]-yn)>2*std)?yn:arr[i];
        for(int j=i; j<interval; j++)
            arr[j] = arr[i];
    }
}

void PreprocessBikeRoute::applyLowPassFilter() {
    std::vector<int> intervals = {10, 10, 10, 2, 2, 2, 1, 2, 2, 2};
    std::vector<double> stds = {0,0,0,0.1,0.1,0.1,5.,0,0,0};
    for(int i=0; i<arrs.size(); i++){
        if(stds[i]>0) LowPassFilter(arrs[i], stds[i], intervals[i]);
    }
}

void PreprocessBikeRoute::WriteImage(cv::Mat image, string filepath){
    static vector<int> compression_params;
    /*compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);*/
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(99);
    
    try {
        cv::imwrite(filepath.c_str(), image, compression_params);
    } catch (runtime_error& ex) {
        fprintf(stderr, "PreprocessBikeRoute::ReadVideo Error. Exception converting image to JPG format: %s\n", ex.what());
        exit(-1);
    }
}

double PreprocessBikeRoute::GetNearestTimeToPosition(double x, double y){
    double mindist=10000000;
    int mini=0;
    for(int i=0; i<arrs[0].size(); i++){
        double curdist = sqrt(pow(x-arrs[0][i], 2)+pow(y-arrs[1][i], 2));
        if(curdist < mindist) {
            mindist = curdist;
            mini = i;
        }
    }
    return timings[mini];
}

double PreprocessBikeRoute::InterpolateValue(double t, double vals, double vale, double s, double e) {
    return (e-t)/(e-s)*vals + (t-s)/(e-s)*vale;
}

void PreprocessBikeRoute::AlignDataToImages() {
    int n = arrs.size();
    std::vector<int> intervals = {10, 10, 10, 2, 2, 2, 1, 2, 2, 2};
    std::vector<int> lastidxs(n, 0.0);
    
    double starttime = GetNearestTimeToPosition(0, 0);
    double endtime = GetNearestTimeToPosition(end_pos.x(), end_pos.y());
    std::cout.precision(dbl::max_digits10);
    std::cout << "times: " << starttime << " to " << endtime << ", taken from " << timings[0] << ", " << timings[timings.size()-1] << std::endl;
    int nentries = (int) floor((endtime-starttime)*video_fps);
    double inc_time = 1./video_fps;
    std::cout << "number of entries in the aux, and number of sift files: " << nentries << std::endl;
    std::vector<double> vtimes(nentries, 0.0);
    std::vector<std::vector<double> > fullset(n, std::vector<double>(nentries, 0.0));
    
    int idx=0;
    //use a buffer at the end due to possible synchronization issues.
    //while(curtime < endtime) { //timings[timings.size()-10]
    int last_idx=0;
    for(double time = starttime; endtime - time > inc_time; time += inc_time) {
        //double prior_weight = (post time - time) / (post time - prior time);
        //set aux(time)[val] <= prior_weight * aux(prior time)[val] + (1 - prior_weight) * aux(post time)[val];
        while(timings[last_idx] < time) last_idx++;
        last_idx--;
        for(int j=0; j<n; j++) {
            double prior_time = timings[last_idx];
            double post_time = timings[last_idx+intervals[j]];
            double prior_weight = (post_time - time) / (post_time - prior_time);
            fullset[j][idx] = prior_weight * arrs[j][last_idx] + (1 - prior_weight) * arrs[j][last_idx+intervals[j]];
        }

        vtimes[idx] = time;
        idx++;
    }
    std::swap(arrs, fullset);
    std::swap(timings, vtimes);
    vopose.resize(timings.size(), gtsam::Pose3());
    poses.resize(timings.size(), vector<double>());
}

void PreprocessBikeRoute::MakeAux(){
    std::cout.precision(dbl::max_digits10);
    if(timings.size() == 0){
        std::cout << "PreprocessBikeRoute::MakeAux() no data."<<std::endl;
        exit(1);
    }
    
    int idx = 0;
    string saveto = _bdbase + _name + ParseSurvey::_auxfile;
    FILE * fp = OpenFile(saveto, "w");
    for(int i=0; i<timings.size(); i++) {
        fprintf(fp, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", timings[i],
                poses[i][0], poses[i][1], poses[i][2], poses[i][3], poses[i][4], poses[i][5], arrs[9][i]);
    }
    fclose(fp);
    std::cout << "Aux file at: " << saveto << " with " << timings.size() << " lines"<< std::endl;
}

std::vector<double> PreprocessBikeRoute::YPRToRotationMatrix(double y, double p, double r){
    /*Converts ypr to a rotation matrix.
     see http://planning.cs.uiuc.edu/node102.html
     */
    
    std::vector<double> R = {cos(y)*cos(p), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r),
        sin(y)*cos(p), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r),
        -sin(p), cos(p)*sin(r), cos(p)*cos(r)};
    return R;
}

std::vector<double> PreprocessBikeRoute::ComposeRotationMatrices(std::vector<double> A, std::vector<double> B){
    std::vector<double> R = {
        A[0]*B[0]+A[1]*B[3]+A[2]*B[6],
        A[0]*B[1]+A[1]*B[4]+A[2]*B[7],
        A[0]*B[2]+A[1]*B[5]+A[2]*B[8],
        A[3]*B[0]+A[4]*B[3]+A[5]*B[6],
        A[3]*B[1]+A[4]*B[4]+A[5]*B[7],
        A[3]*B[2]+A[4]*B[5]+A[5]*B[8],
        A[6]*B[0]+A[7]*B[3]+A[8]*B[6],
        A[6]*B[1]+A[7]*B[4]+A[8]*B[7],
        A[6]*B[2]+A[7]*B[5]+A[8]*B[8]};
    return R;
}

std::vector<double> PreprocessBikeRoute::RotationMatrixToRPY(std::vector<double> R){
    std::vector<double> rpy = {atan2(R[7], R[8]), atan2(-1*R[6], sqrt(R[7]*R[7]+R[8]*R[8])), atan2(R[3],R[0])};
    return rpy;
}

double PreprocessBikeRoute::CombineAngles(double a1, double a2, double w){
    while(a1<-M_PI) a1 += 2*M_PI;
    while(a2<-M_PI) a2 += 2*M_PI;
    while(a1>M_PI) a1 -= 2*M_PI;
    while(a2>M_PI) a2 -= 2*M_PI;
    
    double m1 = min(a1, a2);
    double m2 = max(a1, a2);
    if(m1==a2) w = 1-w;
    
    if(abs(m2-m1)>M_PI){
        if(m1<0) m1 += 2*M_PI;
        else m2 -= 2*M_PI;
    }
    
    return (1-w)*m1 + w*m2;
}

void PreprocessBikeRoute::GetPoses() {
    //see p.9 of 'Tilt Sensing Using a Three-Axis Accelerometer' for these equations.
    //and this post: https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
    //Complementary filter. https://theboredengineers.com/2012/09/19/the-quadcopter-get-its-orientation-from-sensors/
    //initial gyro position?
    //maybe set this manually using the accel reading, captured non-moving, in the same orientation while riding.
    //although, does that actually correspond to the gyroscope for the current run?
    //apparently arbitrary is ok, how?
    //rather than integrate for gyro using only gyro values, why not sum each value individually to the previous one?
    //(which should get rid of the bad initial estimate)
    //IMU data procedure: http://www.starlino.com/imu_guide.html
    //do I have to apply a transform to the data?
    //assume it starts at the manually measured angle, and these values measure the deviation from that.
    //
    if(arrs[0].size() == 0){
        std::cout <<"PreprocessBikeRoute::GetPoses() error. Array size: " << arrs[0].size() << std::endl;
        exit(1);
    }
    double r=0,p=0,y=arrs[6][0];
    double dt = 1./video_fps;
    double mu = 0.1;
    double weight = 0.98;
    std::vector<double> res(3,0);
    for(int i=0; i<arrs[0].size(); i++){
        //data to angles, then use the complementary filter to combine angles from different sources.
        //dt with accel?
        //seems to depend on Rxyz or Ryxz; there are six variations.
        
        double r_acc = sqrt(pow(arrs[3][i],2.)+pow(arrs[4][i],2.)+pow(arrs[5][i],2.));
        std::vector<double> n_acc = {arrs[3][i]/r_acc, arrs[4][i]/r_acc, arrs[5][i]/r_acc};
        std::vector<double> acc_angles(2,0.);
        acc_angles[0] = atan(n_acc[1]/((n_acc[2]/abs(n_acc[2]))*sqrt(pow(n_acc[0],2.)+pow(n_acc[2],2.))));
        acc_angles[1] = atan2(-1.*n_acc[0],sqrt(pow(n_acc[1],2.)+pow(n_acc[2],2.)));
        double cam_yaw_from_compass = -1 * arrs[6][i]-M_PI_2;
        res[2] = cam_yaw_from_compass;
        
        if(i>0) {
            double dx = arrs[0][i] - arrs[0][i-1];
            double dy = arrs[1][i] - arrs[1][i-1];
            double d = sqrt(pow(dx,2.)+pow(dy,2.));
            if(d>0){
                dx /= d;
                dy /= d;
                
                double cam_yaw_from_gps = -atan2(dx,dy);
                double weight = arrs[2][i]/(1+arrs[2][i]);
                res[2] = CombineAngles(cam_yaw_from_compass, cam_yaw_from_gps, weight);
            }
            
            for(int i=0; i<2; i++)
                res[i] = acc_angles[i]*(1-weight) + (res[i]+arrs[i+7][i]*dt)*weight;
        } else {
            res = acc_angles;
        }
        
        //a height of 0 is inaccurate, but optimization may be able to compensate.
        vector<double> cam = YPRToRotationMatrix(res[2], res[1], res[0]);
        vector<double> align_with_world = YPRToRotationMatrix(-M_PI_2, 0, M_PI_2);
        vector<double> R = ComposeRotationMatrices(cam, align_with_world);
        vector<double> RPY = RotationMatrixToRPY(R);
        std::vector<double> curpose = {arrs[0][i], arrs[1][i], 0.0, RPY[0], RPY[1], RPY[2]}; //todo: convert to matrix format.
        poses[i] = curpose;
    }
}

void PreprocessBikeRoute::PlayPoses(){
    //unit test the poses.
    
    SLAMDraw art;
    art.SetScale(-2000,400,-500,1500);
    art.ResetCanvas();
    
    for(int i=0; ; i=i+video_fps){
        if(i>=poses.size()) i -= video_fps;
        printf("%lf,%lf,%lf,%lf,%lf\n",poses[i][0],poses[i][1],poses[i][3]*180/M_PI,poses[i][4]*180/M_PI,poses[i][5]*180/M_PI);
        for(int j=0; j<=i; j++)
            art.AddShape(SLAMDraw::shape::CIRCLE, poses[j][0], poses[j][1], 0, 0, 0);
        art.DrawSight(poses[i][0], poses[i][1], poses[i][3], 0.838, 255, 0, 0);
        art.DrawSight(poses[i][0], poses[i][1], poses[i][4], 0.838, 0, 255, 0);
        art.DrawSight(poses[i][0], poses[i][1], poses[i][5], 0.838, 0, 0, 255);
        char c = art.Display();
        if(c==83) i = (i>=2*video_fps)?i-2*video_fps:i-video_fps;
        art.ResetCanvas();
    }
}

void PreprocessBikeRoute::Preprocess2() {
    ReadDelimitedFile(_bdbase + _name + "/" + _name + ".csv", 0);
    std::cout << "RAW bike route. info: " << timings.size() << " readings. " << std::endl;
    int num_video_frames = 20824; //countVideoFrames(); //TODO CHANGE BACK..
    double elapsed_time = timings[timings.size()-1] - timings[0];
    video_fps = num_video_frames / elapsed_time;
    
    AlignDataToImages();
    
    string saveto = _bdbase + _name + "/cropped_aux_file.csv";
    FILE * fp = OpenFile(saveto, "w");
    fprintf(fp, "%s", first_line.c_str());
    for(int i=0; i<timings.size(); i++) {
        fprintf(fp, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", timings[i],
                arrs[0][i], arrs[1][i], arrs[2][i], arrs[3][i], arrs[4][i],
                arrs[5][i], arrs[6][i], arrs[7][i], arrs[8][i], arrs[9][i]);
    }
    fclose(fp);
    std::cout << "Aux file at: " << saveto << " with " << timings.size() << " lines"<< std::endl;
    
    GetPoses();
    
    VOForCameraTrajectory();
    
    correctPosesUsingVO();
    
    //write poses and vo
    saveto = _bdbase + _name + "/poses_and_vo.csv";
    fp = OpenFile(saveto, "w");
    
    for(int i=0; i<timings.size(); i++) {
        std::vector<double> vodom = GTSAMInterface::PoseToVector(vopose[i]); //todo, save different format.
        fprintf(fp, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", timings[i],
                poses[i][0], poses[i][1], poses[i][2], poses[i][3], poses[i][4], poses[i][5],
                vodom[0], vodom[1], vodom[2], vodom[3], vodom[4], vodom[5]);
    }
    fclose(fp);
    std::cout << "Preprocess file at: " << saveto << " with " << timings.size() << " lines"<< std::endl;
}

void PreprocessBikeRoute::Preprocess(){
    ReadDelimitedFile(_bdbase + _name + "/" + _name + ".csv", 0);
    std::cout << "RAW bike route. info: " << timings.size() << " readings. " << std::endl;
    
    applyLowPassFilter();
    
//    ProcessRawVideo(); //run this before cropping.
    
    AlignDataToImages();
    
    //get RPY
    GetPoses();
    ModifyPoses();
//    PlayPoses();
//    exit(1);
    
    MakeAux();
}

int PreprocessBikeRoute::countVideoFrames() {
    std::cout << "..counting video frames.." << std::endl;
    string videofile = _bdbase + _name + "/" + _name + ".mp4";
    cv::VideoCapture vid(videofile);
    if(!vid.isOpened()){
        std::cout << "ProcessRawVideo::ReadVideo Error. Couldn't open " << videofile << std::endl;
        exit(-1);
    }
    
    int count=0;
    while(vid.grab()){
        count++;
    }
    std::cout << "video has " << count << " frames" << std::endl;
    double elapsed_time = timings[timings.size()-1] - timings[0];
    std::cout << "frame rate: " << count / elapsed_time << std::endl;
    return count;
}

void PreprocessBikeRoute::ProcessRawVideo(){
    string videofile = _bdbase + _name + "/" + _name + ".mp4";
    string facedir = "/cs-share/dream/facedetection";

    cv::VideoCapture vid(videofile);
    if(!vid.isOpened()){
        std::cout << "ProcessRawVideo::ReadVideo Error. Couldn't open " << videofile << std::endl;
        exit(-1);
    }
    
    ImageModification blurfaces(facedir); //directory with the xml files.
    
    Camera cam = ParseBikeRoute::GetCamera();
    KLT k(cam);
    std::vector<double> KLTparms = {18., 36., 600., 4., 100., 100., 100., 9., 125., 40.};
    k.SetSizes(KLTparms);
    
    double starttime = GetNearestTimeToPosition(0,0);
    double endtime = GetNearestTimeToPosition(end_pos.x(), end_pos.y());
    int timingidx = 0;
    int nimages = 0;
    int savedimages = 0;
    while(vid.grab()){
        cv::Mat image;
        if(!vid.retrieve(image)){
            std::cout << "couldn't retrive the image." << std::endl;
            exit(-1);
        }
        
        double imtime = timings[0] + (1.*nimages)/video_fps;
        nimages++;
        if(imtime < starttime) continue;
        if(imtime >= endtime) break;
        
        ParseFeatureTrackFile PFT = k.TrackKLTFeatures(image, _bdbase + _name, savedimages, imtime);
        blurfaces.DetectFacesAndBlur(image);
        string imagepath = ParseSurvey::GetImagePath(_bdbase + _name, savedimages++, true);
        WriteImage(image, imagepath);
        PFT.WriteFeatureTrackFile();
        
        std::cout << "saving image " <<nimages << ":" << imagepath << std::endl;
        std::cout << "saving pft as " << PFT.siftfile << std::endl;
    }

    std::cout << "saved " << savedimages << " images from the video file." << std::endl;
}

void PreprocessBikeRoute::FindKLTParams(){
    bool show = true;
    if(show) cv::namedWindow("klt points");
    
    Camera cam = ParseBikeRoute::GetCamera();
    string videofile = _bdbase + _name + "/" + _name + ".mp4";
    
    cv::VideoCapture vid(videofile);
    if(!vid.isOpened()){
        std::cout << "PreprocessBikeRoute::ReadVideo Error. Couldn't open " << videofile << std::endl;
        exit(-1);
    }
    
    KLT k(cam);
    //long tracks, but they get pulled along by occlusions, and aren't spread out well.
    //std::vector<double> parms = {30., 45., 600., 20., 400., 400., 100., 9., 125., 40.};
    
    //features aren't as susceptible to dragging, but they still do, and they're not well spread throughout the scene.
    //std::vector<double> parms = {30., 45., 600., 20., 30., 30., 100., 9., 125., 40.};
    
    //works fairly well with the smaller grid size in reducing feature wiping by occluders, but can't handle larger transitions.
    //some points seem to drift.
    //std::vector<double> parms = {18., 36., 850., 4., 30., 30., 100., 9., 125., 40.};
    std::vector<double> KLTparms = {18., 36., 600., 4., 100., 100., 100., 9., 125., 40.};
    k.SetSizes(KLTparms);
    for(int i=0; i<KLTparms.size(); i++) {
        std::cout << KLTparms[i] << ", ";
    } std::cout << std::endl;
    
    int m_track = 5;
    double sumall=0;
    int count = 0;
    int start = 11829;//8475;//6400;
    int counter = 0;
    while(vid.grab()) {
        cv::Mat image;
        if(!vid.retrieve(image)) {
            std::cout << "couldn't retrieve the image." << std::endl;
            exit(-1);
        }
        counter++;
        if(counter < start) continue;
        std::cout << std::endl;
        
        ParseFeatureTrackFile PFT = k.TrackKLTFeatures(image, _bdbase + _name, 0, 0);
        vector<LandmarkTrack> inactive = PFT.ProcessNewPoints((int) 'x', counter, active);
        if(inactive.size() > 0) {
            int suml = 0;
            int minl = 10000000;
            int maxl = 0;
            int gttrack = 0;
            for(int i=0; i<inactive.size(); i++) {
                suml += inactive[i].points.size();
                minl = std::min(minl, (int) inactive[i].points.size());
                maxl = std::max(maxl, (int) inactive[i].points.size());
                gttrack = (inactive[i].points.size()>m_track)?gttrack+1:gttrack;
            }
            sumall += suml;
            count += inactive.size();
            std::cout << "KLT track "<<counter<<": " << 1.0*suml/inactive.size() << ", [" << minl << ", " << maxl << "] all avg: " << 1.0*sumall/count << ", num > " << gttrack << " of " << inactive.size() << std::endl;
        }
        
        if(show){
            IMDraw art(image);
            for(int j=0; j<PFT.ids.size(); j++){
                art.DrawPoint(PFT.imagecoord[j].x(), PFT.imagecoord[j].y(), PFT.ids[j]);
            }
            cv::imshow("klt points", image);
            char c = cvWaitKey(30);
            if(c == 'q') exit(1);
        }
    }
    
    if(show) cv::destroyWindow("klt points");
}

void PreprocessBikeRoute::Play(){
    bool with_klt = true;
    cv::namedWindow("klt points");
    
    Camera cam = ParseBikeRoute::GetCamera();
    
    for(int i=0; i<timings.size(); i++){
        string impath = ParseSurvey::GetImagePath(_bdbase + _name, i);
        cv::Mat image = cv::imread(impath.c_str());
        string ftfname = ParseFeatureTrackFile::GetFeatureTrackFilePath(_bdbase + _name, i);
        ParseFeatureTrackFile PFT(cam, ftfname);

        if(with_klt){
            IMDraw art(image);
            for(int j=0; j<PFT.ids.size(); j++)
                art.DrawPoint(PFT.imagecoord[j].x(), PFT.imagecoord[j].y(), PFT.ids[j]);
        }

        cv::imshow("klt points", image);
        char c = cvWaitKey(30);
        if(c == 'q') break;
    }

    cv::destroyWindow("klt points");
}


bool PreprocessBikeRoute::DistanceCriterion(std::vector<double>& pose1, std::vector<double>& pose2){
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

std::vector<double> PreprocessBikeRoute::InterpolatePoses(int idx, int a, int b, vector<double> pa, vector<double> pb){
    std::vector<double> p(pa.size(), 0);
    for(int i=0; i<pa.size(); i++){
        double w2 = (idx - 1.0*a)/(b-a);
        double w1 = 1 - w2;
        p[i] = pa[i] * w1 + pb[i] * w2;
    }
    return p;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void PreprocessBikeRoute::SetZ(std::vector<double>& unkz, std::vector<double> nmlzd){
    unkz[2] = sgn(nmlzd[2]) * sqrt((pow(unkz[0],2)+pow(unkz[1],2))/(1/pow(nmlzd[2],2) - 1));
    unkz[3] = nmlzd[3];
    unkz[4] = nmlzd[4];
    unkz[5] = nmlzd[5];
}

void PreprocessBikeRoute::ModifyPoses() {
    
    /*
     another method:
      >convert the unit translation from VO to the correct one:
       z = sgn(x3) * \sqrt (x^2 + y^2)/(1/x3^2 - 1)
     >>this might rely too much on ..hmm
     */
    Camera nexus = ParseBikeRoute::GetCamera();
    VisualOdometry vo(nexus);
    vector<double> lastp;
    vector<vector<double>> filtered;
    filtered.push_back(poses[0]);
    vector<int> indices = {0};
    std::shared_ptr<ParseFeatureTrackFile> PFT0 = std::make_shared<ParseFeatureTrackFile>(nexus, _bdbase + _name, 0);
    
    vector<double> curpose;
    for(int i=2; i<timings.size(); i=i+2){
        std::shared_ptr<ParseFeatureTrackFile> PFT1 = std::make_shared<ParseFeatureTrackFile>(nexus, _bdbase + _name, i);
        std::pair<gtsam::Pose3, std::pair<double, int> > vop = vo.PoseFromEssential(PFT0, PFT1);
        vector<double> vp = GTSAMInterface::PoseToVector(vop.first);
        if(i>2){
            bool smooth = DistanceCriterion(vp, lastp);;
            while(!smooth){
                std::cout << "skipped " << i << std::endl;
                PFT1->Next(++i);
                if(PFT1->time==-1) break; //this will add a bad pose to the end. problem?
                std::pair<gtsam::Pose3, std::pair<double, int> > vop = vo.PoseFromEssential(PFT0, PFT1);
                vp = GTSAMInterface::PoseToVector(vop.first);
                smooth = DistanceCriterion(vp, lastp);
            }
        }
        //printf("pose %d from vo (%lf,%lf,%lf,%lf,%lf,%lf)\n",i,vp[0],vp[1],vp[2],vp[3],vp[4],vp[5]);
        if(i==2) curpose = vp;
        else {
            gtsam::Pose3 lip = GTSAMInterface::VectorToPose(lastp);
            gtsam::Pose3 vip = GTSAMInterface::VectorToPose(vp);
            gtsam::Pose3 cip = lip.compose(vip);
            curpose = GTSAMInterface::PoseToVector(cip);
        }
        filtered.push_back(curpose);
        indices.push_back(i);
        PFT0 = PFT1;
        lastp = vp;
    }
    
    vector<vector<double>> newposes(timings.size(), vector<double>());
    for(int i=0, c=0; i<timings.size(); i++){
        while(indices.size() > c && indices[c] <= i) c++;
        if(indices.size() <= c){
            //hmm, but this may cause issues like before with the duplicated GPS.
            std::cout << i << " using duplicated pose" << std::endl;
            newposes[i] = newposes[i-1];
        }else newposes[i] = InterpolatePoses(i, indices[c-1], indices[c], filtered[c-1], filtered[c]);
        SetZ(poses[i], newposes[i]);
        printf("pose %d from vo (%lf,%lf,%lf,%lf,%lf,%lf)\n",i,poses[i][0],poses[i][1],poses[i][2],poses[i][3],poses[i][4],poses[i][5]);
    }
    
    poses = newposes;
    if(poses.size() != arrs[0].size()){
        std::cout << "size incompatibility" << std::endl;
    }
}

gtsam::Pose3 PreprocessBikeRoute::interpolatePose(double weight_a, gtsam::Pose3 a, gtsam::Pose3 b, bool unit_translation) {
    if(weight_a < 0 or weight_a > 1) {
        std::cout << "weight should be 0-1 " << std::endl;
        exit(-1);
    }
    gtsam::Point3 interpt = a.translation() * weight_a + b.translation() * (1-weight_a);
    if(unit_translation)
        interpt /= interpt.norm();
    gtsam::Rot3 interpr = b.rotation().slerp(weight_a, a.rotation());
    return gtsam::Pose3(interpr, interpt);
}

void PreprocessBikeRoute::VOForCameraTrajectory() {
    double MIN_KEYPOINT_TRACKS = 10;
    double MIN_INLIERS = 10;
    double MIN_RATIO_INLIERS = 0.25;
    Camera nexus = ParseBikeRoute::GetCamera();
    VisualOdometry vo(nexus);
    
    int lasti = 0;
    std::shared_ptr<ParseFeatureTrackFile> last;
    gtsam::Pose3 identity();
    
    const double alpha = (move_avg_win - 1.)/move_avg_win;
    const double std = 3;
    
    std::vector<bool> hasvo(vopose.size(), false);
    hasvo[0] = true;
    double move_avg_rerror = 5;
    for(int i=0; i<timings.size(); i++) {
        std::shared_ptr<ParseFeatureTrackFile> cur = std::make_shared<ParseFeatureTrackFile>(nexus, _bdbase + _name, i);
        if(cur->time <= 0) {
            std::cout << "CHECK. no tracking for pftf idx " << i <<". Yet, have " << timings.size() - i << " timings left. " << std::endl;
            break;
        }
        if(fabs(cur->time - timings[i])>0.001) {
            std::cout.precision(17);
            std::cout << "timing is off. rewriting." << std::endl;
            std::cout << i << " : pft file:  " << cur->time << ", " << timings[i] << std::endl;
            cur->time = timings[i];
            cur->WriteFeatureTrackFile();
        }
        if(last and last->time > 0)
        {
            std::pair<double, int> dist = vo.KeypointChange(last, cur);
            if(dist.second < MIN_KEYPOINT_TRACKS) {
                std::cout << "too few tracks" << std::endl;
                last = cur;
                lasti = i;
                continue;
            }
            std::cout << "disparity: " << dist.first << std::endl;
            if(dist.first < 50) //larger disparity appears to produce better rotation matrices.
                continue;
            std::pair<gtsam::Pose3, std::pair<double, int> > res;
            if(i < 100) {
                res = vo.PoseFromEssential(last, cur);
                if(res.first.translation().norm() > 1.001 ) {
                    std::cout <<"iteration " << i << std::endl;
                    std::cout << "issue with vo. norm: " << res.first.translation().norm() <<  std::endl;
                    std::cout << "translation() " << res.first.translation() << std::endl;
                    exit(-1);
                }
            } else {
                break;
                /*
                gtsam::Point3 unnorm(i, i, 0); //DUMMY VO VALS.
                gtsam::Point3 norm = unnorm / unnorm.norm();
                res.first = gtsam::Pose3(gtsam::Rot3(), norm);
                res.second = std::make_pair(1.0, 500);*/
            }
            move_avg_rerror = alpha * move_avg_rerror + (1-alpha)*res.second.first;
            bool passed_rerror_check = (std::abs(res.second.first-move_avg_rerror)>2.*std)?false:true;
            if(not passed_rerror_check or res.second.second < MIN_INLIERS or res.second.second < dist.second * MIN_RATIO_INLIERS) {
                std::cout << "vo failed on frame " << i << std::endl;
                std::cout << "reprojection error: " << res.second.first << ", inliers: " << res.second.second << std::endl;
                last = cur;
                lasti = i;
                continue;
            }
            gtsam::Pose3 delta_pose = res.first;
            
            //distribute the value over the range vo was calculated.
            //TODO: check this. Is this method of distributing the value correct?
            //interpolate between identity and this transformation.
            double s = 1.0/(i-lasti);
            for(int j=i; j>lasti; j--) {
                vopose[j] = interpolatePose(s, delta_pose, gtsam::Pose3(), true);
                hasvo[j] = true;
//                std::cout << j << ": " << vopose[j] << std::endl;
            }
        }
        
        last = cur;
        lasti = i;
    }
    
    //interpolation to fill in any empty vo.
    int li = -1;
    int curi = 0;
    while(curi < vopose.size()) {
        if(not hasvo[curi]) {
            int nexti = curi;
            while(nexti < hasvo.size() and not hasvo[++nexti]);
            for(int i=curi; i<nexti; i++) {
                if(nexti > hasvo.size()) {
                    vopose[curi] = interpolatePose(1, vopose[li], vopose[li], true); //may want to change to identity().
                    hasvo[curi] = true;
                } else if(li == -1) {
                    vopose[curi] = interpolatePose(1, vopose[nexti], vopose[nexti], true);
                    hasvo[curi] = true;
                } else {
                    double prior_weight = (1.0 * nexti - curi) / (nexti - li);
                    vopose[curi] = interpolatePose(prior_weight, vopose[li], vopose[nexti], true);
                    hasvo[curi] = true;
                }
                
                //set the translation to the unit translation.
                vopose[curi] = gtsam::Pose3(vopose[curi].rotation(), vopose[curi].translation() / vopose[curi].translation().norm());
            }
            
            li = nexti;
        }
        else li = curi;
        curi = li + 1;
    }
}

//gtsam::Vector3 scaleTranslation(double s, gtsam::Vector3 trans) {
//    
//}

void
PreprocessBikeRoute::
correctPosesUsingVO()
{
    //X) poses doesn't have z.
    //X) vopose isn't scaled.
    double lastz = 0.;
    std::vector<double> z(poses.size(),0);
    for(int i=0; i<poses.size()-1; i++) {
        //poses[i+1] = poses[i].compose(vopose[i+1]);
        // ::
        // compose:  Pose3(R_ * T.R_, t_ + R_ * T.t_);
        gtsam::Pose3 vo = vopose[i+1];
        gtsam::Pose3 Pi = GTSAMInterface::VectorToPose(poses[i]);
        gtsam::Pose3 Pj = GTSAMInterface::VectorToPose(poses[i+1]);
        gtsam::Pose3 btwn = Pi.between(Pj);
        gtsam::Vector3 votrans = Pi.rotation() * vo.translation();
        
        //get z
        double W = pow(pow(btwn.translation().x(),2) + pow(btwn.translation().y(),2),0.5);
        double c = pow(pow(votrans.x(),2) + pow(votrans.y(),2),0.5);
        int sign = sgn(votrans.x())*sgn(votrans.y())*sgn(btwn.translation().x())*sgn(btwn.translation().y());
        double delta_z = sign * pow(pow(W/c,2)-pow(W,2), 0.5);
        if(c > 1.0) {
            delta_z = 0;
            
            if(c > 1.01)
            {
                std::cout << "issue with pose from essential, norm(), or interpolation.. " << std::endl;
                std::cout << "W: " << W << ", c: " << c << ", W/c: " << W/c << ", " << pow(W/c,2)-pow(W,2) << std::endl;
                std::cout << "c is larger than norm. " << c << std::endl;
                std::cout << "vo: " << vo << std::endl;
                std::cout << "vo norm: " << vo.translation().norm() << std::endl;
                exit(-1);
            }
        }
        z[i+1] = z[i] + delta_z;
        
        //scale vopose
        double s = pow(pow(btwn.translation().x(),2) + pow(btwn.translation().y(),2) + pow(delta_z,2), 0.5);
        gtsam::Vector3 votscaled = vo.translation() * s;
        vopose[i+1] = gtsam::Pose3(vopose[i+1].rotation(), votscaled);
    }
    
    for(int i=0; i<poses.size(); i++) {
        poses[i][2] = z[i];
    }
}




