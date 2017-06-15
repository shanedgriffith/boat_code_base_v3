/*
 * PreprocessBikeRoute.cpp
 *
 *  Created on: May 1, 2017
 *      Author: shane
 */

#include <VisualOdometry/KLT.hpp>
#include <Visualizations/IMDraw.hpp>
#include <Visualizations/SLAMDraw.h>
#include <FileParsing/ParseSurvey.h>
#include "ParseBikeRoute.hpp"
#include "PreprocessBikeRoute.hpp"

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
    char line[LINESIZE]="";

    //skip the first line.
    fgets(line, LINESIZE-1, fp);
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

void PreprocessBikeRoute::LowPassFilter(std::vector<double>& arr, double std){
    //cedric pointed out this more elegant moving average.
    double alpha = (move_avg_win - 1.)/move_avg_win;
    double yn = arr[0];
    for(int i=0; i<arr.size(); i++){
        yn = alpha * yn + (1-alpha)*arr[i];
        arr[i] = (std::abs(arr[i]-yn)>2*std)?yn:arr[i];
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
    
    double curtime = GetNearestTimeToPosition(0, 0);
    double endtime = GetNearestTimeToPosition(end_pos.x(), end_pos.y());
    int nentries = (int) ceil((endtime-curtime)*video_fps);
    std::vector<double> vtimes(nentries, 0.0);
    std::vector<std::vector<double> > fullset(n, std::vector<double>(nentries, 0.0));
    
    int idx=0;
    //use a buffer at the end due to possible synchronization issues.
    while(curtime < endtime) { //timings[timings.size()-10]
        for(int j=0; j<n; j++) {
            while(timings[lastidxs[j]+intervals[j]] <= curtime)
                lastidxs[j] += intervals[j];
            fullset[j][idx] = InterpolateValue(curtime, arrs[j][lastidxs[j]], arrs[j][lastidxs[j]+intervals[j]],
                                                  timings[lastidxs[j]], timings[lastidxs[j]+intervals[j]]);
        }
        curtime += 1./video_fps;
        vtimes[idx] = curtime;
        idx++;
    }
    std::swap(arrs, fullset);
    std::swap(timings, vtimes);
}

void PreprocessBikeRoute::MakeAux(){
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
    double dt = 1./29;//5;
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
        
        if(i>0){
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
        std::vector<double> R = ComposeRotationMatrices(cam, align_with_world);
        vector<double> RPY = RotationMatrixToRPY(R);
        poses.push_back({arrs[0][i], arrs[1][i], 0.0, RPY[0], RPY[1], RPY[2]}); //res[0], res[1], res[2]});//
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

void PreprocessBikeRoute::Preprocess(){
    ReadDelimitedFile(_bdbase + _name + "/" + _name + ".csv", 0);
    std::cout << "RAW bike route. info: " << timings.size() << " readings. " << std::endl;
    
    std::vector<double> stds = {0,0,0,0.1,0.1,0.1,5.,0,0,0};
    for(int i=0; i<arrs.size(); i++){
        if(stds[i]>0) LowPassFilter(arrs[i], stds[i]);
    }
    
//    ProcessRawVideo(); //run this before cropping.
    
    AlignDataToImages();
    
    //get RPY
    GetPoses();
//    PlayPoses();
//    exit(1);
    
    MakeAux();
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

vector<LandmarkTrack> PreprocessBikeRoute::ProcessNewPoints(int ckey, ParseFeatureTrackFile& pft) {
    vector<LandmarkTrack> inactive;
    int num_landmarks_skipped = 0;
    static int last_skipped = 0;
    int next_entry = 0;
    int lasti=0;
    for(int i=0; i<pft.ids.size(); i++) {
        //remove features that aren't tracked anymore
        //add to the entry using the info from the new frame.
        while(active.size() > next_entry && active[next_entry].GetKey() < pft.ids[i]) {
            if(debug)cout << "Removed landmark " << active[next_entry].key << endl;
            if(active[next_entry].Length()>1) inactive.push_back(active[next_entry]);
            active.erase(active.begin() + next_entry, active.begin() + next_entry + 1);
        }

        //create a new entry if its key is greater than anything that's active
        if(active.size() == next_entry) {
            lasti=i;
            break;
        } else if(pft.ids[i] < active[next_entry].key) {
            if(last_skipped < pft.ids[i]){
                last_skipped = pft.ids[i];
                num_landmarks_skipped++;
            }
            //cout << "ActiveFactors Error: skipped " << pft.ids[i] << endl;
            continue;
        } else if(pft.ids[i] == active[next_entry].key) {
            //accumulate info about the landmark (should be the only remaining case)
            active[next_entry].AddToTrack(pft.time, pft.imagecoord[i], ckey);
            if(debug) cout << "landmark measurement for " << active[next_entry].key << endl;
            next_entry++;
        }
    }

    //add the rest
    srand(time(NULL));
    for(int i=lasti; i<pft.ids.size(); i++) {
        //used to limit the size of the optimization problem for inter-survey optimization
        bool used = true;
        LandmarkTrack lt(pft.ids[i], used);
        lt.AddToTrack(pft.time, pft.imagecoord[i], ckey);
        active.push_back(lt);
    }
    return inactive;
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
        vector<LandmarkTrack> inactive = ProcessNewPoints(counter, PFT);
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











