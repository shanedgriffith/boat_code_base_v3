/*
 * ParseBikeRoute.cpp
 *
 *  Created on: May 1, 2017
 *      Author: shane
 */


#include "PreprocessBikeRoute.hpp"

#include <VisualOdometry/KLT.hpp>
#include <Visualizations/IMDraw.hpp>
#include <Visualizations/SLAMDraw.h>

using namespace std;

const string PreprocessBikeRoute::_auxfile = "/image_auxilliary.csv";

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
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    
    try {
        cv::imwrite(filepath.c_str(), image, compression_params);
    } catch (runtime_error& ex) {
        fprintf(stderr, "ParseBikeRoute::ReadVideo Error. Exception converting image to PNG format: %s\n", ex.what());
        exit(-1);
    }
}

string PreprocessBikeRoute::GetImagePath(int number, bool makepath){
    char imagefile[100];
    if(makepath){
        if(number%1000==0) {
            if(number == 0) {
                sprintf(imagefile, "%s%s/images/", _bdbase.c_str(), _name.c_str());
                FileParsing::MakeDir(imagefile);
            }
            sprintf(imagefile, "%s%s/images/%04d/", _bdbase.c_str(), _name.c_str(), number/1000);
            FileParsing::MakeDir(imagefile);
        }
    }
    sprintf(imagefile, "%s%s/images/%04d/%04d.png", _bdbase.c_str(), _name.c_str(), number/1000, number%1000);
    return imagefile;
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
    std::vector<std::vector<double> > fullset(n, std::vector<double>());
    
    double curtime = GetNearestTimeToPosition(0, 0);
    double endtime = GetNearestTimeToPosition(end_pos.x(), end_pos.y());
    
    //use a buffer at the end due to possible synchronization issues.
    while(curtime < endtime) { //timings[timings.size()-10]
        for(int j=0; j<n; j++) {
            while(timings[lastidxs[j]+intervals[j]] <= curtime)
                lastidxs[j] += intervals[j];
            fullset[j].push_back(InterpolateValue(curtime, arrs[j][lastidxs[j]], arrs[j][lastidxs[j]+intervals[j]],
                                                  timings[lastidxs[j]], timings[lastidxs[j]+intervals[j]]));
        }
        curtime += 1./video_fps;
    }
    std::swap(arrs, fullset);
}

void PreprocessBikeRoute::MakeAux(){
    if(timings.size() == 0){
        std::cout << "ParseBikeRoute::MakeAux() no data."<<std::endl;
        exit(1);
    }
    double curtime = GetNearestTimeToPosition(0, 0);
    double endtime = GetNearestTimeToPosition(end_pos.x(), end_pos.y());
    
    int idx = 0;
    string saveto = _bdbase + _name + "/" + _auxfile;
    FILE * fp = OpenFile(saveto, "w");
    while(curtime < endtime) {
        fprintf(fp, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", curtime,
                poses[idx][0], poses[idx][1], poses[idx][2], poses[idx][3], poses[idx][4], poses[idx][5], arrs[9][idx]);
        curtime += 1./video_fps;
        idx++;
    }
    fclose(fp);
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
        std::vector<double> acc_angles(3,0.);
        acc_angles[2] = arrs[6][i]*M_PI/180 - M_PI;
        acc_angles[0] = atan(n_acc[1]/((n_acc[2]/abs(n_acc[2]))*sqrt(pow(n_acc[0],2.)+pow(n_acc[2],2.))));
        acc_angles[1] = atan2(-1.*n_acc[0],sqrt(pow(n_acc[1],2.)+pow(n_acc[2],2.)));
        
        if(i>0){
            double dx = arrs[0][i] - arrs[0][i-1];
            double dy = arrs[1][i] - arrs[1][i-1];
            double d = sqrt(pow(dx,2.)+pow(dy,2.));
            if(d>0){
                dx /= d;
                dy /= d;
                acc_angles[2] = (acc_angles[2]  + arrs[2][i]*atan2(dx,dy))/(1+arrs[2][i]);
            }
            
            for(int i=0; i<3; i++)
                res[i] = acc_angles[i]*(1-weight) + (res[i]+arrs[i+7][i]*dt)*weight;
        } else {
            res = acc_angles;
        }
        
        //a height of 0 is inaccurate, but optimization may be able to compensate.
        poses.push_back({arrs[0][i], arrs[1][i], 0.0, acc_angles[0], acc_angles[1], acc_angles[2]});
    }
}

void PreprocessBikeRoute::PlayPoses(){
    //unit test the poses.
    
    SLAMDraw art;
    art.SetScale(-2000,2000,-2000,2000);
    art.ResetCanvas();
    
    for(int i=0; i<poses.size(); i=i+video_fps){
        std::cout << i<<":"<<poses[i][0] << ", " << poses[i][1] << ", " << poses[i][5] << std::endl;
        art.DrawSight(poses[i][0], poses[i][1], poses[i][5]);
        art.Display();
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
    
    ProcessRawVideo(); //run this before cropping.
    
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
    
    Camera cam = PreprocessBikeRoute::GetCamera();
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
        
        double imtime = timings[0] + nimages/video_fps;
        nimages++;
        if(imtime < starttime) continue;
        if(imtime >= endtime) break;
        
        ParseFeatureTrackFile PFT = k.TrackKLTFeatures(image, _bdbase + _name, savedimages, imtime);
        blurfaces.DetectFacesAndBlur(image);
        string imagepath = GetImagePath(savedimages++, true);
        WriteImage(image, imagepath);
        PFT.WriteFeatureTrackFile();
        
        std::cout << "saving image " <<nimages << ":" << imagepath << std::endl;
        std::cout << "saving pft as " << PFT.siftfile << std::endl;
    }

    std::cout << "read " << savedimages << " images from the file." << std::endl;
}

Camera PreprocessBikeRoute::GetCamera(){
    Camera nexus(1206.41699, 1205.09164, 636.766777, 371.147712, 1280, 720);
    nexus.SetDistortion(0.0817643033, 0.682168738, 0.00118562419, -0.00100627619, 0.0); //assume k3 is unused???
    //0.0817643033, 0.682168738, 0.00118562419, -0.00100627619, -7.05688254
    return nexus;
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
    
    Camera cam = PreprocessBikeRoute::GetCamera();
    string videofile = _bdbase + _name + "/" + _name + ".mp4";
    
    cv::VideoCapture vid(videofile);
    if(!vid.isOpened()){
        std::cout << "ParseBikeRoute::ReadVideo Error. Couldn't open " << videofile << std::endl;
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
            std::cout << "couldn't retrive the image." << std::endl;
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
    
    Camera cam = PreprocessBikeRoute::GetCamera();
    
    for(int i=0; i<timings.size(); i++){
        string impath = GetImagePath(i);
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











