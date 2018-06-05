//
//  ParseFeatureTrackFile.cpp
//  VisualizationCode
//
//  Created by Shane Griffith on 6/10/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "ParseFeatureTrackFile.h"


using namespace std;

void ParseFeatureTrackFile::ProcessLineEntries(int type, vector<string> lp){
    vector<string> e = ParseLineAdv((char *)lp[3].c_str(), "[];");
    time = stod(lp[1]);
    ids.push_back(stoi(e[0]));
    double x = stod(e[1]), y=stod(e[2]);
    if(x < 0) x=0;
    if(y < 0) y=0;
    if(x >= _cam.w()) x = _cam.w()-1;
    if(y >= _cam.h()) y = _cam.h()-1;
    imagecoord.push_back(gtsam::Point2(x, y));
}

/*
void ParseFeatureTrackFile::ReadDelimitedFile(string file, int type) {
    char line[LINESIZE];
    char l3[LINESIZE];
    int entry;
    int id;
    double x, y;
    double fill1, fill2;
    
    bool success = false;
    while(!success) {
        success = true;
        FILE * fp = OpenFile(file,"r");
        if(fp==NULL){
            printf("ParseFeatureTrackFile: NULL file %s", file.c_str());
            sleep(1);
            success = false;
            continue;
        }
        
        fgets(line, LINESIZE-1, fp); //do nothing with the first line.
        while (fgets(line, LINESIZE-1, fp)) {
            int ret = sscanf(line,"%d,%lf,%[^,],[%d;%lf;%lf;%lf;%lf]",
                             &entry, &time, (char *) l3, &id, &x, &y, &fill1, &fill2);
            if (ret!=8) {
                success = false;
                cout << "Problematic file: " << file << ". Note: if using NFS, try unmounting and remounting." << endl;
                printf("ParseFeatureTrackFile: Parsed %d of8 arguments of '%s' of file %s\nFull line:%s", ret, l3, file.c_str(), line);
                sleep(1);
                ids.clear();
                imagecoord.clear();
                break;
            }
            if(x < 0) x=0;
            if(y < 0) y=0;
            if(x >= _cam.w()) x = _cam.w()-1;
            if(y >= _cam.h()) y = _cam.h()-1;
            ids.push_back(id);
            imagecoord.push_back(gtsam::Point2(x, y));
            //cout << "line: "<<time << ", "<<id << ", "<<x<<", "<<y<<endl;
        }
        if(fp != NULL)
            fclose(fp);
    }
    //cout <<"Finished reading file: "<<file.c_str() <<endl;
}
 */

//This version is less buggy, in some way due to lack of reliance on string, which uses dynamic memory.
void ParseFeatureTrackFile::ReadDelimitedFile(string file, int type) {
    char line[LINESIZE];
    char l3[LINESIZE];
    int entry;
    int id;
    double x, y;
    double fill1, fill2;
    
    bool success = false;
    while(!success){
        success = true;
        FILE * fp = OpenFile(file,"r");
        if(fp==NULL){
            printf("ParseFeatureTrackFile: NULL file %s", file.c_str());
            sleep(1);
            success = false;
            continue;
        }
        
        fgets(line, LINESIZE-1, fp);
        int ret = sscanf(line, "%lf", &time);
        if(ret!=1){
            cout << "ParseFeatureTrackFile: Couldn't read the time of file: "<< file << endl;
            sleep(1);
            continue;
        }
        
        while (fgets(line, LINESIZE-1, fp)) {
            int id;
            float x, y;
            int ret = sscanf(line,"%d, %e, %e", &id, &x, &y);
            
            if (ret!=3) {
                success = false;
                cout << "Problematic file: " << file << ". Note: if using NFS, try unmounting and remounting." << endl;
                printf("ParseFeatureTrackFile: Parsed %d of 3 arguments of file %s\nFull line:%s", ret, file.c_str(), line);
                sleep(1);
                ids.clear();
                imagecoord.clear();
                break;
            }
            if(x < 0) x=0;
            if(y < 0) y=0;
            if(x >= _cam.w()) x = _cam.w()-1;
            if(y >= _cam.h()) y = _cam.h()-1;
            if(id==0 && x==0 && y==0) continue;
            ids.push_back(id);
            imagecoord.push_back(gtsam::Point2(x, y));
        }
        if(fp != NULL)
            fclose(fp);
    }
}

string ParseFeatureTrackFile::GetFeatureTrackFilePath(string base, int no, bool makedir){
    char imgfile[100];
    if(makedir) {
        sprintf(imgfile, "%s", base.c_str());
        MakeDir(imgfile);
        sprintf(imgfile, "%s/sift", base.c_str());
        MakeDir(imgfile);
        sprintf(imgfile, "%s/sift/%04d", base.c_str(), (((int) no)/1000));
        MakeDir(imgfile);
    }
    sprintf(imgfile, "%s/sift/%04d/%04d.csv", base.c_str(), (((int) no)/1000), (((int) no)%1000));
    return string(imgfile);
}

void ParseFeatureTrackFile::SetPFTContents(string base, int ftnum, double timestamp, vector<int>& _ids, vector<cv::Point2f>& _points){
    siftfile = ParseFeatureTrackFile::GetFeatureTrackFilePath(base, ftnum, true);
    time = timestamp;
    for(int i=0; i<_ids.size(); i++){
        ids.push_back(_ids[i]);
        imagecoord.push_back(gtsam::Point2(_points[i].x, _points[i].y));
    }
}

void ParseFeatureTrackFile::WriteFeatureTrackFile() {
    FILE * fp = fopen(siftfile.c_str(), "w");
    if(!fp){
        std::cout << "ParseFeatureTrackFile::WriteFeatureTrackFile couldn't open the file for writing: " <<siftfile << std::endl;
        exit(1);
    }
    fprintf(fp, "%lf\n", time);
    for(int i=0; i<ids.size(); i++){
        fprintf(fp, "%d, %e, %e\n", ids[i], imagecoord[i].x(), imagecoord[i].y());
    }
    fflush(fp);
    fclose(fp);
}

int ParseFeatureTrackFile::GetIndexOfPoint(int point_id){
    for(int i=0; i<ids.size(); i++){
        if(ids[i]==point_id){
            return i;
        }
    }
    return -1;
}

void ParseFeatureTrackFile::Load(){
	time = -1;
	ids.clear();
	imagecoord.clear();

	if(Exists(siftfile)) {
	    ReadDelimitedFile(siftfile, 0);
	}
}

void ParseFeatureTrackFile::Next(int no){
	_no = no;
	siftfile = ParseFeatureTrackFile::GetFeatureTrackFilePath(_base, no);
	Load();
}


double ParseFeatureTrackFile::GetDisplacementFrom(ParseFeatureTrackFile& compared)
{
    /*
     Determine the number of pixels displaced from the nearby image.
      -Computes displacement stats from all the overlapping features.
      -average displacement is less informative in edge cases. I should also consider the number of overlapping features.
     returns -1 if there are no overlapping features.
     */
    
    if(ids[ids.size()-1] > compared.ids[compared.ids.size()-1]){
        cout <<"Input is out of order. This function doesn't handle the backwards case."<<endl;
        exit(-1);
    }
    
    int count = 0;
    double sum = 0;
    int last = 0;
    for(int i=0; i<ids.size(); i++)
    {
        for(int j=last+1; j<compared.ids.size(); j++)
        {
            if(compared.ids[j] < ids[i]) continue;
            else if(compared.ids[j] > ids[i]) break;
            
            gtsam::Point2 ip = imagecoord[i];
            gtsam::Point2 jp = compared.imagecoord[j];
            double displacement = pow(pow(ip.x() - jp.x(), 2) + pow(ip.y() - jp.y(), 2), 0.5);
            sum += displacement;
            count++;
            last = j;
        }
    }
    
    if(count==0) return -1;
    return sum/count;
}


gtsam::Point2 ParseFeatureTrackFile::GetAverageOpticalFlowFrom(ParseFeatureTrackFile& compared)
{
    /*
     Determine the number of pixels displaced from the nearby image.
     -Computes displacement stats from all the overlapping features.
     -average displacement is less informative in edge cases. I should also consider the number of overlapping features.
     returns -1 if there are no overlapping features.
     */
    
    if(ids[ids.size()-1] > compared.ids[compared.ids.size()-1]){
        cout <<"Input is out of order. This function doesn't handle the backwards case."<<endl;
        exit(-1);
    }
    
    int count = 0;
    double sumx = 0;
    double sumy = 0;
    int last = 0;
    for(int i=0; i<ids.size(); i++)
    {
        for(int j=last+1; j<compared.ids.size(); j++)
        {
            if(compared.ids[j] < ids[i]) continue;
            else if(compared.ids[j] > ids[i]) break;
            
            gtsam::Point2 ip = imagecoord[i];
            gtsam::Point2 jp = compared.imagecoord[j];
            sumx += ip.x() - jp.x();
            sumy += ip.y() - jp.y();
//            double displacement = pow(pow(ip.x() - jp.x(), 2) + pow(ip.y() - jp.y(), 2), 0.5);
//            sum += displacement;
            count++;
            last = j;
        }
    }
    
    if(count==0) return gtsam::Point2(-1,-1);
    return gtsam::Point2(sumx/count, sumy/count);
}


void ParseFeatureTrackFile::Reset(){
    ids.clear();
    imagecoord.clear();
    siftfile = "";
    time = -1;
}

ParseFeatureTrackFile ParseFeatureTrackFile::LoadFTF(Camera& _cam, string base, int ftfileno) {
    ParseFeatureTrackFile pftf = ParseFeatureTrackFile(_cam, base, ftfileno);
    if(pftf.time == -1) {
        cout << "ParseFeatureTrackFile::LoadFTF() Error. Couldn't open: " << pftf.siftfile << endl;
        exit(-1);
    }
    return pftf;
}

void ParseFeatureTrackFile::ModifyFTFData(vector<gtsam::Point3>& p3d){
    //Finds and keeps only the landmarks that were good in the stand-alone optimization.
    //vector<gtsam::Point3> p3d = POR.GetSubsetOf3DPoints(pftf.ids);
    vector<int> subset_ids;
    vector<gtsam::Point2> subset_p2d;
    for(int i=0; i<p3d.size(); i++){
        if(p3d[i].x()==0 && p3d[i].y()==0 && p3d[i].z()==0) continue;
        subset_ids.push_back(ids[i]);
        subset_p2d.push_back(imagecoord[i]);
    }
    ids = subset_ids;
    imagecoord = subset_p2d;
}

vector<LandmarkTrack> ParseFeatureTrackFile::ProcessNewPoints(int survey, int ckey, vector<LandmarkTrack>& active, double percent_of_tracks) {
    vector<LandmarkTrack> inactive;
    static int last_skipped = 0;
    int next_entry = 0;
    int num_landmarks_skipped=0;
    int lasti=0;
    for(int i=0; i<ids.size(); i++) {
        lasti=i;
        //remove features that aren't tracked anymore
        //add to the entry using the info from the new frame.
        while(active.size() > next_entry && active[next_entry].GetKey() < ids[i]) {
            if(debug)cout << "Removed landmark " << active[next_entry].key << endl;
            if(active[next_entry].Length()>1) inactive.push_back(active[next_entry]);
            active.erase(active.begin() + next_entry, active.begin() + next_entry + 1);
        }
        
        //create a new entry if its key is greater than anything that's active
        if(active.size() == next_entry) {
            break;
        } else if(ids[i] < active[next_entry].key) {
            if(last_skipped < ids[i]){
                last_skipped = ids[i];
                num_landmarks_skipped++;
                cout << "ParseFeatureTrackFile::ProcessNewPoints() Skipped landmark " << ids[i] << " of file " << siftfile << endl;
            }
            continue;
        } else if(ids[i] == active[next_entry].key) {
            //accumulate info about the landmark (should be the only remaining case)
            active[next_entry].AddToTrack(imagecoord[i], survey, ckey);
            if(debug) cout << "landmark measurement for " << active[next_entry].key << endl;
            //inc next_entry.
            next_entry++;
            lasti=i+1;
        }
    }
    
    //add the rest
    srand(std::time(0));
    for(int i=lasti; i<ids.size(); i++) {
        //used to limit the size of the optimization problem for inter-survey optimization
        bool used = (rand()%100 < percent_of_tracks);
        LandmarkTrack lt(ids[i], used);
        lt.AddToTrack(imagecoord[i], survey, ckey);
        active.push_back(lt);
    }
    return inactive;
}

bool ParseFeatureTrackFile::CheckImageDuplication(vector<LandmarkTrack>& active){
    if(ids.size() != active.size()) return false;
    int next_entry = 0;
    for(int i=0; i<ids.size(); i++){
        if(ids[i] != active[next_entry].key) return false;
        if(imagecoord[i].distance(active[next_entry].points[active[next_entry].Length()-1]) > 0.0000001) return false;
        next_entry++;
    }
    return true;
}

void swap(ParseFeatureTrackFile& first, ParseFeatureTrackFile& second){
    //see http://stackoverflow.com/questions/5695548/public-friend-swap-member-function
    //for an explanation of the syntax, and use of using, without specifying std::swap below.
    //Also see https://stackoverflow.com/questions/4117002/why-can-i-access-private-variables-in-the-copy-constructor
    // which says that two objects of the same class can access each other's private data.
    using std::swap;
    
    swap(first._no, second._no);
    swap(first._base, second._base);
    swap(first.siftfile, second.siftfile);
    swap(first.ids, second.ids);
    swap(first.imagecoord, second.imagecoord);
    swap(first.time, second.time);
}

ParseFeatureTrackFile& ParseFeatureTrackFile::operator=(ParseFeatureTrackFile other){
    //see http://stackoverflow.com/questions/3279543/what-is-the-copy-and-swap-idiom
    //for an explanation of why this is the right way to define an assignment operator.
    //This assignment isn't passed by reference, because we don't want to
    //copy the function arguments. See
    //https://web.archive.org/web/20140113221447/http://cpp-next.com/archive/2009/08/want-speed-pass-by-value/
    swap(*this, other);
    return *this;
}

int BinarySearchLandmarkRange(std::vector<LandmarkTrack>& landmarks, int ckey, bool end){
    //binary search with repeats.
    if(ckey<0) return 0;
    if(ckey>=landmarks.size()) return landmarks.size()-1;
    int top = landmarks.size();
    int bot = 0;
    int med;
    int ckeycomp;
    double comp = (end)? ckey+0.1:ckey-0.1;
//    std::cout << "start binary search " << std::endl;
    while(top-bot>1) {
        med = bot + (top - bot)/2;
        ckeycomp = landmarks[med].camera_keys[0].index();
        if(end) ckeycomp = landmarks[med].camera_keys[landmarks[med].Length()-1].index();
//        std::cout << "range (" << bot << ", " << top << ") " << med << ", " <<ckeycomp << " compared to " << comp << std::endl;
        if(ckeycomp < comp) bot = med;
        else if(ckeycomp > comp) top = med;
        else break;
    }
//    std::cout << "finished binary search " << std::endl;
    return med;
}

std::vector<int> ParseFeatureTrackFile::ApproximateLandmarkSet(std::vector<LandmarkTrack>& landmarks, int ckey){
    //MAX_NOT_FOUND depends on a landmark's distance to the camera, as does the other one, which is why it's approximate.
    int MAX_NOT_FOUND = 10;
    
    int refkey = ckey;
    int end = BinarySearchLandmarkRange(landmarks, ckey+1, false);
    vector<int> indices;
    int not_found = 0;
    while(1){
        int s = landmarks[end].camera_keys[0].index();
        int e = s + landmarks[end].camera_keys.size();
        if(s <= ckey && e > refkey) {
            indices.push_back(end);
            end--;
            not_found = 0;
        } else if(not_found > MAX_NOT_FOUND || ckey <= 0) {
            break;
        } else {
            //while(landmarks[end].camera_keys[0].index() < ckey)
            ckey = s-1;
            end = BinarySearchLandmarkRange(landmarks, ckey--, false);
            not_found++;
        }
    }
    
    return indices;
}

ParseFeatureTrackFile ParseFeatureTrackFile::ReconstructFromCachedSet(Camera& cam, std::vector<LandmarkTrack>& landmarks, int ckey){
    vector<int> approxset = ParseFeatureTrackFile::ApproximateLandmarkSet(landmarks, ckey);

    if(approxset.size() > 300){
        std::cout << "approx set size: " << approxset.size()<< " for key " << ckey << std::endl;
        for(int i=0; i<approxset.size(); i++){
            int offset = ckey - landmarks[approxset[i]].camera_keys[0].index();
            std::cout << "camera key: " << landmarks[approxset[i]].camera_keys[offset].index() << std::endl;
        }
        exit(1);
    }
    
    ParseFeatureTrackFile pftf(cam);
    for(int i=approxset.size()-1; i>=0; i--) {
        pftf.ids.push_back(landmarks[approxset[i]].key);
        int offset = ckey - landmarks[approxset[i]].camera_keys[0].index();
        pftf.imagecoord.push_back(landmarks[approxset[i]].points[offset]);
    }
    return pftf;
}
