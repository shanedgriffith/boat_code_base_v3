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


//This version is less buggy, in some way due to lack of reliance on string, which uses dynamic memory.
void ParseFeatureTrackFile::ReadDelimitedFile(string file, int type) {
    char line[LINESIZE]="";
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




