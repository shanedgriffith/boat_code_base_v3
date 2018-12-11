

#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseSurvey.h>
#include <Visualizations/FlickeringDisplay.h>
#include <RFlowOptimization/LocalizedPoseData.hpp>
#include <RFlowOptimization/LPDInterface.hpp>

#include "AlignICPImagePairs.hpp"


cv::Vec3b AlignICPImagePairs::FindGrayBorderColor(const cv::Mat& a)
{
    cv::Vec3b tl = a.at<cv::Vec3b>(0,0);
    cv::Vec3b bl = a.at<cv::Vec3b>(a.rows-1,0);
    if(tl==bl) return tl;
    cv::Vec3b tr = a.at<cv::Vec3b>(0,a.cols-1);
    cv::Vec3b br = a.at<cv::Vec3b>(a.rows-1,a.cols-1);
    if(tr==br) return tr;
    return cv::Vec3b(0,0,0);
}

void AlignICPImagePairs::FixImagesForComparison(cv::Mat& a, cv::Mat& b)
{
    cv::Vec3b black(0,0,0);
    cv::Vec3b graya = FindGrayBorderColor(a);
    cv::Vec3b grayb = FindGrayBorderColor(b);
    
    for(int i=0; i<a.rows; ++i)
    {
        for(int j=0; j<a.cols; j++)
        {
            cv::Vec3b cola = a.at<cv::Vec3b>(i,j);
            cv::Vec3b colb = b.at<cv::Vec3b>(i,j);
            if(cola==black or colb==black)
            {
                a.at<cv::Vec3b>(i,j) = black;
                b.at<cv::Vec3b>(i,j) = black;
            }
            if(cola==graya or colb==grayb)
            {
                a.at<cv::Vec3b>(i,j) = black;
                b.at<cv::Vec3b>(i,j) = black;
            }
        }
    }
}

int AlignICPImagePairs::DateToIdx(int date){
    std::string d = std::to_string(date);
    for(int i=0; i<_dates.size(); i++){
        if(d == _dates[i]) return i;
    }
    return -1;
}

std::vector<std::string> AlignICPImagePairs::ParseLineAdv(char * line, std::string separator) {
    std::vector<std::string> parsedline;
    int idx = 0;
    const char* tok;
    for (tok = strtok(line, separator.c_str());
         tok && *tok;
         tok = strtok(NULL, std::string(separator+"\n").c_str()))
    {
        parsedline.push_back(tok);
        idx++;
    }
    return parsedline;
}

std::string AlignICPImagePairs::PaddedInt(int num){
    char n[7];
    sprintf(n, "%06d", num);
    std::string res(n);
    return res;
}

void AlignICPImagePairs::AlignTimelapsesRFlow(std::string dirnum) {
    std::vector<ParseOptimizationResults> por;
    std::vector<Map> maps;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults(_maps_dir, _dates[i]));
        maps.push_back(Map(_maps_dir));
        maps[i].LoadMap(_dates[i]);
    }
    
    std::string base = "/home/shaneg/data/timelapses/" + dirnum + "/";
    std::string csvfile = base + "image_all_pairs.csv";
    
    FILE * fp = fopen(csvfile.c_str(), "r");
    if(!fp){
        std::cout << "couldn't open " << csvfile << std::endl;
        exit(1);
    }
    
    char line[1000];
    while(fgets(line, 999, fp)){
        char * tmp = line;
        std::vector<std::string> lp = ParseLineAdv(tmp, ",");
        if(lp[0]=="#")continue;
        int d1 = DateToIdx(stoi(lp[1]));
        if(d1 < 0) continue;
        std::cout << "Aligning images for " << lp[1] << "_" << lp[2] << std::endl;
        
        int refnum = stoi(lp[2]);
        std::string savebase = base + lp[1] + "_" + PaddedInt(refnum) + "/rf/";
        FileParsing::MakeDir(savebase);
        int pose1 = por[d1].GetNearestPoseToImage(refnum);
        
        for(int i=3; i<lp.size(); i++){ // 39
            int d2 = i-3;
            if(d2 == d1) continue;
            
            int alignnum = stoi(lp[i]);
            if(alignnum==-1) continue;
            
            int tidx = man.GetOpenMachine();
            int pose2 = por[d2].GetNearestPoseToImage(alignnum);
            
            std::string savename = savebase + _dates[d2] + "_" + PaddedInt(alignnum) + ".jpg";
            ws[tidx]->Setup(pose1, savename, pose2);
            ws[tidx]->SetMaps({&maps[d1], &maps[d2]});
            ws[tidx]->SetDates({_dates[d1], _dates[d2]});
            ws[tidx]->SetPOR({&por[d1], &por[d2]});
            man.RunMachine(tidx);
        }
        man.WaitForMachine(true);
    }
    fclose(fp);
    
    std::cout << "Finished aligning images  " << std::endl;
}

void AlignICPImagePairs::AlignTimelapsesSFlow(std::string dirnum){
    std::string base = "/home/shaneg/data/timelapses/" + dirnum + "/";
    std::string csvfile = base + "image_all_pairs.csv";
    
    FILE * fp = fopen(csvfile.c_str(), "r");
    if(!fp){
        std::cout << "couldn't open " << csvfile << std::endl;
        exit(1);
    }
    
    char line[1000];
    while(fgets(line, 999, fp)){
        char * tmp = line;
        std::vector<std::string> lp = ParseLineAdv(tmp, ",");
        if(lp[0]=="#")continue;
        int d1 = DateToIdx(stoi(lp[1]));
        if(d1 < 0) continue;
        std::cout << "Aligning images for " << lp[1] << "_" << lp[2] << std::endl;
        
        int refnum = stoi(lp[2]);
        std::string savebase = base + lp[1] + "_" + PaddedInt(refnum) + "/warpsf/";
        FileParsing::MakeDir(savebase);
        
        for(int i=3; i<39; i++){
            int d2 = i-3;
            if(d2 == d1) continue;
            
            int alignnum = stoi(lp[i]);
            if(alignnum==-1) continue;
            
            int tidx = man.GetOpenMachine();

            std::string _image0 = base + lp[1] + "_" + PaddedInt(refnum) + "/reference.jpg";
            std::string _image1 = base + lp[1] + "_" + PaddedInt(refnum) + "/warp/warp_" + _dates[d2] + "_" + PaddedInt(alignnum) + ".jpg";
            std::string savename = savebase + "warpsf_" + _dates[d2] + "_" + PaddedInt(alignnum) + ".jpg";
            
            ws[tidx]->SetImages(_image0, _image1, savename);
            man.RunMachine(tidx);
        }
        man.WaitForMachine(true);
    }
    fclose(fp);
    
    std::cout << "Finished aligning images  " << std::endl;
}

void AlignICPImagePairs::AlignImagesRFlow(std::string file, int firstidx, int lastidx) {
    std::vector<std::vector<int> > imgparams = ReadCSVFile(file, firstidx, lastidx);
    std::vector<ParseOptimizationResults> por;
    std::vector<Map> maps;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults(_maps_dir, _dates[i]));
        maps.push_back(Map(_maps_dir));
        maps[i].LoadMap(_dates[i]);
    }
    
    for(int i=0; i<imgparams.size(); i++){
        std::cout << "aligning: "<<imgparams[i][0] << ", " <<imgparams[i][1] <<"." << imgparams[i][2] << " to " << imgparams[i][3] <<"." << imgparams[i][4] << std::endl;
        int tidx = man.GetOpenMachine();
        char filename[100];
        sprintf(filename, "%s%06d_w.jpg", _savebase.c_str(), imgparams[i][0]);
        std::string savename(filename);
        
        int d1 = DateToIdx(imgparams[i][1]);
        int d2 = DateToIdx(imgparams[i][3]);
        int pose1 = por[d1].GetNearestPoseToImage(imgparams[i][2]);
        int pose2 = por[d2].GetNearestPoseToImage(imgparams[i][4]);
        
        ws[tidx]->Setup(pose1, savename, pose2);
        ws[tidx]->SetMaps({&maps[d1], &maps[d2]});
        ws[tidx]->SetDates({_dates[d1], _dates[d2]});
        ws[tidx]->SetPOR({&por[d1], &por[d2]});
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    
    std::cout << "Finished aligning images  " << std::endl;
}

void AlignICPImagePairs::AlignImagesWarped() {
    for(int i=0; i<1000; i++){
        char num[10];
        sprintf(num, "%06d", i);
        num[6] = '\0';
        std::string n(num);
        int tidx = man.GetOpenMachine();
        std::string _image0 = "/home/shaneg/data/icp_pairs_0000/warp/" + n + "_warp_1.jpg";
        std::string _image1 = "/home/shaneg/data/icp_pairs_0000/warp/" + n + "_warp_2.jpg";
        std::string savename = "/home/shaneg/results/aligned_icp/" + n + "_warp_w.jpg";

        ws[tidx]->SetImages(_image0, _image1, savename);
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
}

void AlignICPImagePairs::AlignImagesSFlow(std::string file, int firstidx, int lastidx){
    std::vector<std::vector<int> > imgparams = ReadCSVFile(file, firstidx, lastidx);
    
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults(_maps_dir, _dates[i]));
    }
    
    for(int i=0; i<imgparams.size(); i++){
        std::cout << "aligning: "<<imgparams[i][0] << ", " <<imgparams[i][1] <<"." << imgparams[i][2] << " to " << imgparams[i][3] <<"." << imgparams[i][4] << std::endl;
        int tidx = man.GetOpenMachine();
        char filename[100];
        sprintf(filename, "%s%06d_w.jpg", _savebase.c_str(), imgparams[i][0]);
        std::string savename(filename);
        
        int d1 = DateToIdx(imgparams[i][1]);
        int pose1 = por[d1].GetNearestPoseToImage(imgparams[i][2]);
        int d2 = DateToIdx(imgparams[i][3]);
        int pose2 = por[d2].GetNearestPoseToImage(imgparams[i][4]);

        std::string _image0 = ParseSurvey::GetImagePath(_query_loc + std::to_string(imgparams[i][1]), por[d1].cimage[pose1]);
        std::string _image1 = ParseSurvey::GetImagePath(_query_loc + std::to_string(imgparams[i][3]), por[d2].cimage[pose2]);
        
        ws[tidx]->SetImages(_image0, _image1, savename);
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    
    std::cout << "Finished aligning images  " << std::endl;
}

std::vector<std::vector<int> > AlignICPImagePairs::ReadCSVFileLabels(std::string file, int firstidx, int lastidx){
    FILE * fp = FileParsing::OpenFile(file, "r");
    char line[1000];
    
    std::vector<std::vector<int> > imglabels;
    while (fgets(line, 1000-1, fp)) {
        char * tmp = line;
        std::vector<std::string> lp = FileParsing::ParseLine(tmp);
        std::vector<int> res;
        for(int i=0; i<lp.size(); ++i)
        {
            res.push_back(stoi(lp[i]));
        }
        imglabels.push_back(res);
    }
    fclose(fp);
    return imglabels;
}

std::vector<std::vector<int> > AlignICPImagePairs::ReadCSVFile(std::string file, int firstidx, int lastidx){
    FILE * fp = FileParsing::OpenFile(file, "r");
    
    std::vector<std::vector<int> > imgparams;
    
    std::cout << "Aligning images between " << firstidx << " and " << lastidx << std::endl;
    while(!feof(fp)){
        int idx, date0, img0num, date1, img1num;
        double a, b, c, x, y, z;
        int ret = fscanf(fp, "%d,%d,%d,%lf,%lf,%lf,%d,%d,%lf,%lf,%lf",
               &idx, &date0, &img0num, &a, &b, &c, &date1, &img1num, &x, &y, &z);
        
        if(ret != 11){
            std::cout << "error reading file: " << file << ". Line has " << ret << " of 11 values. Read " << imgparams.size() << " entries successfully." << std::endl;
            break;
        }
        
        if(idx < firstidx) continue;
        else if(idx > lastidx) break;
        
        std::vector<int> toalign = {idx, date0, img0num, date1, img1num};
        imgparams.push_back(toalign);
    }
    fclose(fp);
    return imgparams;
}

void AlignICPImagePairs::GetResults() {
    std::string rfbase = "/Users/shane/Documents/research/experiments/rf_pairs/";
    std::string sfbase = "/Users/shane/Documents/research/experiments/aligned_icp/";
    std::string wbase = "/Users/shane/Documents/research/experiments/pairs_unaligned/warp/";
//    std::string sfbase = "/Users/shane/Documents/research/experiments/sf_pairs/";
    std::string file = "/Users/shane/Documents/research/experiments/image_pairs.csv";
    
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults("/Volumes/Untitled/data/SingleSessionSLAM/", _dates[i]));
    }
    
    std::vector<std::vector<int> > from2014 = ReadCSVFile(file, 0, 999);
    std::vector<int> counter(3,0);
    
    FlickeringDisplay fd;
    for(int i=0; i<from2014.size(); i++){
        std::cout << "aligning: "<<from2014[i][0] << ", " <<from2014[i][1] <<"." << from2014[i][2] << " to " << from2014[i][3] <<"." << from2014[i][4] << std::endl;
        int d1 = DateToIdx(from2014[i][1]);
        int pose1 = por[d1].GetNearestPoseToImage(from2014[i][2]);
        std::cout << "1:closest image to " <<from2014[i][2] << " is " << pose1 << " with " << por[d1].cimage[pose1] << std::endl;
        int d2 = DateToIdx(from2014[i][3]);
        int pose2 = por[d2].GetNearestPoseToImage(from2014[i][4]);
        std::cout << "2:closest image to " <<from2014[i][4] << " is " << pose2 << " with " << por[d2].cimage[pose2] << std::endl;
        
        if(from2014[i][1] == 141010 || from2014[i][3] == 141010) continue;
        
        char strnum[10];
        sprintf(strnum, "%06d", from2014[i][0]);
        
        std::string imagename(strnum);
        
//        std::string _image0_sf = ParseSurvey::GetImagePath(_query_loc + std::to_string(from2014[i][1]), from2014[i][2]);
        std::string _image0_rf = ParseSurvey::GetImagePath(_query_loc + std::to_string(from2014[i][1]), por[d1].cimage[pose1]);
        std::string _imagerf = rfbase + imagename + "_w.jpg";
        std::string _image0w_sf = wbase + imagename + "_warp_1.jpg";
        std::string _imagewsf = sfbase + imagename + "_warp_w.jpg";
        
//        cv::Mat refsf = ImageOperations::Load(_image0_sf);
        cv::Mat refsf = ImageOperations::Load(_image0w_sf);
        cv::Mat refrf = ImageOperations::Load(_image0_rf);
        cv::Mat Imrf = ImageOperations::Load(_imagerf);
        cv::Mat Imsf = ImageOperations::Load(_imagewsf);
//        cv::Mat Imsf = ImageOperations::Load(_imagesf);
        
        cv::Mat refx2 = FlickeringDisplay::CombinedImage(refsf, refrf);
        cv::Mat rfsf = FlickeringDisplay::CombinedImage(Imsf, Imrf);
        
        char c = fd.FlickerImages(refx2, rfsf);
        switch(c){
            case 'c':{
                counter[0]++;
                break;}
            case 'g':{
                counter[1]++;
                break;}
            case 'b':{
                counter[2]++;
                break;}
        }
        std::cout << ""<<c << std::endl;
        std::cout << std::endl;
    }
    
    std::cout << "c,g,p: " << counter[0] << ", " << counter[1] << ", " << counter[2] << std::endl;
}

void AlignICPImagePairs::GetResultsTimelapse(std::string argnum, std::string argdate){
    std::string rootdir = "/Users/shane/Documents/research/results/random timelapses/000" + argnum + "/" +argdate +"/";
    std::string rfbase = rootdir + "rf/";
    std::string sfbase = rootdir + "warpsf/";
    std::string sfref = rootdir + "reference.jpg";
    
    std::string date = argdate.substr(0, 6);
    int ref_num = stoi(argdate.substr(7, 13));
    
    ParseOptimizationResults POR(_maps_dir, date);
    int pose1 = POR.GetNearestPoseToImage(ref_num);
    int image_num = POR.cimage[pose1];
    std::string reference = ParseSurvey::GetImagePath(_query_loc + date, image_num);
    
    std::vector<std::string> rffiles = FileParsing::ListFilesInDir(rfbase, "jpg");
    std::vector<std::string> warpsffiles = FileParsing::ListFilesInDir(sfbase, "jpg");
    
    cv::Mat Imsfref = ImageOperations::Load(sfref);
    cv::Mat ref = ImageOperations::Load(reference);
    std::vector<int> counter(3,0);
    
    FlickeringDisplay fd;
    for(int i=0; i<rffiles.size(); i++) {
        int idxW = -1;
        for(int j=0; j<warpsffiles.size(); j++){
            if(warpsffiles[j].find(rffiles[i])!=std::string::npos) {idxW = j; break;}
        }
        if(idxW==-1) continue;
        std::cout << "files: " << rfbase + rffiles[i] << "\n       " << sfbase + warpsffiles[idxW] << std::endl;
        
        cv::Mat Imrf = ImageOperations::Load(rfbase + rffiles[i]);
        cv::Mat Imsf = ImageOperations::Load(sfbase + warpsffiles[idxW]);
        
        cv::Mat refx2 = FlickeringDisplay::CombinedImage(Imsfref, ref);
        cv::Mat rfsf = FlickeringDisplay::CombinedImage(Imsf, Imrf);
        
        char c = fd.FlickerImages(refx2, rfsf);
        switch(c){
            case 'c':{
                counter[0]++;
                break;}
            case 'g':{
                counter[1]++;
                break;}
            case 'b':{
                counter[2]++;
                break;}
        }
        std::cout << ""<<c << std::endl;
        std::cout << std::endl;
    }
    
    std::cout << "c,g,p: " << counter[0] << ", " << counter[1] << ", " << counter[2] << std::endl;
}

void AlignICPImagePairs::LabelTimelapse(){
    std::string rootdir = "/Users/shanehome/Documents/140502_571/";
    std::string rfbase = rootdir + "rf/";
    std::string sfbase = rootdir + "mappoints/";
    std::string sfref = rootdir + "reference.jpg";
    
    std::vector<std::string> rffiles = FileParsing::ListFilesInDir(rfbase, "jpg");
    std::vector<std::string> warpsffiles = FileParsing::ListFilesInDir(sfbase, "jpg");
    
    cv::Mat ref = ImageOperations::Load(sfref);
    std::vector<int> counter(3,0);
    
    FlickeringDisplay fd;
    for(int i=0; i<rffiles.size(); i++) {
        cv::Mat Imrf = ImageOperations::Load(rfbase + rffiles[i]);
        
        cv::Mat refx2 = FlickeringDisplay::CombinedImage(ref, ref);
        cv::Mat rfsf = FlickeringDisplay::CombinedImage(Imrf, Imrf);
        
        char c = fd.FlickerImages(refx2, rfsf);
        switch(c){
            case 'c':{
                counter[0]++;
                break;}
            case 'g':{
                counter[1]++;
                break;}
            case 'b':{
                counter[2]++;
                break;}
            case 'q':{
                exit(1);
                break;}
        }
        std::cout << rffiles[i] << ", "<<c << std::endl;
    }
    
    std::cout << "c,g,p: " << counter[0] << ", " << counter[1] << ", " << counter[2] << std::endl;
}

void AlignICPImagePairs::GetResultsLabels(bool sflow) {
    std::string rfbase = "/Volumes/Untitled/data/aligned_rf_2014/";
    if(sflow) rfbase = "/Volumes/Untitled/data/aligned_sf_2014/";
    std::string file = "/Volumes/Untitled/data/aligned_icp_2014_orig/image_pairs.csv";
    
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults("/Volumes/SAMSUNG/Data/origin/", _dates[i]));
    }
    
    std::vector<std::vector<int> > from20star = ReadCSVFile(file, 0, 999);
    std::vector<int> counter(3,0);
    
    FlickeringDisplay fd;
    for(int i=0; i<from20star.size(); i++){
        int d1 = DateToIdx(from20star[i][1]);
        int pose1 = por[d1].GetNearestPoseToImage(from20star[i][2]);
        std::string _image0_rf = ParseSurvey::GetImagePath(_query_loc + std::to_string(from20star[i][1]), por[d1].cimage[pose1]);
        
        std::string imagename = PaddedInt(i);
        std::string _imagerf = rfbase + imagename + "_w.jpg";
        
        cv::Mat refrf = ImageOperations::Load(_image0_rf);
        cv::Mat Imrf = ImageOperations::Load(_imagerf);
        
        FixImagesForComparison(refrf, Imrf);
        char c = fd.FlickerImages(refrf, Imrf);
        switch(c){
            case 'c':{
                counter[0]++;
                break;}
            case 'g':{
                counter[1]++;
                break;}
            case 'b':{
                counter[2]++;
                break;}
            case 'q':{
                exit(1);
                break;}
        }
        std::cout << from20star[i][1] << "."<<from20star[i][2] << ", " << from20star[i][3] << "." << from20star[i][4]<< "," <<c << std::endl;
    }
    
    std::cout << "c,g,p: " << counter[0] << ", " << counter[1] << ", " << counter[2] << std::endl;
}

void AlignICPImagePairs::GetResultsLabelsICP() {
    std::string icpbase = "/Volumes/Untitled/data/aligned_icp_2014/";
    std::string icporig = "/Volumes/Untitled/data/aligned_icp_2014_orig/";
    std::string file = icporig + "image_pairs.csv";
    
    std::vector<std::vector<int> > from20star = ReadCSVFile(file, 0, 999);
    if(from20star.size() ==0){
        std::cout << "no entries." << std::endl;
        exit(-1);
    }
    
    std::vector<int> counter(3,0);
    
    FlickeringDisplay fd;
    for(int i=0; i<1000; i++){
        if(not(from20star[i][2]==29285))
            continue;
        std::string imagename = PaddedInt(i);
        std::string refimg = icporig + "warp/" + imagename + "_warp_1.jpg";
        std::string aligned = icpbase + imagename + "_warp_w.jpg";
        
        cv::Mat refrf = ImageOperations::Load(refimg);
        cv::Mat Imrf = ImageOperations::Load(aligned);
        
        FixImagesForComparison(refrf, Imrf);
        char c = fd.FlickerImages(refrf, Imrf);
        switch(c){
            case 'c':{
                counter[0]++;
                break;}
            case 'g':{
                counter[1]++;
                break;}
            case 'b':{
                counter[2]++;
                break;}
            case 'q':{
                exit(1);
                break;}
        }
        std::cout << from20star[i][1] << "."<<from20star[i][2] << ", " << from20star[i][3] << "." << from20star[i][4]<< ", " << c << std::endl;
    }
    
    std::cout << "c,g,p: " << counter[0] << ", " << counter[1] << ", " << counter[2] << std::endl;
}

void AlignICPImagePairs::CompareRFWithICP() {
    std::string icpbase = "/Volumes/Untitled/data/aligned_icp_2014/";
    std::string icporig = "/Volumes/Untitled/data/aligned_icp_2014_orig/";
    std::string file = icporig + "image_pairs.csv";
    std::string rfbase = "/Volumes/Untitled/data/aligned_rf_2014/";
    
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults("/Volumes/SAMSUNG/Data/origin/", _dates[i]));
    }
    
    std::vector<std::vector<int> > from20star = ReadCSVFile(file, 0, 999);
    std::vector<int> counter(3,0);
    
    std::vector<int> compa = {24384, 55196, 58867, 33970, 41306, 27539, 20416, 7048};
    std::vector<int> compb = {30954, 45413, 54714, 21015, 41946, 16250, 28226, 2797};
    
    int c=0;
    FlickeringDisplay fd;
    for(int i=0; i<1000; i++){
        int d1 = DateToIdx(from20star[i][1]);
        int pose1 = por[d1].GetNearestPoseToImage(from20star[i][2]);
        std::cout << "1:closest image to " <<from20star[i][2] << " is " << pose1 << " with " << por[d1].cimage[pose1] << std::endl;
        
        if(not (from20star[i][2] == compa[c] and from20star[i][4] == compb[c])){
            continue;
        }
        c++;
        
        std::string imagename = PaddedInt(i);
        std::string _image0_rf = ParseSurvey::GetImagePath(_query_loc + std::to_string(from20star[i][1]), por[d1].cimage[pose1]);
        std::string _imagerf = rfbase + imagename + "_w.jpg";
        
        cv::Mat refrf = ImageOperations::Load(_image0_rf);
        cv::Mat Imrf = ImageOperations::Load(_imagerf);
        
        std::string refimg = icporig + "warp/" + imagename + "_warp_1.jpg";
        std::string aligned = icpbase + imagename + "_warp_w.jpg";
        
        cv::Mat reficp = ImageOperations::Load(refimg);
        cv::Mat Imicp = ImageOperations::Load(aligned);
        
        FixImagesForComparison(reficp, Imicp);
        FixImagesForComparison(refrf, Imrf);
        cv::Mat refx2 = FlickeringDisplay::CombinedImage(reficp, refrf);
        cv::Mat rfaligned = FlickeringDisplay::CombinedImage(Imicp, Imrf);
        
        char c = fd.FlickerImages(refx2, rfaligned);
        switch(c){
            case 'c':{
                counter[0]++;
                break;}
            case 'g':{
                counter[1]++;
                break;}
            case 'b':{
                counter[2]++;
                break;}
            case 'q':{
                exit(1);
                break;}
        }
        std::cout << from20star[i][1] << "."<<from20star[i][2] << ", " << from20star[i][3] << "." << from20star[i][4]<< ", " << c << std::endl;
    }
    
    std::cout << "c,g,p: " << counter[0] << ", " << counter[1] << ", " << counter[2] << std::endl;
}

void AlignICPImagePairs::PareComparisonFile()
{
    FILE * fp = fopen("/Users/shanehome/Documents/Results/comparison.rtf", "r");
    FILE * fw = fopen("/Users/shanehome/Documents/Results/comparison_pared.txt", "w");
    if(!fp or !fw) {
        std::cout << "error reading file" << std::endl;
        exit(-1);
    }
    
    char line[1000];
    while(!feof(fp)){
        fgets(line, 1000, fp);
        if(line[0] == '1' and line[1] == ':')
            continue;
        
        fputs(line, fw);
    }
    fclose(fp);
    fclose(fw);
}

void AlignICPImagePairs::AnalyzeAlignmentQualityTrend()
{
    FILE * fp = fopen("/Users/shanehome/Documents/Results/Data/rf_pared.txt", "r");
    if(!fp) {
        std::cout << "error reading file" << std::endl;
        exit(-1);
    }
    
    while(!feof(fp)){
        int s0, s1, d0, d1;
        char label;
        fscanf(fp, "%d, %d, %d, %d, %c", &d0, &s0, &d1, &s1, &label);
        
        
    }
    fclose(fp);
    
}

void AlignICPImagePairs::CountPosesWithALocalization()
{
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults(_maps_dir, _dates[i]));
    }
    
    for(int i=0; i<_dates.size(); i++)
    {
        LPDInterface lint;
        int nloaded = lint.LoadLocalizations(_maps_dir + _dates[i], _dates);
        
        int count = 0;
        for(int j=0; j<por[i].boat.size(); j++)
        {
            int lpdcur = lint.GetLPDIdx(j);
            if(lpdcur >= 0)
                count++;
        }
        
        std::cout << _dates[i] << " , " << 1.0*count / por[i].boat.size() << std::endl;
    }
}

void AlignICPImagePairs::PercentLocalizedPoses(){
    //    std::string maps = "/home/shaneg/results/maps_/";
    std::string maps = "/Users/shanehome/Documents/";
    
    for(int i=0; i<_dates.size(); i++)
    {
        std::string dir = maps + _dates[i] + "/localizations/";
        if(not FileParsing::DirectoryExists(dir)) continue;
        ParseOptimizationResults por(_maps_dir, _dates[i]);
        std::vector<std::string> files = FileParsing::ListFilesInDir(dir, ".loc");
        std::cout << files.size() << std::endl;
        std::set<int> localized_poses;
        for(int j=0; j<files.size(); j++)
        {
            int idx = static_cast<int>(files[j].find("_"));
            localized_poses.insert(stoi(files[j].substr(0,idx)));
        }
        
        std::cout << _dates[i] << ", " << 1.0*localized_poses.size()/por.boat.size() << std::endl;
    }
}

















