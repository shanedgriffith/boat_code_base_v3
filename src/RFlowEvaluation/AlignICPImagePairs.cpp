

#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseSurvey.h>
#include <Visualizations/FlickeringDisplay.h>
#include <RFlowOptimization/LocalizedPoseData.hpp>
#include <RFlowOptimization/LPDInterface.hpp>
#include <FileParsing/ParseVisibilityFile.h>
#include <Visualizations/SLAMDraw.h>
#include <RFlowEvaluation/ForBMVCFigure.hpp>
#include <ImageAlignment/GeometricFlow/MultiSurveyViewpointSelection.hpp>
#include <RFlowOptimization/LocalizePose.hpp>
#include <Optimization/SingleSession/EvaluateSLAM.h>
#include <Optimization/SingleSession/GTSAMInterface.h>

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

std::vector<std::vector<int> > AlignICPImagePairs::ReadCSVFileLabels(std::string file) {
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

void AlignICPImagePairs::AnalyzeTimeLapses() {
    std::string basedir = "/Users/shane/Documents/research/results/2018_winter/example_alignments/set_of_100/";
    std::vector<std::string> aligndirs = FileParsing::ListDirsInDir(basedir);
    
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++)
        por.push_back(ParseOptimizationResults("/Volumes/Untitled/data/maps_only/maps_only_2014/", _dates[i]));
    
    SLAMDraw draw(1000,1000);
    draw.SetScale(-300,300,-300,300);
    draw.ResetCanvas();
    
    cv::Mat img = draw.GetDrawing();
    
    std::vector<std::vector<int> > counting;
    std::vector<double> perc;
    std::vector<int> bins(10,0);
    std::vector<int> nallimgs(10,0);
    
    for(int d=0; d<aligndirs.size(); ++d) {
        std::string rootdir = basedir + aligndirs[d] + "/";
        std::vector<std::string> numaligned = FileParsing::ListFilesInDir(rootdir + "rf/", "jpg");
        std::vector<std::string> num = FileParsing::ListFilesInDir(rootdir + "scene/", "jpg");
        std::vector<int> counts = {static_cast<int>(numaligned.size()-1), static_cast<int>(num.size())};
        perc.push_back(1.0*counts[0]/counts[1]);
        counting.push_back(counts);
//        std::cout << d << " " <<aligndirs[d] << ": " << counting[d][0] << " of " << counting[d][1] << ": " << perc[d] << std::endl;
        std::cout << perc[d] << ", " << counting[d][0]<< std::endl;
        bins[static_cast<int>(perc[d]*10)]++;
//        bins[counts[0]/5]++;
        nallimgs[counts[1]/5]++;
        
        std::string refdate = aligndirs[d].substr(0,aligndirs[d].find("_"));
        std::string refpose = aligndirs[d].substr(aligndirs[d].find("_")+1, aligndirs[d].size());
        int rd = DateToIdx(stoi(refdate));
        int rp = stoi(refpose);
        
        circle(img, draw.Scale(cv::Point2f(por[rd].boat[rp][0], por[rd].boat[rp][1])), 7, CV_RGB(0,0,0), -1, 8, 0);
        circle(img, draw.Scale(cv::Point2f(por[rd].boat[rp][0], por[rd].boat[rp][1])), 6, CV_RGB((1-perc[d])*255,0,perc[d]*255), -1, 8, 0);
    }
    
    std::sort(perc.begin(), perc.end());
    for(int i=0; i<bins.size(); ++i) {
        std::cout << i*10 << ": " << bins[i] << std::endl;
//        std::cout << i*5 << ": " << nallimgs[i] << std::endl;
    }
    
    cv::Mat flipped;
    cv::flip(img, flipped, 0);
    img = flipped;
    
    cv::namedWindow("disp");
    cv::imshow("disp", img);
    char c = cvWaitKey(0);
    cv::destroyWindow("disp");
    if(c=='s')
    {
        static std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        
        try {
            cv::imwrite("/Users/shane/Desktop/place.png", img, compression_params);
        } catch (std::exception& ex) {
            fprintf(stderr, "PreprocessBikeRoute::ReadVideo Error. Exception converting image to JPG format: %s\n", ex.what());
            exit(-1);
        }
    }
    
    return;
}

void AlignICPImagePairs::LabelTimelapse() {
    std::string basedir = "/Users/shane/Documents/research/results/2018_winter/example_alignments/set_of_100/";
    std::vector<std::string> aligndirs = FileParsing::ListDirsInDir(basedir);
    
//    std::string rootdir = "/Users/shane/Documents/research/results/2018_winter/example_alignments/140625_to_others_examples/140625_1141/";
//    std::string rootdir = "/Users/shanehome/Documents/140502_571/";
    std::vector<std::vector<int> > counting;
    for(int d=0; d<aligndirs.size(); ++d)
    {
        std::string rootdir = basedir + aligndirs[d] + "/";
        std::string rfbase = rootdir + "rf/";
        std::string badimgs = rfbase + "bad/";
        int nbad = 0;
        std::string sfbase = rootdir + "mappoints/";
        std::vector<std::string> forref = FileParsing::ListFilesInDir(rootdir, "jpg");
        if(forref.size() == 0)
            continue;
        std::string sfref = rootdir + forref[0];
        
        std::vector<std::string> rffiles = FileParsing::ListFilesInDir(rfbase, "jpg");
        std::vector<std::string> warpsffiles = FileParsing::ListFilesInDir(sfbase, "jpg");
        
        cv::Mat ref = ImageOperations::Load(sfref);
        std::vector<int> counter(3,0);
        
        FlickeringDisplay fd;
        for(int i=0; i<rffiles.size(); i++) {
            cv::Mat Imrf = ImageOperations::Load(rfbase + rffiles[i]);
            
            char c = fd.FlickerImages(ref, Imrf);
            switch(c){
                case 'c':{
                    counter[0]++;
                    if(nbad++ == 0)
                        FileParsing::MakeDir(badimgs);
                    FileParsing::MoveFile(rfbase + rffiles[i], badimgs + rffiles[i]);
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
        
        std::cout << d << ":" << aligndirs[d] << " c,g,p: " << counter[0] << ", " << counter[1] << ", " << counter[2] << std::endl;
        counting.push_back(counter);
    }
    
    std::cout << "all results" << std::endl;
    for(int d=0; d<aligndirs.size(); ++d) {
        std::cout << d << ":" << aligndirs[d] << " " << counting[d][0] << ", " << counting[d][1] << ", " << counting[d][2] << std::endl;
    }
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
    
//    std::vector<int> idcs = {10, 18, 42, 66, 89, 91, 99,102,105,107,111,114,116,119,126,135,138,148,196,203,238,261,280,281,303,304,320,325,334,350,351,356,368,389,402,415,420,436,441,445,463,467,468,473,507,525,526,538,542,562,591,625,658,661,665,681,708,754,773,785,805,830,856,860,874,880,920,925,944,951,954,968,974,979,985,1000};
    //std::vector<int> idcs = {2,5,10,17,46,88,102,112,115,146,226,255,256,333,374,387,482,485,493,572,623,626,636,708,733,754,780,799,850,851,873,879,913,926,997};
//    std::vector<int> idcs = {2,8,10,15,30,39,49,56,61,102,107,135,138,176,200,210,211,213,217,222,238,244,250,253,254,264,303,304,320,325,333,341,345,350,366,374,393,402,420,425,451,454,464,467,472,481,507,513,538,539,541,542,546,562,567,571,591,602,609,611,615,623,625,628,637,655,659,661,681,683,690,691,701,708,716,725,747,754,760,772,785,789,796,805,808,815,824,830,853,856,858,870,874,880,925,927,928,930,932,940,942,943,944,954,965,967,974,976,985};
//    int n=0;
    
    FlickeringDisplay fd;
    for(int i=0; i<from20star.size(); i++){
//        if(i!=(idcs[n]-1))continue;
//        n++;
//        std::cout << "iteration " << i << std::endl;
        
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
    //2,5,10,17,46,88,102,112,115,146,226,255,256,333,374,387,482,485,493,572,623,626,636,708,733,754,780,799,850,851,873,879,913,926,997
    std::vector<int> idcs = {22,29,39,54,57,70,91,135,136,159,167,194,195,217,238,274,318,319,332,341,345,347,389,393,414,489,513,515,571,720,725,748,760,764,768,794,796,825,908,918,921,932,951};
    int n=0;
    
    FlickeringDisplay fd;
    for(int i=0; i<1000; i++){
        if(i!=(idcs[n]-1))continue;
        n++;
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

void AlignICPImagePairs::CheckRF() {
    std::string icporig = "/Volumes/Untitled/data/aligned_icp_2014_orig/";
    std::string file = icporig + "image_pairs.csv";
    std::string rfbase1 = "/Volumes/Untitled/data/aligned_rf_2014/";
    std::string rfbase0 = "/Volumes/Untitled/data/aligned_rf_2014_older_map/"; //aligned_rf_2014_no_constraints_new_map/";
    
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults("/Volumes/SAMSUNG/Data/origin/", _dates[i]));
    }
    
    std::vector<std::vector<int> > from20star = ReadCSVFile(file, 0, 999);
    std::vector<int> counter(3,0);
    
    std::vector<int> compa = {16961, 25585, 24619, 29496, 44487, 37034, 47989, 18879, 35076, 15381, 35119, 33002, 12711, 9917};
    std::vector<int> compb = {8153, 26480, 20045, 26927, 49914, 26947, 50498, 23027, 34340, 22136, 24600, 38077, 19048, 10895};
    
    int c=0;
    FlickeringDisplay fd;
    for(int i=0; i<1000; i++){
        if(not (from20star[i][2] == compa[c] and from20star[i][4] == compb[c])){
            continue;
        }
        c++;
        
        int d1 = DateToIdx(from20star[i][1]);
        int pose1 = por[d1].GetNearestPoseToImage(from20star[i][2]);
        std::cout << "1:closest image to " <<from20star[i][2] << " is " << pose1 << " with " << por[d1].cimage[pose1] << std::endl;
        
        std::string imagename = PaddedInt(i);
        std::string _image0_rf = ParseSurvey::GetImagePath(_query_loc + std::to_string(from20star[i][1]), por[d1].cimage[pose1]);
        std::string _imagerf0 = rfbase0 + imagename + "_w.jpg";
        std::string _imagerf1 = rfbase1 + imagename + "_w.jpg";
        
        cv::Mat refrf = ImageOperations::Load(_image0_rf);
        cv::Mat Imrf0 = ImageOperations::Load(_imagerf0);
        cv::Mat Imrf1 = ImageOperations::Load(_imagerf1);
        
        FixImagesForComparison(refrf, Imrf1);
        FixImagesForComparison(refrf, Imrf0);
        cv::Mat refx2 = FlickeringDisplay::CombinedImage(refrf, refrf);
        cv::Mat rfaligned = FlickeringDisplay::CombinedImage(Imrf1, Imrf0);
        
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
    
//    std::vector<int> compa = {16441};
//    std::vector<int> compb = {22349};
    
//    int c=0;
    FlickeringDisplay fd;
    for(int i=0; i<1000; i++){
        int d1 = DateToIdx(from20star[i][1]);
        int pose1 = por[d1].GetNearestPoseToImage(from20star[i][2]);
        std::cout << i<<":closest image to " <<from20star[i][2] << " is " << pose1 << " with " << por[d1].cimage[pose1] << std::endl;
        
//        if(not (from20star[i][2] == compa[c] and from20star[i][4] == compb[c])){
//            continue;
//        }
//        c++;
        
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
        lint.LoadLocalizations(_maps_dir + _dates[i], _dates);
        
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
    
    std::vector<std::vector<int> > numlocsper(_dates.size(), std::vector<int>(_dates.size(), 0) );
    
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
            int date = stoi(files[j].substr(idx+1, files[j].size()));
            int didx = DateToIdx(date);
            numlocsper[i][didx]++;
        }
        
        std::cout << _dates[i] << ", " << 1.0*localized_poses.size()/por.boat.size() << std::endl;
    }
    
    std::cout <<"ISC count table" << std::endl;
    for(int i=0; i<_dates.size(); i++)
    {
        for(int j=0; j<_dates.size(); j++)
        {
            std::cout << numlocsper[i][j] << ", ";
        } std::cout << std::endl;
    }
}

int AlignICPImagePairs::DaysBetween(std::string date1, std::string date2) {
    //https://stackoverflow.com/questions/14218894/number-of-days-between-two-dates-c
    
    int intdate1 = stoi(date1);
    int ye1 = intdate1/10000;
    int m1 = intdate1/100 - ye1*100;
    int d1 = intdate1 - ( ye1*10000 + m1*100 );
    struct std::tm a = {0,0,0,d1,m1-1,100+ye1};
    
    int intdate2 = stoi(date2);
    int ye2 = intdate2/10000;
    int m2 = intdate2/100 - ye2*100;
    int d2 = intdate2 - ( ye2*10000 + m2*100 );
    struct std::tm b = {0,0,0,d2,m2-1,100+ye2};
    
    std::time_t x = std::mktime(&a);
    std::time_t y = std::mktime(&b);
//    std::cout << "times for : " << date1 << ": {"<<d1<<","<<m1-1<<","<<100+ye1<<"}"<< std::ctime(&x) << ", " << date2 << ": " << std::ctime(&y) << std::endl;
    if ( x != (std::time_t)(-1) && y != (std::time_t)(-1) )
    {
        double difference = std::difftime(y, x) / (60 * 60 * 24);
        return difference;
    }
    return -1;
}

std::vector<double> AlignICPImagePairs::ConvertToWeighted(double c, std::vector<int>& counts, std::vector<int>& tots) {
    std::vector<double> weighted(counts.size(), 0);
    
    for(int i=0; i<counts.size(); ++i) {
        if(tots[i] == 0) continue;
        double a = pow(c, counts[i]);
        double b = pow(1-c, counts[i]);
        weighted[i] = a /(a+b);
    }
    return weighted;
}

void AlignICPImagePairs::CheckSessions(){
    std::vector<Map> maps;
    std::vector<ParseOptimizationResults> por;
    
    for(int i=0; i<_dates.size(); i++){
        maps.push_back(Map("/Volumes/Untitled/data/maps_only/maps_only_2014/"));
        maps[i].LoadMap(_dates[i]);
    }
    
    std::vector<ReprojectionFlow> rf;
    for(int i=0; i<_dates.size(); i++) {
        por.push_back(ParseOptimizationResults("/Volumes/Untitled/data/maps_only/maps_only_2014/", _dates[i]));
        rf.push_back(ReprojectionFlow(_cam, maps[i]));
        if(i>0) std::cout << "size "<< i-1 << ": " << rf[i-1].MapSize() << ", " << maps[i-1].map.size() << std::endl;
    }

    std::string basedir = "/Users/shane/Documents/research/results/2018_winter/example_alignments/set_of_100/";
    std::vector<std::string> aligndirs = FileParsing::ListDirsInDir(basedir);
    
    std::vector<std::vector<std::vector<double> > *> poselists;
    std::vector<ReprojectionFlow*> rfs;
    for(int i=0; i<_dates.size(); i++) {
        poselists.push_back(&(por[i].boat));
        rfs.push_back(&rf[i]);
    }
    
    MultiSurveyViewpointSelection msvs;
    
    for(int d=22; d<aligndirs.size(); ++d) {
        std::string rootdir = basedir + aligndirs[d] + "/";
        std::vector<std::string> num = FileParsing::ListFilesInDir(rootdir + "scene/", "jpg");
        
        std::string refdate = aligndirs[d].substr(0,aligndirs[d].find("_"));
        std::string refpose = aligndirs[d].substr(aligndirs[d].find("_")+1, aligndirs[d].size());
        int rd = DateToIdx(stoi(refdate));
        int rp = stoi(refpose);
        
        std::vector<int> poses(_dates.size(), -1);
        for(int i=0; i<num.size(); ++i) {
            std::string cdate = num[i].substr(0,num[i].find("_"));
            std::string cpose = num[i].substr(num[i].find("_")+1, num[i].size());
            
            for(int j=0; j<_dates.size(); ++j){
                if(_dates[j]==cdate) {
                    poses[j] = stoi(cpose);
                }
            }
        }
        
        std::vector<int> allviewpoints = msvs.FindTheSameViewpoints(rfs, poselists, _dates, por[rd].boat[rp], rd);
        
        int c = 0;
        for(int i=0; i<_dates.size(); ++i) {
            if(poses[i] != allviewpoints[i]) {
                std::cout << "Missed " << aligndirs[d] << " to " << _dates[i] << "_" << allviewpoints[i] <<", expected " << poses[i] << std::endl;
                c++;
            }
        }
        
        std::cout << "total missed at " << d << ", " << aligndirs[d] << " : " << c << std::endl;
    }
}


void AlignICPImagePairs::AlignmentQualityByPlace() {
    //TWO PARAMETERS:
    // 1) the number of days between sessions that are counted. controls for time between sessoins.
    // 2) the displayed locations along the shore.
    int PARAM1 = 365;
    int PARAM2 = 4;
    
    std::string icporig = "/Volumes/Untitled/data/aligned_icp_2014_orig/";
    std::string file = icporig + "image_pairs.csv";
    std::string rfbase = "/Volumes/Untitled/data/aligned_rf_2014/";
    std::string _visibility_dir = "/Volumes/SAMSUNG/Data/visibility_poses/all/";
    std::string labels_file = "/Users/shane/Documents/research/results/2018_winter/year-wise-labeling/rf_2014_with_constraints.txt";
    
    std::vector<std::vector<int> > from20star = ReadCSVFile(file, 0, 999);
    std::vector<std::vector<int> > flabels = ReadCSVFileLabels(labels_file);
    ParseVisibilityFile vis(_visibility_dir, "140625", "140613");
    int d0 = DateToIdx(atoi(vis.date1.c_str()));
    
    std::vector<Map> maps;
    std::vector<ParseOptimizationResults> por;
    
    for(int i=0; i<_dates.size(); i++){
        maps.push_back(Map("/Volumes/Untitled/data/maps_only/maps_only_2014/"));
        maps[i].LoadMap(_dates[i]);
    }
    
    std::vector<ReprojectionFlow> rf;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults("/Volumes/Untitled/data/maps_only/maps_only_2014/", _dates[i]));
        rf.push_back(ReprojectionFlow(_cam, maps[i]));
        if(i>0) std::cout << "size "<< i-1 << ": " << rf[i-1].MapSize() << ", " << maps[i-1].map.size() << std::endl;
    }
    
    std::vector<std::vector<double> >subset;
    for(int i=0; i<vis.boat1.size(); i++)
        subset.push_back(por[d0].boat[vis.boat1[i]]);
    
    double conf=0.65;
    std::vector<int> ss(vis.boat1.size(), 0);
    std::vector<int> counter(vis.boat1.size(), 0);
    std::vector<int> tots(vis.boat1.size(), 0);
    int none=0, all=0;
    std::cout << _dates[d0] << " with d0: " << d0 << " of " << rf.size() << ", with map size: " << rf[d0].MapSize() << ", map: " << maps[d0].map.size() << std::endl;
    
    for(int i=0; i<1000; i++) {
        int d1 = DateToIdx(from20star[i][1]);
        int pose1 = por[d1].GetNearestPoseToImage(from20star[i][2]);
        int d2 = DateToIdx(from20star[i][3]);
        
        if(std::abs(DaysBetween(_dates[d1], _dates[d2])) > PARAM1)
            continue;
        
//        double gstat;
//        int midx = rf[d0].IdentifyClosestPose(subset, por[d1].boat[pose1], &gstat, false);
        int midx=0;
        double ming=100000000000;
        for(int j=0; j<subset.size(); ++j){
            double dist = pow( pow(subset[j][0]-por[d1].boat[pose1][0], 2.0) + pow(subset[j][1]-por[d1].boat[pose1][1], 2.0), 0.5);
            if(dist < ming)
            {
                ming = dist;
                midx = j;
            }
        }
        
        if(midx < 0) {std::cout <<"none found to " << _dates[d1] << ":" << por[d1].cimage[pose1] << std::endl; none++; continue;}
        else if(midx > vis.boat1.size()) {std::cout << "oob " << std::endl; exit(-1);}
        if(flabels[i][4]==1) { counter[midx]++;
            ss[midx]++;
        } else ss[midx]--;
        tots[midx]++;
        all++;
    }
    
    SLAMDraw draw(1000,1000);
    draw.SetScale(-300,300,-300,300);
    draw.ResetCanvas();
    
//    //draw the estimated landmark points
//    for(int i=0; i<por[d0].landmarks.size(); i++) {
//        if(por[d0].landmarks[i][0] == 0 && por[d0].landmarks[i][1] == 0 && por[d0].landmarks[i][2] == 0) continue;
//        draw.AddPointLandmark(por[d0].landmarks[i][0], por[d0].landmarks[i][1], por[d0].landmarks[i][3]);
//    }
    
    std::vector<double> weighted = ConvertToWeighted(conf, ss, tots);
    
    int listpoint = 0;
    std::vector<int> printlist = {10622, 14742, 17442, 19392, 25082, 38872, 40412, 40502, 41702};
    
    cv::Mat img = draw.GetDrawing();
    int disp = 0;
    for(int i=0; i<counter.size(); i++) {
        if(tots[i] < PARAM2) continue;
        std::cout << counter[i] << "/" << tots[i] << ", " << weighted[i] << ", " << por[d0].cimage[vis.boat1[i]] << std::endl;
        disp++;
        
        double w = (1.0 * counter[i] / tots[i]);
        circle(img, draw.Scale(cv::Point2f(por[d0].boat[vis.boat1[i]][0], por[d0].boat[vis.boat1[i]][1])), 7, CV_RGB(0,0,0), -1, 8, 0);
//        circle(img, draw.Scale(cv::Point2f(por[d0].boat[vis.boat1[i]][0], por[d0].boat[vis.boat1[i]][1])), 6, CV_RGB((1-weighted[i])*255,0,weighted[i]*255), -1, 8, 0);
        circle(img, draw.Scale(cv::Point2f(por[d0].boat[vis.boat1[i]][0], por[d0].boat[vis.boat1[i]][1])), 6, CV_RGB((1-w)*255,0,w*255), -1, 8, 0);
//        circle(img, draw.Scale(cv::Point2f(por[d0].boat[vis.boat1[i]][0], por[d0].boat[vis.boat1[i]][1])), 5, CV_RGB(w*255,w*255,w*255), -1, 8, 0);
        
        if(listpoint < printlist.size() && por[d0].cimage[vis.boat1[i]] == printlist[listpoint]) {
            std::cout << por[d0].cimage[vis.boat1[i]] << ": pose #: " << vis.boat1[i] << std::endl;
            listpoint++;
        }
    }
    std::cout << std::endl;
    std::cout << "found: " << all << ", not found: " << none << ", displayed: " << disp << std::endl;
    
    cv::Mat flipped;
    cv::flip(img, flipped, 0);
    img = flipped;
    
    cv::namedWindow("disp");
    cv::imshow("disp", img);
    char c = cvWaitKey(0);
    cv::destroyWindow("disp");
    if(c=='s')
    {
        static std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        
        try {
            cv::imwrite("/Users/shane/Desktop/place.png", img, compression_params);
        } catch (std::exception& ex) {
            fprintf(stderr, "PreprocessBikeRoute::ReadVideo Error. Exception converting image to JPG format: %s\n", ex.what());
            exit(-1);
        }
    }
}

void AlignICPImagePairs::AlignmentQualityByPlace_SPECTRUM() {
    cv::Mat spec(cv::Size(300, 300), CV_8UC3, cv::Scalar::all(255));
    
    for(int i=0; i<255; i++){
        cv::line(spec, cv::Point2f(139, i+20), cv::Point2f(181, i+20), CV_RGB(0,0,0));
        cv::line(spec, cv::Point2f(140, i+20), cv::Point2f(180, i+20), CV_RGB(i,0,255-i));
    }
    
    cv::namedWindow("disp");
    cv::imshow("disp", spec);
    char c = cvWaitKey(0);
    cv::destroyWindow("disp");
    if(c=='s')
    {
        static std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        
        try {
            cv::imwrite("/Users/shane/Desktop/spectrum.png", spec, compression_params);
        } catch (std::exception& ex) {
            fprintf(stderr, "PreprocessBikeRoute::ReadVideo Error. Exception converting image to JPG format: %s\n", ex.what());
            exit(-1);
        }
    }
}

void AlignICPImagePairs::CreateTimeLapsesForEvaluation() {
    std::vector<ParseOptimizationResults> por;
    std::vector<Map> maps;
    for(int i=0; i<_dates.size(); i++){
        por.push_back(ParseOptimizationResults(_maps_dir, _dates[i]));
        maps.push_back(Map(_maps_dir));
        maps[i].LoadMap(_dates[i], por[i]);
    }
    
    ForBMVCFigure forfig(_cam, _dates, _pftbase, _query_loc, _maps_dir + "../", _nthreads);
    
    srand(892340);
    for(int i=0; i<110; ++i) //a few more than 100 to have enough for the evaluation of 100 time-lapses.
    {
        int d = rand()%_dates.size();
        int p = rand()%por[d].boat.size();
        if(i < 100) continue;
        std::cout << "aligning images to " << _dates[d] << ":" << p << std::endl;
        continue;
        forfig.MakeTimelapse(d, p, false, por, maps);
    }
}

void AlignICPImagePairs::ShowMaps() {
    SLAMDraw draw(4000,4000);
    draw.SetScale(-300,300,-300,300);
    draw.ResetCanvas();
    cv::Mat img = draw.GetDrawing();
    
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++){
        ParseOptimizationResults por(_maps_dir, _dates[i]); //  + "../origin/"
        
        CvScalar color = IMDraw::GetLandmarkColor(stoi(_dates[i]));
        
        for(int j=1; j<por.boat.size(); ++j) {
            double dist = pow(pow(por.boat[j][0] - por.boat[j-1][0],2) + pow(por.boat[j][1] - por.boat[j-1][1],2),0.5);
            
            cv::Point2f p0 = draw.Scale(cv::Point2f(por.boat[j-1][0], por.boat[j-1][1]));
            cv::Point2f p1 = draw.Scale(cv::Point2f(por.boat[j][0], por.boat[j][1]));
            
            if(dist > 2.5) {
                continue;
            }
            
            cv::line(img, p0, p1, color);
        }
        
        for(int j=1; j<por.landmarks.size(); j++) {
            CvScalar col = IMDraw::GetLandmarkColor((int) (j*_dates.size() + i));
            cv::circle(img, draw.Scale(cv::Point2f(por.landmarks[j][0], por.landmarks[j][1])), 7, col, -1, 8, 0);
        }
    }
    
    cv::Mat flipped;
    cv::flip(img, flipped, 0);
    img = flipped;
    
    cv::namedWindow("disp");
    cv::imshow("disp", img);
    char c = cvWaitKey(0);
    cv::destroyWindow("disp");
    if(c=='s')
    {
        static std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        
        try {
            cv::imwrite("/Users/shane/Desktop/place.png", img, compression_params);
        } catch (std::exception& ex) {
            fprintf(stderr, "PreprocessBikeRoute::ReadVideo Error. Exception converting image to JPG format: %s\n", ex.what());
            exit(-1);
        }
    }
}

void AlignICPImagePairs::ProjectToImage(const std::vector<double>& boat, const std::vector<gtsam::Point2>& orig_imagecoords, const std::vector<gtsam::Point3>& p) {
    gtsam::Pose3 tf = GTSAMInterface::VectorToPose(boat);
    
    double total_error = 0;
    double count = 0;
    double num_bad=0;
    std::string string_data = "";
    double sum=0;
    for(int j=0; j<orig_imagecoords.size(); j++) {
        if(p[j].x()==0.0 && p[j].y()==0.0 && p[j].z()==0.0) {std::cout << "point is zeros " << std::endl; continue; }
        
        gtsam::Point3 res = tf.transform_to(p[j]);
        gtsam::Point2 twodim = _cam.ProjectToImage(res);
        if(!_cam.InsideImage(twodim)) {
            std::cout << "point " << p[j] << " projected outside of the image to " << twodim << std::endl;
        }
        gtsam::Point2 orig = orig_imagecoords[j];
        
        double error = pow(pow(twodim.x() - orig.x(), 2)+pow(twodim.y() - orig.y(), 2),0.5);
        sum += error;
        count++;
        std::cout << "reprojection error: " << error << std::endl;
    }
    
    std::cout << "average reprojection error: " << sum/count << std::endl;
}

double AlignICPImagePairs::AngleBetweenTwoVectors(gtsam::Vector3 a, gtsam::Vector3 b) {
    return acos(a.dot(b) / (a.norm()*b.norm()));
}

void AlignICPImagePairs::AnalyzeManualLabels(std::string dir) {
    std::string loadf = dir + "localization_result.csv";
    std::vector<std::vector<std::string> > fs = FileParsing::ReadCSVFile(loadf);
    
    std::vector<ParseOptimizationResults> por;
    for(int i=0; i<_dates.size(); i++) {
        ParseOptimizationResults p(_maps_dir, _dates[i]);
        por.push_back(p);
    }
    
    for(int i=0; i<fs.size(); ++i) {
        int d1 = DateToIdx(stoi(fs[i][3]));
        int p1 = stoi(fs[i][4]);
        
        std::vector<double> p1f0(6, 0.0);
        for(int j=0; j<6; ++j)
            p1f0[j] = stod(fs[i][j+5]);
        
        double posd = pow(pow(p1f0[0] - por[d1].boat[p1][0], 2) + pow(p1f0[1] - por[d1].boat[p1][1], 2) + pow(p1f0[2] - por[d1].boat[p1][2], 2), 0.5);
        
        gtsam::Pose3 a = GTSAMInterface::VectorToPose(por[d1].boat[p1]);
        gtsam::Pose3 b = GTSAMInterface::VectorToPose(p1f0);
        b = gtsam::Pose3(b.rotation(), a.translation());
        
        gtsam::Cal3_S2::shared_ptr cal = _cam.GetGTSAMCam();
        gtsam::Point2 mid = gtsam::Point2(_cam.w()/2, _cam.h()/2);
        gtsam::PinholePose<gtsam::Cal3_S2> pca(a, cal);
        gtsam::Unit3 au3 = pca.backprojectPointAtInfinity(mid);
        gtsam::PinholePose<gtsam::Cal3_S2> pcb(b, cal);
        gtsam::Unit3 bu3 = pcb.backprojectPointAtInfinity(mid);
        
        double ang = acos(au3.dot(bu3)) * (180/M_PI); //AngleBetweenTwoVectors(au3, bu3);
        
        std::cout << posd << ", " << ang << std::endl;
        
        
//        std::cout << p1f0[0] - por[d1].boat[p1][0] << ", " << p1f0[1] - por[d1].boat[p1][1] << ", " << p1f0[2] - por[d1].boat[p1][2] << ", " << fabs(remainder(p1f0[3]-por[d1].boat[p1][3],2*M_PI))* (180/M_PI) << ", "  << fabs(remainder(p1f0[4]-por[d1].boat[p1][4],2*M_PI))* (180/M_PI) << ", " << fabs(remainder(p1f0[5]-por[d1].boat[p1][5],2*M_PI))* (180/M_PI) << std::endl;
    }
}

void AlignICPImagePairs::ConvertToGIFs(std::string dir) {
    std::vector<std::string> dirsindir = FileParsing::ListDirsInDir(dir);
    std::string save_dir = dir + "../gifs/";
    for(int i=0; i<dirsindir.size(); i++) {
        std::string name = dir + dirsindir[i] + "/scene/";
        std::string command = "ffmpeg -f image2 -framerate 3 -pattern_type glob -i '" + name + "*.jpg' -filter_complex \"[0:v] fps=3,scale=720:-1,split [a][b];[a] palettegen [p];[b][p] paletteuse\" " + save_dir + dirsindir[i] + ".gif";
        std::cout << command << std::endl;
//        system(command.c_str());
//        break;
    }
}












