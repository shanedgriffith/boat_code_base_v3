

#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseSurvey.h>
#include <Visualizations/FlickeringDisplay.h>

#include "AlignICPImagePairs.hpp"


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

void AlignICPImagePairs::AlignTimelapsesRFlow(std::string dirnum){
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
        
        for(int i=3; i<39; i++){
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

void AlignICPImagePairs::AlignImagesRFlow(std::string file, int firstidx, int lastidx){
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


void AlignICPImagePairs::AlignImagesWarped(){
    
    std::vector<std::string> images={"000004", "000033", "000046", "000047", "000062", "000092", "000101", "000125", "000138", "000172", "000175", "000196", "000215", "000227", "000235", "000251", "000256", "000271", "000287", "000309", "000310", "000313", "000349", "000373", "000375", "000412", "000413", "000423", "000430", "000459", "000460", "000467", "000476", "000482", "000488", "000529", "000546", "000567", "000575", "000610", "000617", "000639", "000654", "000669", "000673", "000685", "000689", "000704", "000716", "000753", "000793", "000825", "000831", "000852", "000859", "000864", "000865", "000872", "000878", "000888", "000892", "000899", "000924", "000935", "000987", "000994"};
    
    for(int i=0; i<images.size(); i++){
        int tidx = man.GetOpenMachine();
        std::string _image0 = "/home/shaneg/data/icp_pairs_0000/warp/" + images[i] + "_warp_1.jpg";
        std::string _image1 = "/home/shaneg/data/icp_pairs_0000/warp/" + images[i] + "_warp_2.jpg";
        std::string savename = "/home/shaneg/results/aligned_icp/" + images[i] + "_warp_w.jpg";

        ws[tidx]->SetImages(_image0, _image1, savename);
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
}


void AlignICPImagePairs::AlignImagesSFlow(std::string file, int firstidx, int lastidx){
    std::vector<std::vector<int> > imgparams = ReadCSVFile(file, firstidx, lastidx);
    
    for(int i=0; i<imgparams.size(); i++){
        std::cout << "aligning: "<<imgparams[i][0] << ", " <<imgparams[i][1] <<"." << imgparams[i][2] << " to " << imgparams[i][3] <<"." << imgparams[i][4] << std::endl;
        int tidx = man.GetOpenMachine();
        char filename[100];
        sprintf(filename, "%s%06d_w.jpg", _savebase.c_str(), imgparams[i][0]);
        std::string savename(filename);
        std::string _image0 = ParseSurvey::GetImagePath(_query_loc + std::to_string(imgparams[i][1]), imgparams[i][2]);
        std::string _image1 = ParseSurvey::GetImagePath(_query_loc + std::to_string(imgparams[i][3]), imgparams[i][4]);
        
        ws[tidx]->SetImages(_image0, _image1, savename);
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    
    std::cout << "Finished aligning images  " << std::endl;
}


std::vector<std::vector<int> > AlignICPImagePairs::ReadCSVFile(std::string file, int firstidx, int lastidx){
    FILE * fp = fopen(file.c_str(), "r");
    if(!fp){
        std::cout << "couldn't open " << file << std::endl;
        exit(1);
    }
    
    std::vector<std::vector<int> > imgparams;
    
    std::cout << "Aligning images between " << firstidx << " and " << lastidx << std::endl;
    while(!feof(fp)){
        int idx, date0, img0num, date1, img1num;
        double a, b, c, x, y, z;
        int ret = fscanf(fp, "%d,%d,%d,%lf,%lf,%lf,%d,%d,%lf,%lf,%lf",
               &idx, &date0, &img0num, &a, &b, &c, &date1, &img1num, &x, &y, &z);
        
        if(ret != 11){
            std::cout << "error reading file: got " << ret << " of 11" << std::endl;
            break;
        }
        
        if(idx < firstidx) continue;
        else if(idx > lastidx) break;
        
        if(date0/10000 != 14 || date1/10000 != 14) continue;
        
        
        std::vector<int> toalign = {idx, date0, img0num, date1, img1num};
        imgparams.push_back(toalign);
    }
    fclose(fp);
    return imgparams;
}

void AlignICPImagePairs::GetResults(){
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
    std::string reference = rootdir + "reference.jpg";
//    std::string wbase = "/Users/shane/Documents/research/experiments/pairs_unaligned/warp/";
    //    std::string sfbase = "/Users/shane/Documents/research/experiments/sf_pairs/";
//    std::string file = "/Users/shane/Documents/research/experiments/image_pairs.csv";
    
    
    std::vector<std::string> rffiles = FileParsing::ListFilesInDir(rfbase, "jpg");
    std::vector<std::string> warpsffiles = FileParsing::ListFilesInDir(sfbase, "jpg");
    
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
        
        cv::Mat refx2 = FlickeringDisplay::CombinedImage(ref, ref);
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

































