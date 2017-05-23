//
//  AlignmentResult.h
//  SIFTFlow
//
//  Created by Shane Griffith on 6/23/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SIFTFlow_AlignmentResult_h
#define SIFTFlow_AlignmentResult_h

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <ImageAlignment/DREAMFlow/ImageOperations.h>

#include <gtsam/geometry/Point2.h>
#include <DataTypes/Camera.hpp>

#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

class AlignmentResult{
private:
    bool debug = false;
    bool stats_computed = false;
    //energy statistics
    double _Min=1000000, _Max=0, _Total=0, _Avg=0;
    bool have_inverted = false;
public:
    //the flow. 2 channels.
    cv::Mat flow;
    cv::Mat inverted_flow;
    
    cv::Mat siftref;
    cv::Mat sift2;
    
    //image of alignment energy, which indicates where changes occurred.
    cv::Mat energyimage;
    
    //in case a mask was used to bias the alignment.
    cv::Mat mask;
    
    //(optional) the images used to generate the flow and the warped image.
    cv::Mat ref, im2;//, warped; //the warped image can be computed on the fly using im2 and flow.
    
    //the low-res image of pixels that were consistent on forward and backward alignments.
    cv::Mat consistent_set;

    double alignment_energy=0;
    double alignment_energy_lowres = 0;
    double alignment_energy_desc=0;
    double consistency = 0;
    double verified_ratio = 0;
    
    double computation_time=0;
    
    int _height=0, _width=0;
    
    AlignmentResult(){}
    
    AlignmentResult(int height, int width): _height(height), _width(width) {
        flow = cv::Mat(height, width, CV_64FC2, cv::Scalar::all(0));
        inverted_flow = cv::Mat(_height, _width, CV_64FC2, cv::Scalar::all(-1000));
        energyimage = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(-1));
        mask = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
        ref = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
        im2 = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
    }
    
    AlignmentResult(std::string directory){
        LoadFileInfo(directory);
        LoadImages(directory);
    }
    
    void SetDebug(){
        debug = true;
    }
    
    void SetSIFT(cv::Mat i1, cv::Mat i2){
        siftref = i1.clone();
        sift2 = i2.clone();
    }
    
    void SetFlow(double * v){
        memcpy(flow.data, v, flow.total()*flow.channels()*sizeof(double));
    }
    
    void SetEnergy(double * e){
        memcpy(energyimage.data, e, energyimage.total() * sizeof(double));
    }
    
    void SetMask(unsigned char * m){
        memcpy(mask.data, m, flow.total());
    }
    
    void SetReferenceImage(unsigned char * r){
        memcpy(ref.data, r, ref.total()*ref.channels());
    }
    
    void SetSecondImage(unsigned char * s){
        memcpy(im2.data, s, im2.total()*im2.channels());
    }
    
    double * GetFlow(){
        return (double*)flow.data;
    }
    
    double * GetEnergy(){
        return (double*)energyimage.data;
    }
    
    void SetAlignmentEnergy(double e, double e_d=-1){
        alignment_energy = e;
        alignment_energy_desc = e_d;
    }
    
    void SetConsistency(double c){
    	consistency = c;
    }

    void SetVerifiedRatio(double v){
        verified_ratio = v;
    }

    double GetAlignmentEnergy(){
        return alignment_energy;
    }
    
    unsigned char * GetMask(){
        return mask.data;
    }
    
    void SetComputationTime(double t){
        computation_time = t;
        if(debug) printf ("It took you %.2lf seconds to get the image alignment result.\n", computation_time);
    }
    
    void AddComputationTime(double t){
        computation_time += t;
    }
    
    void SetConsistentSet(unsigned char * cs, int r, int c) {
    	consistent_set = cv::Mat(r, c, CV_8UC2, cv::Scalar::all(0));
		memcpy(consistent_set.data, cs, r*c*consistent_set.channels());
    }

    void SaveUncompressedImage(cv::Mat& img, std::string name){
        if(name.length() == 0) return;
        ImageOperations::writeUncompressedImage(img, name);
    }
    
    Mat GetWarpedImage(){
        return ImageOperations::CreateWarpedImage(im2, flow);
    }
    
    void SaveWarpedImage(std::string swarp){
        if(swarp.length() == 0)return;
        cv::Mat warped = GetWarpedImage();
        ImageOperations::Save(warped, swarp);
    }
    
    void SaveImages(std::string directory){
//        SaveUncompressedImage(flow, directory + "flow.png");
//        SaveUncompressedImage(energyimage, directory + "energy.png");
        ImageOperations::Save(im2, directory + "im2.jpg");
        ImageOperations::Save(ref, directory + "ref.jpg");
        if(flow.rows > 0) SaveWarpedImage(directory + "warped.jpg");
    }
    
    void SaveFileInfo(std::string directory){
        mkdir(directory.c_str(), (mode_t) (S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH));
        std::string filename = directory + "info.txt";
        FILE * fs = fopen(filename.c_str(), "w");
        if(!fs) {std::cout << "FileParsing: Error. Couldn't open " << filename << "." << std::endl; exit(-1);}
        fprintf(fs, "%d,%d,%lf,%lf\n", _height, _width, alignment_energy, computation_time);
        fclose(fs);
    }
    
    void Save(std::string directory){
        if(debug) cout << "save to: " << directory << endl;
        SaveFileInfo(directory);
        SaveImages(directory);
    }
    
    void LoadFileInfo(std::string directory){
        std::string filename = directory + "info.txt";
        FILE * fp = fopen(filename.c_str(), "r");
        if(!fp) {std::cout << "FileParsing: Error. Couldn't open " << filename << "." << std::endl; exit(-1);}

        char line[1024]="";
        
        if (fgets(line,1023,fp)==NULL ) {
            cout<<"AlignmentResult: Can't parse file "<< filename<<endl;
            exit(-1);
        }
        
        if (sscanf(line,"%d,%d,%lf,%lf",
                   &_height,&_width,&alignment_energy,&computation_time)!=4) {
            cout<<"AlignmentResult: Can't parse file "<< filename<<endl;
            exit(-1);
        }
        fclose(fp);
    }
    

    void LoadImages(std::string directory){
        flow = ImageOperations::readUncompressedImage(directory + "flow.png");
        energyimage = ImageOperations::readUncompressedImage(directory + "energy.png");
        im2 = ImageOperations::Load(directory + "im2.jpg");
        ref = ImageOperations::Load(directory + "ref.jpg");
    }
    
    void CreateInvertedFlow(){
        double * map = (double*)flow.data;
        double * res = (double*)inverted_flow.data;
        cv::Mat check(_height, _width, CV_8UC1, cv::Scalar::all(0));
        unsigned char * mark = (unsigned char *) check.data;
        
        for(int i=0; i<_height; i++){
            for(int j=0; j<_width; j++){
                int idx = (i*_width + j)*2;
                double fx = map[idx];
                double fy = map[idx+1];
                int locx = j + fx;
                int locy = i + fy;
                if(locx < 0 || locy < 0 || locx >= _width || locy >= _height)
                    continue;
                int i_idx = (int) (locy*_width + locx)*2;
                if(i_idx >= _width*_height*2){
                    cout << "("<<locx<<","<<locy<<")"<<endl;
                    cout << "trying to place data at: " << i_idx <<" of "<< _width*_height << endl;
                    cout << "heightxwidth: ("<<_height<<", " <<_width<<")"<<endl;
                    exit(1);
                }
                if(mark[i_idx/2]==1){
                	//in case of reflections, use the flow that's closest to the original pixel.
                	if(abs(res[i_idx]) + abs(res[i_idx+1]) < abs(fx) + abs(fy)) continue;
                }
                res[i_idx] = -fx;
                res[i_idx+1] = -fy;
                mark[i_idx/2] = 1;
            }
        }
    }

    gtsam::Point2 MapPoint(Camera _cam, gtsam::Point2 p, bool forward = true){
        double * map;
        if(!forward){
            if(!have_inverted){
                CreateInvertedFlow();
                have_inverted = true;
            }
            map = (double *)inverted_flow.data;
        }
        else{
            map = (double*)flow.data;
        }
        
        double mult = _cam.w()/_width;
        
        int x = p.x()/mult;
        int y = p.y()/mult;
        int idx = (y*_width + x)*2;
        
        double fx = map[idx];
        double fy = map[idx+1];
        
        //don't subtract; the negative is already performed in the inverted_flow.
        gtsam::Point2 mp(mult*fx+p.x(), mult*fy+p.y());

        if(!_cam.InsideImage(mp)) return gtsam::Point2(-1,-1);
        
        return mp;
    }
    
    
    std::vector<std::vector<double> > MapPointAndGetInfo(Camera _cam, gtsam::Point2 p, bool forward = true){
        gtsam::Point2 mp = MapPoint(_cam, p, forward);
        
        if(mp.x()==-1) return {};
        double * nrg = (double *) energyimage.data;
        gtsam::Point2 ep = p;
        unsigned char * imsift = (unsigned char  *) siftref.data; //TODO: check.
        if(!forward){
            ep = mp;
            imsift = (unsigned char  *) sift2.data; //TODO: check.
        }

        std::vector<double> mappedpoint = {mp.x(), mp.y()};
        std::vector<double> inlier = {(double) 0};
        
        //energy
        double mult = _cam.w()/_width;
        int x = ep.x()/mult;
        int y = ep.y()/mult;
        int idx = (y*_width + x);
        double epoint = nrg[idx];
        std::vector<double> energy = {epoint};
        
        //sift descriptor.
        //Mat imsift(sift_height,sift_width,CV_8UC(siftdim), Scalar::all(0));
        mult = _cam.w()/siftref.cols;
        x = p.x()/mult;
        y = p.y()/mult;
        idx = (y*siftref.cols + x)*128;
        std::vector<double> desc;
        for(int i=0; i<128; i++){
            desc.push_back(imsift[idx+i]);
        }
        
        std::vector<std::vector<double> > ret = {mappedpoint, inlier, energy, desc};
        return ret;
    }
    
    bool IsPointConsistent(Camera& _cam, gtsam::Point2 p, int imset){
    	if(consistent_set.rows==0) return true;

    	int width = consistent_set.cols;
		int height = consistent_set.rows;
		double scale = _cam.w()/width;

		double x = round(p.x()/scale);
		double y = round(p.y()/scale);
		int idx = y*width + x;
		if(idx > width*height) return false;
		else return ((unsigned char *)consistent_set.data)[idx + imset];
    }

    gtsam::Point2 GetAverageFlow(){
        double * f = (double *) flow.data;
        double sumx=0, sumy=0;
        int countbad = 0;
        int countnan=0;
        for(int i=0; i<_height; i++){
            for(int j=0; j<_width; j++){
                int idx = (i*_width + j)*2;
                double fx = f[idx];
                double fy = f[idx+1];
                if(j+fx < 0 || i+fy < 0 || j+fx >= _width || i+fy >= _height) {
                    countbad++;
                    continue;
                }
                if(std::isnan(fx) || std::isnan(fy)){
                    countnan++;
                    cout <<"AlignmentResult: likely image size issue."<<endl;
                    continue;
                }
//                cout << "sum: (" <<sumx<<","<<sumy<<")"<<endl;
                sumx += f[idx];
                sumy += f[idx+1];
            }
        }
        cout << "percent bad =" <<100.0*countbad/(_width*_height)<<endl;
        cout << "number nan =" <<countnan<<endl;
        
        return gtsam::Point2(sumx/(1.0*_width*_height), sumy/(1.0*_width*_height));
    }
    
    std::vector<double> ComputeStats() {
        if(!stats_computed){
            double * en = (double *) energyimage.data;
            _Min=en[0];
            _Max=en[0];
            _Total = 0;
            int nElements = (int) energyimage.total();
            int count = 0;
            
            //cost function for 2*weighted good region.
            for(int i=0;i<nElements;i++) {
                double w=2.0/3.0; //for a multiplier of 1.5
                if(mask.data[i] < 50) w=1;
                double score = w*en[i];
                
                _Min=std::min(_Min,score);
                _Max=std::max(_Max,score);
                _Total += score;
                count++;
            }
            
            _Avg = _Total/count;
            stats_computed = true;
        }
        
        if(debug)
            cout << "min, max: " << _Min << ", " << _Max << ", total: " << _Total << ", avg: " << _Avg << endl;
        
        std::vector<double> stats;
        stats.push_back(_Min);
        stats.push_back(_Max);
        stats.push_back(_Total);
        stats.push_back(_Avg);
        return stats;
    }
};





#endif
