#pragma once


#include <FileParsing/FileParsing.hpp>
#include <DataTypes/Camera.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <DataTypes/LandmarkTrack.h>

#include <vector>
#include <string>
#include <iostream>


class PreprocessDroneRun: public FileParsing {
private:
    double move_avg_win = 20;
    int video_fps;
    gtsam::Point3 default_start;
    gtsam::Point3 end_pos;
    std::vector<LandmarkTrack> active;
    std::string first_line;

    std::vector<std::string> ParseLine(char * line);
    
protected:
    void ReadDelimitedFile(std::string file, int type);
    void ProcessLineEntries(int type, std::vector<std::string>& lp);
    
public:
    std::string _base;
    std::string _name;
    
    std::vector<double> timestamps;
    std::vector<std::string> paths;
    
    PreprocessDroneRun(std::string base, std::string name):
    _base(base), _name(name)
    {
        std::string img_file = _base + name + "/images.txt";
        ReadDelimitedFile(img_file, 0);
    }
    
    void ProcessRawVideo();
    void FindKLTParams();
    
    

};
