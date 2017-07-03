/*
 * ParseBikeRoute.hpp
 *
 *  Created on: May 1, 2017
 *      Author: shane
 */

#ifndef SRC_BIKESURVEY_PREPROCESSBIKEROUTE_HPP_
#define SRC_BIKESURVEY_PREPROCESSBIKEROUTE_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include "ImageModification.hpp"
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/Camera.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <DataTypes/LandmarkTrack.h>

class PreprocessBikeRoute: public FileParsing{
private:
    double move_avg_win = 20;
    int video_fps;
    gtsam::Point3 default_start;
    gtsam::Point3 end_pos;
    std::vector<LandmarkTrack> active;
    
    std::vector<double> YPRToRotationMatrix(double X, double Y, double Z);
    std::vector<double> ComposeRotationMatrices(std::vector<double> A, std::vector<double> B);
    double CombineAngles(double a1, double a2, double w);
    std::vector<double> RotationMatrixToRPY(std::vector<double> R);
    
    void WriteImage(cv::Mat image, std::string filepath);
    double GetNearestTimeToPosition(double x, double y);
    double InterpolateValue(double t, double vals, double vale, double s, double e);
    void LowPassFilter(std::vector<double>& arr, double std);
    void AlignDataToImages();
    void PlayPoses();
    void GetPoses();
    void MakeAux();
    void ProcessRawVideo();
protected:
    void ReadDelimitedFile(std::string file, int type);
    void ProcessLineEntries(int type, std::vector<std::string>& lp);
public:
    std::string _bdbase;
    std::string _name;
    
    std::vector<double> timings;
    std::vector<std::vector<double> > arrs;
    std::vector<std::vector<double> > poses;
    
    PreprocessBikeRoute(std::string bdbase, std::string name):
        _bdbase(bdbase), _name(name), video_fps(29)
    {
        //296695.69732963, 5442567.53375366, 0); //location of GTL.
        default_start = gtsam::Point3(296456.933, 5443544.892, 0); //location of survey start.
        end_pos = gtsam::Point3(294812.524-default_start.x(), 5444335.780-default_start.y(), 0);
        
        std::string raw_aux = _bdbase + _name + "/" + _name + ".csv";
        if(!FileParsing::Exists(raw_aux)){
            std::cout << "ParseBikeRoute Survey doesn't exist. " << raw_aux << std::endl;
            exit(1);
        }
    }
    
    void Preprocess();
    void Play();
    void FindKLTParams();
    std::string Base() {return _bdbase + _name + "/";}
};



#endif /* SRC_BIKEROUTE_PREPROCESSBIKEROUTE_HPP_ */


/*
 REFs:
  https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
summary: http://stackoverflow.com/questions/35061294/how-to-calculate-heading-using-gyro-and-magnetometer

 For a simple compatibility workaround, add these:
 vector<vector<double> > boat;   (from visual odometry)
 vector<double> omega;      (average angular velocity; can be derived from the )
 vector<double> cam_pan;    1.569978 (no camera transitions)
 vector<int> imageno;       (1-1 correspondence with the index)
 
 */















