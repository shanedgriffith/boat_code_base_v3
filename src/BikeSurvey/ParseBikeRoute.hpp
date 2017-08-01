#ifndef SRC_BIKESURVEY_PARSEBIKEROUTE_HPP_
#define SRC_BIKESURVEY_PARSEBIKEROUTE_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <FileParsing/ParseSurvey.h>

#include <DataTypes/Camera.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>

class ParseBikeRoute: public ParseSurvey{
private:
    
    std::vector<double> omega;
    gtsam::Pose3 VectorToPose(std::vector<double>& p);
    std::vector<double> InterpolatePoses(int idx, int a, int b, std::vector<double> pa, std::vector<double> pb);
    bool DistanceCriterion(std::vector<double>& pose1, std::vector<double>& pose2);
    void ModifyPoses();
protected:
    void ReadDelimitedFile(std::string file, int type);
    void ProcessLineEntries(int type, std::vector<std::string>& lp);
public:
    ParseBikeRoute(std::string bdbase, std::string name):
    ParseSurvey(bdbase, bdbase, name) {
        constant_velocity = false;
        default_start = gtsam::Point3(296456.933, 5443544.892, 0);
        
        std::string aux = _base + _auxfile;
        if(!FileParsing::Exists(aux)){
            std::cout << "ParseBikeRoute Error: The aux file " << aux << " doesn't exist." << std::endl;
            exit(-1);
        }
        ReadDelimitedFile(aux, 0);
//        ModifyPoses();
    }
    
    static Camera GetCamera();
    int GetImageNumber(int auxidx);
    int GetIndexOfImage(int image){return image;}
    double GetAvgAngularVelocity(int sidx, int eidx);
    bool Useable(int cidx, int lcidx){return true;}
    std::vector<double> GetDrawScale();
};



#endif /* SRC_BIKESURVEY_PARSEBIKEROUTE_HPP_ */

