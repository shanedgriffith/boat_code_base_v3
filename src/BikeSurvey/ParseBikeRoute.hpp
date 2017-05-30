#ifndef SRC_BIKEROUTE_PARSEBIKEROUTE_HPP_
#define SRC_BIKEROUTE_PARSEBIKEROUTE_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <FileParsing/ParseSurvey.hpp>

#include <DataTypes/Camera.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>

class ParseBikeRoute: public ParseSurvey{
private:
    
    std::vector<double> omega;
protected:
    void ReadDelimitedFile(std::string file, int type);
    void ProcessLineEntries(int type, std::vector<std::string>& lp);
public:
    ParseBikeRoute(std::string base):
    ParseBikeRoute(base) {
        constant_velocity = false;
        default_start = gtsam::Point3(296456.933, 5443544.892, 0);
        string aux = _base + _auxfile;
        if(!FileParsing.Exists(aux)){
            std::cout << "ParseBikeRoute Error: The aux file " << aux << " doesn't exist." << std::endl;
            exit(-1);
        }
        ReadDelimitedFile(aux, 0);
    }
    
    static Camera GetCamera();
    int GetImageNumber(int auxidx);
    int GetIndexOfImage(int image){return image;}
    double GetAvgAngularVelocity(int sidx, int eidx);
    bool Useable(int idx){return true;}
};



#endif /* SRC_BIKEROUTE_PARSEBIKEROUTE_HPP_ */

