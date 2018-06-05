

#ifndef SRC_EVALUATION_SESSIONCONVERGENCE_HPP_
#define SRC_EVALUATION_SESSIONCONVERGENCE_HPP_



#include <FileParsing/ParseFeatureTrackFile.h>
#include <FileParsing/ParseOptimizationResults.h>
#include <DataTypes/Camera.hpp>
#include <BoatSurvey/ParseBoatSurvey.hpp>
#include <FileParsing/FileParsing.hpp>
#include <DataTypes/Map.hpp>
#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>    //calibration and performs projections


class SessionConvergence {
private:
    
    double MeasureREDifference(std::vector<double>& bref, std::vector<double>& bcur, std::vector<gtsam::Point3>& p3dref, std::vector<gtsam::Point3>& p3dcur);
    
    Camera& _cam;
public:
    std::string _pftbase;
    
    SessionConvergence(Camera cam, std::string pftbase):
    _cam(cam), _pftbase(pftbase) {}
    
    void CompareSessions();
};




#endif /* SRC_EVALUATION_SESSIONCONVERGENCE_HPP_ */




























































