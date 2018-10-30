/*
 * LocalizedPoseData.hpp
 *
 *  Created on: Jan 26, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_LOCALIZEDPOSEDATA_HPP_
#define SRC_RFLOWOPTIMIZATION_LOCALIZEDPOSEDATA_HPP_

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

#include <FileParsing/FileParsing.hpp>
#include <DataTypes/Camera.hpp>

class LocalizedPoseData: public FileParsing{
protected:
    double ACCEPTABLE_RERROR = 6.0;
    double ACCEPTABLE_OVERLAP = 0.25;

    static const std::string lpath;

    static void CheckLine(std::string& filepath, int numr, int expected);

public:
    std::string date0, date1;
    //this is used for the "back-end", since the relative frames are subject to change as new surveys are reoptimized.
    std::vector<double> tf_p0_to_p1frame0;
    int s0time=0;
    int s1time=0;
    std::vector<double> p1frame0;
    std::vector<double> p0frame1;
    std::vector<int> pids;
    std::vector<gtsam::Point3> p3d;
    std::vector<gtsam::Point3> p3d0;
    std::vector<gtsam::Point2> p2d1;
    std::vector<double> rerrorp;
    std::vector<int> bids;
    std::vector<gtsam::Point3> b3d;
    std::vector<gtsam::Point3> b3d1;
    std::vector<gtsam::Point2> b2d0;
    std::vector<double> rerrorb;
    double perc_dc=0;
    double avg_rerror_inl=0;

    LocalizedPoseData(std::string d0, std::string d1, int s0time_, int s1time_):
        date0(d0), date1(d1),
        tf_p0_to_p1frame0(6),
        s0time(s0time_), s1time(s1time_),
        p1frame0(6), p0frame1(6), p3d(0), b3d(0),
        p3d0(0), p2d1(0), rerrorp(0),
        b3d1(0), b2d0(0), rerrorb(0),
        perc_dc(0), avg_rerror_inl(0)
        {}

    LocalizedPoseData():LocalizedPoseData("", "", 0,0) {}

    LocalizedPoseData(const LocalizedPoseData& l):
        date0(l.date0), date1(l.date1), tf_p0_to_p1frame0(6),
        s0time(l.s0time), s1time(l.s1time),
        p1frame0(6), p0frame1(6), p3d(l.p3d.size()), b3d(l.b3d.size()),
        pids(l.pids.size()), bids(l.bids.size()),
        p3d0(l.p3d0.size()), p2d1(l.p2d1.size()), rerrorp(l.rerrorp.size()),
        b3d1(l.b3d1.size()), b2d0(l.b2d0.size()), rerrorb(l.rerrorb.size()),
        perc_dc(l.perc_dc), avg_rerror_inl(l.avg_rerror_inl){

        tf_p0_to_p1frame0.assign(l.tf_p0_to_p1frame0.begin(), l.tf_p0_to_p1frame0.end());
        p1frame0.assign(l.p1frame0.begin(), l.p1frame0.end());
        p0frame1.assign(l.p0frame1.begin(), l.p0frame1.end());
        p3d.assign(l.p3d.begin(), l.p3d.end());
        b3d.assign(l.b3d.begin(), l.b3d.end());
        pids.assign(l.pids.begin(), l.pids.end());
        bids.assign(l.bids.begin(), l.bids.end());
        p3d0.assign(l.p3d0.begin(), l.p3d0.end());
        p2d1.assign(l.p2d1.begin(), l.p2d1.end());
        rerrorp.assign(l.rerrorp.begin(), l.rerrorp.end());
        b3d1.assign(l.b3d1.begin(), l.b3d1.end());
        b2d0.assign(l.b2d0.begin(), l.b2d0.end());
        rerrorb.assign(l.rerrorb.begin(), l.rerrorb.end());
    }

    friend void swap(LocalizedPoseData& first, LocalizedPoseData& second);
    LocalizedPoseData& operator=(LocalizedPoseData other);
    bool operator < (const LocalizedPoseData& str) const;
    
    gtsam::Pose3 GetTFP0ToP1F0();
    gtsam::Pose3 GetP0frame1();
    gtsam::Pose3 GetP1frame0();

    void SetPoints(std::vector<unsigned char>& inliers, std::vector<gtsam::Point2>& p2d, std::vector<gtsam::Point3>& p3d, std::vector<int> ids, int bot, int top);
    void SetPoses(std::vector<double> p0, std::vector<double> p1frame0_, std::vector<double> p0frame1_);
    void SetLocalizationQuality(double pdc, double rerror);

    double VerifyWith(Camera& _cam, LocalizedPoseData& lpd, gtsam::Pose3 p1_t, gtsam::Pose3 p1_tm1);
    
    bool IsSet();

    void Print(std::string opt = "");

    void Save(std::string toppath, std::string altpath = "");
    std::string GetPath(std::string toppath, std::string altpath="");
    static LocalizedPoseData Read(std::string path);
    static std::vector<LocalizedPoseData> LoadAll(std::string toppath, std::string altpath = "", std::vector<std::string> dates = {});
    
    void CheckLPD(Camera& _cam, std::string _pftbase, std::string _results_dir, std::string _query_loc);
};



#endif /* SRC_RFLOWOPTIMIZATION_LOCALIZEDPOSEDATA_HPP_ */
