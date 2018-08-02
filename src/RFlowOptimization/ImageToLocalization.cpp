/*
 * ImageToLocalization.cpp
 *
 *  Created on: Feb 22, 2017
 *      Author: shane
 */


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <FileParsing/ParseSurvey.h>
#include <FileParsing/FileParsing.hpp>
#include <FileParsing/ParseFeatureTrackFile.h>
#include <DataTypes/AlignmentResult.h>

#include "SFlowDREAM2RF.hpp"
#include "LocalizePose.hpp"

#include "ImageToLocalization.hpp"

using namespace std;
using namespace cv;

CvScalar ImageToLocalization::GetLandmarkColor(int id){
    int red = (71*(id%10) + id%255)%255;
    int green = (111*(id%10) + (2*id)%255)%255;
    int blue = (27*(id%10) + (3*id)%255)%255;
    return CV_RGB(red, green, blue);
}

void ImageToLocalization::DrawFlowPoints(cv::Mat& imageA, cv::Mat& imageB, std::vector<double> inliers, vector<gtsam::Point3>& p3, std::vector<gtsam::Point2>& p2dB, gtsam::Pose3 posea, gtsam::Pose3 poseb){
    double sum=0.0;
    int count=0;
    for(int i=0; i<p2dB.size(); i++){
        if(inliers[i] > 6.0) continue;
        gtsam::Point3 orig = posea.transform_to(p3[i]);
        gtsam::Point2 projA = _cam.ProjectToImage(orig);
        if(!_cam.InsideImage(projA)) continue;
        gtsam::Point3 res = poseb.transform_to(p3[i]);
        gtsam::Point2 projB = _cam.ProjectToImage(res);
        if(!_cam.InsideImage(projB)) continue;
        count++;
        double dist = p2dB[i].dist(projB);
        sum+=dist;
//        if(dist > 2*inliers[i]) {
//            std::cout << "projected to " << projB.x() << ", " << projB.y() <<"; dist: " <<dist << std::endl;
//        }
        CvScalar col = GetLandmarkColor(i);
        CvScalar lblcol = CV_RGB(0,255,0);
        if(dist>6) {
            lblcol = CV_RGB(255,0,0);
            circle(imageB, Point(p2dB[i].x(),p2dB[i].y()), 2, lblcol, -1);
            circle(imageA, Point(projA.x(), projA.y()), 1, col, -1);
            circle(imageB, Point(p2dB[i].x(),p2dB[i].y()), 1, col, -1);
        } else {
            circle(imageB, Point(p2dB[i].x(),p2dB[i].y()), 5, lblcol, -1);
            circle(imageA, Point(projA.x(), projA.y()), 4, col, -1);
            circle(imageB, Point(p2dB[i].x(),p2dB[i].y()), 4, col, -1);
        }
    }
//    std::cout << "Drew: " << count << " of " << inliers.size() << ", average error: " << sum/count << std::endl;
}

void ImageToLocalization::DrawMatchPoints(cv::Mat& imageA, cv::Mat& imageB, vector<gtsam::Point2>& p0, vector<gtsam::Point2>& p1, std::vector<unsigned char>& inliers) {
    for(int i=0; i<p0.size(); i++){
        if(!_cam.InsideImage(p0[i])) continue;
        if(!_cam.InsideImage(p1[i])) continue;

        CvScalar col = GetLandmarkColor(i);
        if(inliers[i]) circle(imageA, Point(p0[i].x(),p0[i].y()), 5, CV_RGB(0,255,0), -1);
        else circle(imageA, Point(p0[i].x(),p0[i].y()), 5, CV_RGB(255,0,0), -1);
        circle(imageA, Point(p0[i].x(),p0[i].y()), 4, col, -1);
        circle(imageB, Point(p1[i].x(),p1[i].y()), 4, col, -1);
    }
}

/*
 * Survey 1, pose2, and image2.
 * ret._height is zero if a good alignment couldn't be found.
 * */
AlignmentResult ImageToLocalization::MatchToSet(std::string image1, std::string image2) {
    SFlowDREAM2RF sf(_cam);
    if(hasRF) {
        if(!sf.SetReprojectionFlow(rf)) {
            std::cout <<"ImageToLocalization::MatchToSet() couldn't set reprojection flow" << std::endl;
            exit(-1);
        }
    }
    else sf.SetVerifyAlignment();
//    sf.ContinueOnConsistency(); //images align well if the verification threshold passed, apparently. The consistency can be low yet have many points in the image that align well. As long as points are only taken from the consistent set (which they are, in MapPoints()).
    sf.SetEpipolar();
    sf.SetTwoCycleConsistency();
    sf.ConstructImagePyramid(image1, image2);
    sf.AlignImages();
    AlignmentResult ar = sf.GetAlignmentResult();

    //this consistency check can be integrated into SFlowDREAM for streamlined code, but leave it for now.
    // && ar.consistency > ALIGN_CONSISTENCY_THRESHOLD && ar.verified_ratio > ALIGN_VERIFICATION_RATIO //these don't appear to be needed.
    //std::cout << "ImageToLocalization::MatchToSet() (Check for > thresholds). consistency: " << ar.consistency << ", verification ratio: " << ar.verified_ratio << std::endl;
    //discard the result if alignment verification failed.
    if(sf.term_layer <= 1) return ar;

    AlignmentResult ret;
    ret.consistency = ar.consistency;
    ret.alignment_energy_lowres = ar.alignment_energy_lowres;
    ret.verified_ratio = ar.verified_ratio;
    return ret;
}

int ImageToLocalization::MapPoints(AlignmentResult& ar, vector<gtsam::Point2>& imagecoord, vector<int> ids, vector<gtsam::Point3>& p3d,
                                    vector<gtsam::Point2>& p1, vector<gtsam::Point2>& p2, vector<gtsam::Point3>& subset3d, vector<int>& subsetids, bool forward){
    int imset = forward?0:1;
    int count = 0;
    int countbad3d=0, countinconsistent=0, countoutsideimage=0;
    for(int i=0; i<imagecoord.size(); i++){
        if(p3d[i].x()==0 && p3d[i].y()==0 && p3d[i].z()==0) {countbad3d++; continue;}

        //use the alignment result to map the KLT points to the other image.
        gtsam::Point2 p = ar.MapPoint(_cam, imagecoord[i], forward);
        if(p.x()==-1) { countoutsideimage++; continue; }

        //skip the forward/backward inconsistent set found by image alignment
        if(!ar.IsPointConsistent(_cam, imagecoord[i], imset)) { countinconsistent++;}// continue;}

        //create a subset of points to estimate the fundamental matrix from
        p1.push_back(imagecoord[i]);
        p2.push_back(p);
        subset3d.push_back(p3d[i]);
        subsetids.push_back(ids[i]);
        count++;
    }
    if(debug) std::cout << "MapPoints: " << count << " of " << p3d.size() <<" ("<<countbad3d<<" bad 3D, "<<countoutsideimage<< " outside image, " <<countinconsistent<<" inconsistent)"<<std::endl;
    return count;
}

bool ImageToLocalization::ApplyEpipolarConstraints(vector<gtsam::Point2>& p0, vector<gtsam::Point2>& p1, std::vector<unsigned char>& inliers){
    FeatureMatchElimination fme;
    fme.debug = true;
    vector<cv::Point2f> cvp0(p0.size());
    vector<cv::Point2f> cvp1(p1.size());
    for(int i=0; i<p0.size();i++) {
        cvp0[i] = cv::Point2f(p0[i].x(), p0[i].y());
        cvp1[i] = cv::Point2f(p1[i].x(), p1[i].y());
    }
    int ninliers = fme.IdentifyInliersAndOutliers(_cam, cvp0, cvp1, inliers);
    bool mean = fme.AreInliersMeaningful(ninliers);
    if(debug) std::cout << "Are inliers meaningful? " << ninliers << " of " << inliers.size() << ". fme says: " << mean << std::endl;
    return mean;
}

double ImageToLocalization::RobustAlignmentConstraints(AlignmentResult& ar, ParseFeatureTrackFile& pftf0,  ParseFeatureTrackFile& pftf1){
    //Localize the pose offset and identify the set of valid inter-survey constraints.
    lpd = LocalizedPoseData(dates[0], dates[1], portimes[0], portimes[1]); //sids[0], sids[1], 
    lpd.p3d = por[0]->GetSubsetOf3DPoints(pftf0.ids);
    lpd.b3d = por[1]->GetSubsetOf3DPoints(pftf1.ids);
    if(lpd.p3d.size()==0 || lpd.b3d.size()==0) return -1;
//    if(debug) std::cout << "3D set sizes: " << lpd.p3d.size() << ", " << lpd.b3d.size() << std::endl;

    vector<gtsam::Point2> p0;
    vector<gtsam::Point2> p1;
    vector<gtsam::Point3> subset3d;
    vector<int> subsetids;
    int count1 = MapPoints(ar, pftf0.imagecoord, pftf0.ids, lpd.p3d, p0, p1, subset3d, subsetids, true);
    int count2 = MapPoints(ar, pftf1.imagecoord, pftf1.ids, lpd.b3d, p1, p0, subset3d, subsetids, false);

    //Failure here is typically due to a lack of alignment consistency.
    // An image can have high consistency %, yet few consistent points because the %
    //is calculated with a tolerance=1.0. Consistent points match with tolerance=0.
    if(debug) std::cout<<"Number of mapped points check: "<< p0.size() << ", count1: "<<count1 << ",  count2: " << count2 << std::endl;
    if(count1+count2 > 7){
        std::vector<unsigned char> inliers(p0.size(), 1);
        if(ApplyEpipolarConstraints(p0, p1, inliers)){ //true){ //
            lpd.SetPoints(inliers, p1, subset3d, subsetids, 0, count1);
            lpd.SetPoints(inliers, p0, subset3d, subsetids, count1, count1+count2);

            //Expectation Maximization with Bundle Adjustment to get the localized pose and identify inliers.
            LocalizePose lp(_cam);
//            lp.debug = true;
            
            std::vector<std::vector<double> > candidates = lp.RobustDualBA(por[0]->boat[portimes[0]], por[1]->boat[portimes[1]],
                                                                             lpd.p3d0, lpd.p2d1, lpd.rerrorp, lpd.b3d1, lpd.b2d0, lpd.rerrorb);
            if(candidates.size()==0) return -1;
            
//            perc_dc = candidates[2][1]/(count1+count2); //old. the percentage is of the points before applying the epipolar constraint. the effect is to make the threshold ... the
            perc_dc = candidates[2][1]/(lpd.p3d0.size()+lpd.b3d1.size()); //percentage is based on the number of points used for localization.
            if(debug) std::cout << dates[0] << ": localization quality check: \n\tForward and Backward:  "
                    <<((int)1000*perc_dc)/10.0 << "% inliers with "
                    << candidates[2][3] << " avg reprojection error" << std::endl;
            
            if(perc_dc > PERCENT_DENSE_CORRESPONDENCES) {
                lpd.SetPoses(por[0]->boat[portimes[0]], candidates[0], candidates[1]);
                lpd.SetLocalizationQuality(perc_dc, candidates[2][3]);
                
                //printf("pose %d from vo (%lf,%lf,%lf,%lf,%lf,%lf)\n",i,candidates[0][0],vp[1],vp[2],vp[3],vp[4],vp[5]);
                
                //save the alignment result (to debug or evaluate RF)
                /*
                DrawFlowPoints(ar.ref, ar.im2, lpd.rerrorp, lpd.p3d0, lpd.p2d1, por[0]->CameraPose(portimes[0]), CameraPose(candidates[0]));
                std::string dir = "/Users/shane/Documents/research/experiments/2018/AE/" + to_string(portimes[1]) + "/";
                FileParsing::MakeDir(dir);
                dir = dir + dates[0] + "/";
                FileParsing::MakeDir(dir);
                ar.Save(dir);*/
                
//                DrawFlowPoints(ar.ref, ar.im2, lpd.rerrorp, lpd.p3d0, lpd.p2d1, por[0]->CameraPose(portimes[0]), CameraPose(candidates[0]));
//                string debugdir = _results_dir + dates[1] + "/debug/";
//                FileParsing::MakeDir(debugdir);
//                ar.Save(debugdir + to_string(portimes[1]) + "/");

                //Verification constraint.
                if(toverify->IsSet()) {
                    gtsam::Pose3 p1_t = por[1]->CameraPose(lpd.s1time);
                    ver_result = toverify->VerifyWith(_cam, lpd, p1_t, p1_tm1);
                }
            }
        }
    }

    return -1;
}

void ImageToLocalization::Setup(LocalizedPoseData * res, double * perc_dc, double * verified){
    thread_state = state::LOCKED;
    _res = res;
    _perc_dc = perc_dc;
    _verified = verified;
}

void ImageToLocalization::Reset(){
    thread_state = state::OPEN;

    poses = {};
    rf = {};
    por = {};
    dates = {};
    sids = {};
    portimes = {};
    hasRF = false;

    toverify = NULL;
    _res = NULL;
    p1_tm1 = gtsam::Pose3();
    ver_result = -1.0;
    perc_dc = -1;
    _perc_dc = NULL;
    _verified = NULL;
    lpd = LocalizedPoseData();
}

void * ImageToLocalization::Run(){
    thread_state = state::RUNNING;
    ParseFeatureTrackFile pftf0 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[0], por[0]->ftfilenos[portimes[0]]);
    ParseFeatureTrackFile pftf1 = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + dates[1], por[1]->ftfilenos[portimes[1]]);

    if(hasRF) {
        rf[0]->ComputeFlow(poses[1], por[0]->boat[portimes[0]]);//map points of survey 0 onto pose1_est.
        rf[1]->ComputeFlow(poses[0], por[1]->boat[portimes[1]]);//map points of survey 1 onto pose0_est.
        rf[0]->CreateRestrictedSet(stoi(dates[0]), pftf0);
        rf[1]->CreateRestrictedSet(stoi(dates[1]), pftf1);
    }

    string image0 = ParseSurvey::GetImagePath(_query_loc + dates[0], por[0]->cimage[portimes[0]]);
    string image1 = ParseSurvey::GetImagePath(_query_loc + dates[1], por[1]->cimage[portimes[1]]);
    AlignmentResult ar = MatchToSet(image0, image1);

    //if(debug) std::cout<<"alignment success check: "<<ar._height<<std::endl;
    if(ar._height > 0) RobustAlignmentConstraints(ar, pftf0, pftf1);

    thread_state = state::FINISHED;
    return (void *) NULL;
}

void ImageToLocalization::LogResults(){
    *_res = lpd;
    *_perc_dc = perc_dc;
    *_verified = ver_result;
}
