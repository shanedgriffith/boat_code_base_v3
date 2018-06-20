//
//  ReprojectionFlow.cpp
//  SIFTFlow
//
//  Created by Shane Griffith on 2/21/16.
//  Copyright © 2016 shane. All rights reserved.
//

#include "math.h"
#include "ReprojectionFlow.hpp"
#include "ImageAlignment/DREAMFlow/FeatureMatchElimination.hpp"


using namespace std;
using namespace cv;

bool ReprojectionFlow::DistanceCriterion(vector<double>& pose1, vector<double>& pose2){
    double dist = pow(pow(pose1[0] - pose2[0], 2) + pow(pose1[1] - pose2[1],2), 0.5);
    if (dist > pose_distance_threshold) {
        // Don't try to match points more than 20m away, GPS is not
        // that bad.
        return false;
    }
    if (fabs(remainder(pose1[5]-pose2[5],2*M_PI)) > pose_angle_threshold*M_PI/180) {
        // If we're not looking remotely in the same direction, no
        // point trying to match
        return false;
    }
    return true;
}

double ReprojectionFlow::GStatistic(vector<int>& ct){
    //Maximize the g-statistic for dependency.
    
    if(ct.size() != 4){
        cout << "ReprojectionFlow::GStatistic() Error: Incorrect input to G-Statistic()" << endl;
        exit(-1);
    }
    
    double tot = ct[0] + ct[1] + ct[2] + ct[3];
    
    double sum = 0.0;
    for(int i=0; i<2; i++){
        for(int j=0; j<2; j++){
            int idx = i*2 + j;
            if(ct[idx]==0) continue;
            
            //use casts to avoid overflow.
            double denom = ((double) ct[i*2]+ct[i*2+1])*((double) ct[j]+ct[2+j]);
            if(denom==0) return 0;
            
            sum += ct[idx] * log(1.0*tot*ct[idx]/denom);
        }
    }
    
    return 2*sum;
}

vector<int> ReprojectionFlow::ExtractContingencyTable(vector<bool>& vi1, vector<bool>& vi2){
    if(vi1.size() != vi2.size()){
        cout << "ReprojectionFlow::ExtractContingencyTable() Error: Incorrect input to ExtractContingencyTable()." << endl;
        exit(-1);
    }
    //[0,0], [0,1], [1,0], [1,1]
    //0, 1, 2, 3
    vector<int> contingency_table = {0,0,0,0};
    for(int i=0; i<vi1.size(); i++){
        int idx = vi1[i]*2 + vi2[i];
        contingency_table[idx]++;
    }
    
    return contingency_table;
}

void ReprojectionFlow::TestGstat(){
//    vector<int> contingency_table1 = {2,45,0,55};
//    vector<int> contingency_table2 = {36,11,0,55};
//    vector<int> contingency_table3 = {20,27,26,29};
    vector<int> contingency_table1 = {3,46,0,56};
    vector<int> contingency_table2 = {38,11,0,56};
    vector<int> contingency_table3 = {21,28,26,30};
    
    double one = GStatistic(contingency_table1);
    double two = GStatistic(contingency_table2);
    double three = GStatistic(contingency_table3);
    cout << "results: "<< one << ", "<<two<<", "<<three<<endl;
}

double ReprojectionFlow::GStatisticForPose(vector<double>& camA, vector<double>& camB){
	double g=0;
	vector<vector<double> > _camA = {camA};
	IdentifyClosestPose(_camA, camB, &g);
	return g;
}

vector<gtsam::Point2> ReprojectionFlow::ProjectPoints(vector<double>& boat, vector<bool>& valid_indices){
    if(boat.size()<6){
        std::cout << "ReprojectionFlow::ProjectPoints() expected a pose vector. Check the code." <<std::endl;
        exit(-1);
    }
    gtsam::Pose3 tf(gtsam::Rot3::RzRyRx(boat[3],boat[4],boat[5]), gtsam::Point3(boat[0],boat[1],boat[2]));
    vector<gtsam::Point2> validset(_map.map.size(), gtsam::Point2(-1, -1));
    for(int j=0; j<_map.map.size(); j++) {
        if(_map.map[j].x()==0.0 && _map.map[j].y()==0.0 && _map.map[j].z()==0.0) continue;
        gtsam::Point3 res = tf.transform_to(_map.map[j]);
		validset[j] = _cam.ProjectToImage(res);
		valid_indices[j] = _cam.InsideImage(validset[j]);
    }
    return validset;
}

void ReprojectionFlow::Reset(){
	viewset.clear();
	restrictedset.clear();
}

void ReprojectionFlow::SparseFlow(vector<bool>& iA, vector<bool>& iB, vector<gtsam::Point2>& rpA, vector<gtsam::Point2>& rpB){
	Reset();
    for(int i=0; i<iA.size(); i++){
    	if(!iA[i] || !iB[i]) continue;
        gtsam::Point2 pointflow(rpA[i].x()-rpB[i].x(), rpA[i].y()-rpB[i].y());
        rfpoint rfp = {rpB[i], rpA[i], pointflow, i, _map.variances[i]};
		viewset.push_back(rfp);
    }
}

void ReprojectionFlow::ComputeFlow(vector<double>& camA, vector<double>& camB){
    vector<bool> valid_indicesA(_map.map.size(), false);
    vector<gtsam::Point2> rpA = ProjectPoints(camA, valid_indicesA);
    
    vector<bool> valid_indicesB(_map.map.size(), false);
    vector<gtsam::Point2> rpB = ProjectPoints(camB, valid_indicesB);
    
    SparseFlow(valid_indicesA, valid_indicesB, rpA, rpB);
}

int ReprojectionFlow::IdentifyClosestPose(vector<vector<double> >& camA, vector<double>& camB, double * gresult, bool save){
    //Identify the closest pose using the G-statistic.

    vector<bool> valid_indicesB(_map.map.size(), false);
    vector<gtsam::Point2> reprojected_pointsB = ProjectPoints(camB, valid_indicesB);

    //NOTE: it may not be possible to save the data in the for loop using pointers.
    vector<bool> valid_indicesA(_map.map.size(), false);
    vector<gtsam::Point2> reprojected_pointsA;

    int closest = -1;
    double maxg = -1;
    for(int i=0; i<camA.size(); i++){
        if(!DistanceCriterion(camA[i], camB)) continue;

        vector<bool> viA(_map.map.size(), false);
        vector<gtsam::Point2> rpA = ProjectPoints(camA[i], viA);

		vector<int> ct = ExtractContingencyTable(viA, valid_indicesB);

		double gstatistic = GStatistic(ct);

        if(gstatistic > maxg){
            maxg = gstatistic;
            closest = i;
            if(save){
                valid_indicesA.assign(viA.begin(), viA.end());
                reprojected_pointsA.assign(rpA.begin(), rpA.end());
            }
        }

		if(debug){
			cout << "    ["<<ct[0] << " | " << ct[1] << "]" << endl;
			cout << "    [  "<<ct[2] << "  |  " << ct[3] << " ]" << endl;
			cout << "  G: " << gstatistic << endl;
			cout << endl;
		}
    }

    if(debug){
        cout << "   " << closest<< " Closest pose." <<endl;
        cout << "   " << maxg << " G-statistic." << endl;
    }

    if(save && closest >= 0){
    	SparseFlow(valid_indicesA, valid_indicesB, reprojected_pointsA, reprojected_pointsB);
    }

    if(gresult!=NULL) *gresult = maxg;
    return closest;
}

void ReprojectionFlow::CreateRestrictedSet(int survey, ParseFeatureTrackFile& pft, bool forward){
	//Find the subset of points that match those of the pft file. This limits
	//reprojection error to a single direction.
	//This function assumes both pft and the map points were relatively ordered according to their keys.

	int indice = 0;
    int count=0;
	vector<rfpoint> rset;
	for(int  i=0; i<viewset.size() && indice < pft.ids.size(); i++){
		//the survey check is important because different surveys may use the same keys.
		int surveyno = _map.survey_labels[viewset[i].map_idx];
		if(survey != surveyno) continue;

		//the points as projected onto the pose from the other survey
		gtsam::Point2 p = forward ? viewset[i].pim1 : viewset[i].pim2;
		gtsam::Point2 m = forward ? viewset[i].pim2 : viewset[i].pim1;

		bool found = false;
		int l = _map.landmark_ids[viewset[i].map_idx];
		for(int j=0; j<pft.ids.size(); j++){
			//note: the use of index j is required in case optimization trashes that point.
			if(l == pft.ids[j]){
				found = true;
				indice = j;
				break;
			}
		}

		if(found){
			//Using the original point coordinate limits correspondence uncertainty to one point.
			gtsam::Point2 pflow = m - pft.imagecoord[indice];
			double dist = pft.imagecoord[indice].distance(p);
			rfpoint rfp = {pft.imagecoord[indice], m, pflow, viewset[i].map_idx, dist};
			rset.push_back(rfp);
            count++;
		}
	}

	restrictedset.push_back(rset);
	if(rset.size() == 0) cout << "ReprojectionFlow::CreateRestrictedSet() WARNING. the restricted set has size zero. check that the correct survey range is specified. survey: " << survey <<endl;
    OutputSize(_cam.w(), _cam.h());
    EliminateOutliers(restrictedset.size()-1);
    if(debug) std::cout<<"num found: " << count <<", after removing outliers: " << restrictedset[restrictedset.size()-1].size()<<std::endl;
}

void ReprojectionFlow::CreateMirroredRestrictedSet(int res_set, int src_set){
	restrictedset[res_set].clear();
	for(int  i=0; i<restrictedset[src_set].size(); i++){
		//Using the original point coordinate limits correspondence uncertainty to one point.
		rfpoint rfp = {restrictedset[src_set][i].pim2,
				restrictedset[src_set][i].pim1,
				-restrictedset[src_set][i].pflow,
				restrictedset[src_set][i].map_idx,
				restrictedset[src_set][i].rerror};
		restrictedset[res_set].push_back(rfp);
	}
}

bool ReprojectionFlow::HaveTwoSets(){
	//creates a mirrored set if needed, or returns false if neither one can be used.
	while(restrictedset.size()<2) restrictedset.push_back(vector<rfpoint>());

	bool havegoodset = false;
	vector<bool> needmirror(2, true);
	for(int i=0; i<restrictedset.size(); i++){
		needmirror[i] = restrictedset[i].size() == 0;
		havegoodset |= !needmirror[i];
	}

	if(!havegoodset){
		restrictedset = {};
		return false;
	}

	for(int i=0; i<needmirror.size(); i++){
		if(needmirror[i]){
			CreateMirroredRestrictedSet(i,(i+1)%2);
		}
	}
	return true;
}

void ReprojectionFlow::EliminateOutliers(int active_set){
	//Remove outliers according to epipolar constraints.
	FeatureMatchElimination fme;

	//find the points that match the epipolar constraint.
	vector<cv::Point2f> orig_sparse = GetRestrictedSetOrig(active_set);
	vector<cv::Point2f> mapped_sparse = GetRestrictedSetMapped(active_set);
    vector<unsigned char> labels;
    int ninliers = fme.IdentifyInliersAndOutliers(_cam, orig_sparse, mapped_sparse, labels);
    if(!fme.AreInliersMeaningful(ninliers)) restrictedset[active_set].clear();
    else{
        int count_rem = 0;
        for(int i=0; i<labels.size(); i++){
            if(labels[i]==0){
                restrictedset[active_set].erase(restrictedset[active_set].begin() + i - count_rem);
                count_rem++;
            }
        }
    }
    if(debug) std::cout<<"num remaining after: " <<restrictedset[active_set].size()<<std::endl;
}

vector<cv::Point2f> ReprojectionFlow::GetRestrictedSetOrig(int active_set){
	vector<cv::Point2f> orig(restrictedset[active_set].size(), cv::Point2f());
	for(int i=0; i<restrictedset[active_set].size(); i++){
		orig[i] = Scale(Point2f(restrictedset[active_set][i].pim1.x(), restrictedset[active_set][i].pim1.y()), false);
	}
	return orig;
}

vector<cv::Point2f> ReprojectionFlow::GetRestrictedSetMapped(int active_set){
	vector<cv::Point2f> mapped(restrictedset[active_set].size(), cv::Point2f());
	for(int i=0; i<restrictedset[active_set].size(); i++){
		mapped[i] = Scale(Point2f(restrictedset[active_set][i].pim2.x(), restrictedset[active_set][i].pim2.y()), false);
	}
	return mapped;
}

int ReprojectionFlow::GetNumberOfConstraints(int active_set){
	if(active_set>=0) return restrictedset[active_set].size();
    return viewset.size();
}

vector<double> ReprojectionFlow::GetConstraint(int active_set, int i){
	rfpoint rfp = viewset[i];
	if(active_set>=0) rfp = restrictedset[active_set][i];

    Point2f pixel_scaled = Scale(Point2f(rfp.pim1.x(), rfp.pim1.y()), false);
    Point2f flow_scaled = Scale(Point2f(rfp.pflow.x(), rfp.pflow.y()), false);
    vector<double> ret = {pixel_scaled.x, pixel_scaled.y, flow_scaled.x, flow_scaled.y, rfp.rerror};
    return ret;
}

vector<double> ReprojectionFlow::GetAverageConstraint(int active_set){
    double sumx=0, sumy=0;
    int n = GetNumberOfConstraints(active_set);
    if(n==0) return {};
    for(int i=0; i<n; i++){
        vector<double> c = GetConstraint(active_set, i);
        sumx += c[2];
        sumy += c[3];
    }
    return {sumx/n, sumy/n};
}

vector<double> ReprojectionFlow::GetMaxConstraint(int active_set){
    double maxx=0, maxy=0;
    int n = GetNumberOfConstraints(active_set);
    if(n==0) return {};
    for(int i=0; i<n; i++){
        vector<double> c = GetConstraint(active_set, i);
        maxx = (abs(c[2])>maxx)?abs(c[2]):maxx;
        maxy = (abs(c[3])>maxy)?abs(c[3]):maxy;
    }
    return {maxx, maxy};
}

vector<double> ReprojectionFlow::GetConstraintBounds(int active_set){
    double maxx=-100000000, maxy=-100000000;
    double minx=100000000, miny=100000000;
    double sumx=0, sumy=0;
    int n = GetNumberOfConstraints(active_set);
    if(n==0) return {0, 0, 0, 0, 0, 0};
    for(int i=0; i<n; i++){
        vector<double> c = GetConstraint(active_set, i);
        maxx = (c[2]>maxx)?c[2]:maxx;
        maxy = (c[3]>maxy)?c[3]:maxy;
        minx = (c[2]<minx)?c[2]:minx;
        miny = (c[3]<miny)?c[3]:miny;
        sumx += c[2];
        sumy += c[3];
    }
    double avgx = (sumx==0)?0:sumx/n;
    double avgy = (sumy==0)?0:sumy/n;
    return {minx, maxx, avgx, miny, maxy, avgy};
}

std::vector<double> ReprojectionFlow::GetkNNConstraint(int active_set, int x, int y, int k){
    //brute force to test. if it's a constraint, them improve.
    //k=1 for now.
    std::vector<ReprojectionFlow::rfpoint>* csearch = &viewset;
    if(active_set>=0) csearch = &restrictedset[active_set];
    
    int minidx = -1;
    double mindist = 100000;
    for(int i=0; i<csearch->size(); i++){
        ReprojectionFlow::rfpoint rfp = (*csearch)[i];
        Point2f pixel_scaled = Scale(Point2f(rfp.pim1.x(), rfp.pim1.y()), false);
        double dist = pow(pixel_scaled.x-x,2.0) + pow(pixel_scaled.y-y,2.0);
        if(dist < mindist){
            mindist = dist;
            minidx = i;
        }
    }
    if(minidx==-1) return {};
    
    rfpoint rfp = (*csearch)[minidx];
    Point2f flow_scaled = Scale(Point2f(rfp.pflow.x(), rfp.pflow.y()), false);
    return {flow_scaled.x, flow_scaled.y};
}

Point2f ReprojectionFlow::Scale(Point2f p, bool up){
	if(outw==0){
		std::cout << "Error ReprojectionFlow Scale(): Set the OutputSize()."<<std::endl;
		exit(-1);
	}
    double xscale=0, yscale=0;
    
    if(up){
        xscale = ((double)_cam.w())/outw;
        yscale = ((double)_cam.h())/outh;
    }
    else{
        xscale = ((double)outw)/_cam.w();
        yscale = ((double)outh)/_cam.h();
    }
    
    return Point2f(p.x*xscale, p.y*yscale);
}

vector<double> ReprojectionFlow::MeasureFlowAgreement(cv::Mat& flow, int active_set){
    /*Measure the average distance between the flow result and the projected flow.*/
	//Useful as a sanity check and for comparison with other dense correspondence methods.
    
    double * data = (double *) flow.data;
    
    vector<double> res = {0,0};
    for(int i=0; i<restrictedset[active_set].size(); i++){
        gtsam::Point2 originpoint = restrictedset[active_set][i].pim1;
        gtsam::Point2 f = restrictedset[active_set][i].pflow;
        
        int index = (((int)originpoint.y())*_cam.w() + ((int)originpoint.x()))*2;
        gtsam::Point2 imflow(data[index], data[index+1]);
        double dist = f.distance(imflow);
        
        res[0] += dist;
        res[1]++;
        if(debug) cout << "Deviation from "<<restrictedset[active_set][i].map_idx<<": " << dist << endl;
    }
    
    return res;
}

vector<double> ReprojectionFlow::MeasureDeviationsPerSurvey(cv::Mat &flow){
//    cout <<"Large deviations from points from one survey but not others may indicate a change"<<endl;
//    cout <<"Large deviations in general may indicate a poor alignment."<<endl;
    
    vector<double> res((_map.num_surveys+1)*2, 0);

    int lastsurvey = -1;
    int sidx = -1;
    double * data = (double *) flow.data;
    for(int i=0; i<viewset.size(); i++){
        gtsam::Point2 originpoint = viewset[i].pim1;
        gtsam::Point2 f = viewset[i].pflow;
        int surveyno = _map.survey_labels[viewset[i].map_idx];
        if(surveyno != lastsurvey) {
        	lastsurvey = surveyno;
        	sidx++;
        }
        
        int index = (((int)originpoint.y())*_cam.w() + ((int)originpoint.x()))*2;
        gtsam::Point2 imflow(data[index], data[index+1]);
        double dist = f.distance(imflow);
        
        res[2*_map.num_surveys] += dist;
        res[2*_map.num_surveys+1]++;
        res[2*sidx] += dist;
        res[2*sidx+1]++;
    }
    
    if(debug) cout << "average re: " << res[2*_map.num_surveys]/res[2*_map.num_surveys+1] << endl;
    return res;
}

/******************************** DRAWING RELATED (for visualization and debugging) *******************************/
void ReprojectionFlow::DrawFlowPoints(cv::Mat& image, int active_set){
    IMDraw art(image);
    art.SetPointSize(15);
    for(int i=0; i<restrictedset[active_set].size(); i++)
        art.DrawPoint(restrictedset[active_set][i].pim2.x(),
                restrictedset[active_set][i].pim2.y(),
                restrictedset[active_set][i].map_idx);
}

void ReprojectionFlow::DrawMapPoints(cv::Mat& image, int active_set){
    IMDraw art(image);
    art.SetPointSize(15);
    for(int i=0; i<restrictedset[active_set].size(); i++)
        art.DrawPoint(restrictedset[active_set][i].pim1.x(),
                        restrictedset[active_set][i].pim1.y(),
                        restrictedset[active_set][i].map_idx);
}

void ReprojectionFlow::DrawFlowSurvey(cv::Mat& imageB, int survey){
    if(viewset.size()==0){
        cout << "ReprojectionFlow::DrawFlowSurvey() Error: Get the flow first." << endl;
        exit(-1);
    }

    CvScalar col = CV_RGB((1.0*rand()/RAND_MAX)*255,(1.0*rand()/RAND_MAX)*255,(1.0*rand()/RAND_MAX)*255);
    IMDraw art(imageB);

    for(int i=0; i<viewset.size(); i++){
        int surveyno = _map.survey_labels[viewset[i].map_idx];
        if(survey != surveyno) continue;
        cout << "survey: " << surveyno << ", key: " << _map.landmark_ids[viewset[i].map_idx];
        art.SetColor(col);
        art.DrawArrow(viewset[i].pim1.x(), viewset[i].pim1.y(), viewset[i].pim2.x(), viewset[i].pim2.y());
        art.DrawPoint(viewset[i].pim1.x(), viewset[i].pim1.y());
    }
}

void ReprojectionFlow::DrawFlowSurvey(cv::Mat& imageB, int survey, ParseFeatureTrackFile& pft){
    if(viewset.size()==0){
        cout << "ReprojectionFlow::DrawFlowSurvey() Error: Get the flow first." << endl;
        exit(-1);
    }

    IMDraw art(imageB);

    srand(time(NULL));
    double dist = 0;
    for(int i=0; i<viewset.size(); i++){
        int surveyno = _map.survey_labels[viewset[i].map_idx];
        if(survey != surveyno) continue;
        int l = _map.landmark_ids[viewset[i].map_idx];
        CvScalar col = CV_RGB(l*13%255,l*57%255,l*37%255);

        gtsam::Point2 p = viewset[i].pim1;
		gtsam::Point2 m = viewset[i].pim2;
        for(int j=0; j<pft.ids.size(); j++){
        	if(l == pft.ids[j]){
        		circle(imageB, Point(pft.imagecoord[j].x(),pft.imagecoord[j].y()), 10, CV_RGB(255,255,255), -1);
        		circle(imageB, Point(pft.imagecoord[j].x(),pft.imagecoord[j].y()), 6, col, -1);
        		line(imageB, cv::Point2f(m.x(), m.y()), Point(pft.imagecoord[j].x(),pft.imagecoord[j].y()), CV_RGB(0,0,0));

        		arrowedLine(imageB, cv::Point2f(p.x(), p.y()), cv::Point2f(m.x(), m.y()), col);
				circle(imageB, Point(p.x(),p.y()), 4, col, -1);
				dist += pow(pow(pft.imagecoord[j].x()-p.x(),2)+pow(pft.imagecoord[j].y()-p.y(),2),0.5);
        		break;
        	}
        }

    }
    cout << "dist: " << dist / pft.ids.size() << endl;
}

/*
 * Desired usage:
 *  ReprojectionFlow rf(map);
 *  int poseloc = rf.IdentifyClosestPose(posesA, poseB);
 *  rf.CreateRestrictedSet(1, pftB);
 *  rf.CreateRestrictedSet(0, pftA[poseloc], false); //OR rf.CreateRestrictedSetMirrored(); (this can be left out)
 *  SFlowDREAM sf;
 *  if(!sf.SetReprojectionFlow(rf)) //checks for two sets, and mirrors one if needed.
 *      continue;
 * */

