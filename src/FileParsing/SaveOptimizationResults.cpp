//
//  SaveOptimizationResults.cpp
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/10/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "SaveOptimizationResults.h"
#include "Visualizations/SLAMDraw.h"


using namespace std;

void SaveOptimizationResults::SaveVisualization(vector<vector<double> >& landmarks, vector<vector<double> >& traj, vector<double> drawscale) {
	if(traj.size() == 0 || landmarks.size() == 0){
		cout << "SaveOptimizationResults::SaveVisualization() Error. Vector sizes are zero, which means optimization didn't run."<<endl;
		exit(-1);
	}
    SLAMDraw draw;
    if(drawscale.size()>0) draw.SetScale(drawscale[0], drawscale[1], drawscale[2], drawscale[3]);
    else draw.SetScale(-300,300,-300,300);
    draw.ResetCanvas();
    
    //draw the estimated landmark points
    for(int i=0; i<landmarks.size(); i++) {
        if(filter_null_points) if(landmarks[i][0] == 0 && landmarks[i][1] == 0 && landmarks[i][2] == 0) continue;
        draw.AddPointLandmark(landmarks[i][0], landmarks[i][1], landmarks[i][3]);
    }
    
    //draw the estimated trajectory
    for(int i=0; i<traj.size(); i++) {
        draw.AddPointPath(traj[i][0], traj[i][1]);
    }
    
    //draw the boat's field of view
    vector<double> lastpose = traj[traj.size()-1];
    draw.DrawSight(lastpose[0], lastpose[1], lastpose[5]);
    
    //save the visualization
    string visualization = _base + drawdir + to_string(traj.size()) + ".jpg";
    draw.SaveDrawing(visualization);
    
    if(debug) cout << "Optimization visualization saved to " << visualization << ". " << endl;
}

void SaveOptimizationResults::SaveLandmarks(vector<vector<double> >& landmarks){
    if(landmarks.size()==0) return;
    FILE * points_f = OpenFile(_base + pointsfile, "w");
    for(int i=0; i<landmarks.size(); i++) {
        if(filter_null_points) if(landmarks[i][0] == 0 && landmarks[i][1] == 0 && landmarks[i][2] == 0) continue;
        fprintf(points_f, "%d, %lf, %lf, %lf\n", (int)landmarks[i][3], landmarks[i][0], landmarks[i][1], landmarks[i][2]);
    }
    fclose(points_f);
}

void SaveOptimizationResults::SavePoses(string file, vector<vector<double> >& poses){
    if(poses.size()==0) return;
    FILE * boat_f = OpenFile(file, "w");
    for(int i=0; i<poses.size(); i++){
        fprintf(boat_f, "%d, %lf, %lf, %lf, %lf, %lf, %lf\n", (int)poses[i][6], poses[i][0], poses[i][1], poses[i][2], poses[i][3], poses[i][4], poses[i][5]);
    }
    fclose(boat_f);
}

void SaveOptimizationResults::PlotAndSaveCurrentEstimate(vector<vector<double> >& landmarks, vector<vector<double> >& traj, vector<vector<double> >& vels, std::vector<double> drawscale){
    if(landmarks.size()>0) SaveLandmarks(landmarks);
    if(traj.size()>0) SavePoses(_base + boatfile, traj);
    if(vels.size()>0) SavePoses(_base + velocityfile, vels);
    if(draw_map) SaveVisualization(landmarks, traj, drawscale);
}

void SaveOptimizationResults::SaveDataCorrespondence(int camera_key, int sift_file_no, int aux_file_idx, int imageno, double timestamp){
    /*
     save the data alignment.
     added time to eliminate indexing ambiguity.
     */
    FILE * bts;
    if(!started_correspondence) {
        bts = OpenFile(_base + correspondencefile, "w");
        started_correspondence=true;
    } else {
        bts = OpenFile(_base + correspondencefile, "a");
    }
    
    //these correspond to: camera variable of gtsam, the sift file number, the aux file index, the image file number, and the time.
    fprintf(bts, "%d,%d,%d,%d,%lf\n", camera_key, sift_file_no, aux_file_idx, imageno, timestamp);
    fflush(bts);
    fclose(bts);
}

void SaveOptimizationResults::StatusMessage(int iteration, double percent_completed){
    if(!save_status) return;
    
    FILE * fp = OpenFile(_base + statusfile, "a");
    time (&end);
    double dif = difftime (end,start);
    char buf[LINESIZE];
    sprintf(buf, "ITERATION %d. Percent complete %lf. Total run time (HH:MM:SS) %s\n", iteration, percent_completed, formattime(dif).c_str());
    fprintf(fp, "%s", buf);
    printf("%s", buf);
    fclose(fp);
}
