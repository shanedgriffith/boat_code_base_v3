/*
 * AlignVisibilitySet.cpp
 *
 *  Created on: Feb 28, 2017
 *      Author: shane
 */


#include <ImageAlignment/GeometricFlow/ReprojectionFlow.hpp>
#include <DataTypes/Map.hpp>
#include <FileParsing/ParseVisibilityFile.h>
#include <Visualizations/SLAMDraw.h>

#include "AlignVisibilitySet.hpp"

using namespace std;

void AlignVisibilitySet::Visibility() {
    ParseVisibilityFile vis(_visibility_dir, _date1, _date2);
    
    std::string saveloc = _results_dir + _date1 + "_to_" + _date2 + "/";
    mkdir(saveloc.c_str(), (mode_t) (S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH));
    
    for(int i=0; i<vis.boat1.size(); i++) {
        std::cout << "Iteration: " << i << std::endl;
        
        //spawn a job to handle the alignment.
        int tidx = man.GetOpenMachine();
        ws[tidx]->Setup(vis.boat1[i]);
        ws[tidx]->SetMaps({&maps[0], &maps[1]});
        ws[tidx]->SetDates({_date1, _date2});
        ws[tidx]->SetPOR({&por[0], &por[1]});
        man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    
    std::cout << "Finished aligning the visibility set for " << _date1 << " to " << _date2 <<". Num images: " << vis.boat1.size() << std::endl;
}


std::vector<char> AlignVisibilitySet::LoadLabelsFile(std::string filepath){
    FILE * fp = OpenFile(filepath,"r");
    char line[99]="";
    
    std::vector<char> labels;
    while (!feof(fp) && fgets(line,99,fp)!=NULL) {
        char label;
        double time;
        if (sscanf(line,"%c,%lf", &label,&time)!=2) {
            std::cout << "AlignVisibilitySet::LoadLabelsFile() Read Error of file. " << filepath << ", line: " << line << std::endl;
            exit(-1);
        }
        labels.push_back(label);
    }
    fclose(fp);
    return labels;
}

void AlignVisibilitySet::VisualizeAllLabelsInOneMap(){
    vector<string> dates = {"140117", "140122", "140129", "140205", "140314"};
    
    vector<int> coarse(7000,0);
    vector<int> precise(7000,0);
    
    
    for(int i=0; i<dates.size(); i++){
        string labelfile = _results_dir + _date1 + "_to_" + dates[i] + "/labels.txt";
        std::vector<char> labels = LoadLabelsFile(labelfile);
//        ParseVisibilityFile vis(_visibility_dir, _date1, dates[i]);
        std::vector<string> dirs = FileParsing::ListDirsInDir(_results_dir + _date1 + "_to_" + dates[i]);
        if(dirs.size() != labels.size()){
            std::cout << "AlignVisibilitySet::VisualizeAllLabelsInOneMap() something went wrong with the labels. Mismatch with the alignment set. " << labels.size() << " labels, " << dirs.size() << " vis set."<<std::endl;
            exit(-1);
        }
        for(int j=0; j<dirs.size(); j++){
            int idx = stoi(dirs[j]);
            if(labels[j]=='g') precise[idx]++;
            else coarse[idx]++;
        }
    }
    
    std::cout << "using poses of file: "<< por[0]._base << std::endl;
    
    SLAMDraw draw;
    draw.SetScale(-300,300,-300,300);
    draw.ResetCanvas();
    for(int i=0; i<coarse.size(); i++){
        if(coarse[i] > 0 || precise[i] > 0){
            double ratio = 1.0*precise[i]/(precise[i]+coarse[i]);
            double col = ratio * 255;
            draw.AddPointPath(por[0].boat[i][0], por[0].boat[i][1], col, col, col);
        }
    }
    
    string resfile = _results_dir + "timelapse_quality_map_.png";
    draw.SaveDrawing(resfile);
}







