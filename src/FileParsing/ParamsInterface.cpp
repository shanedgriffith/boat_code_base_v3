//
//  ParamsInterface.cpp
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/10/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "ParamsInterface.h"

#include <fstream>
#include <iostream>


using namespace std;

const string ParamsInterface::parmsfile = "/parms.csv";

void ParamsInterface::ProcessLineEntries(int type, vector<string> lp){
    if(lp.size() != 2) {cout <<"ParamsInterface Error: The param file is incorrectly formatted."<<endl; exit(-1);}
    keys.push_back(lp[0]);
    values.push_back(stod(lp[1]));
}

void ParamsInterface::ReadDelimitedFile(string file, int type) {
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

void ParamsInterface::AddParam(string key, double value) {
    bool haskey = false;
    for(int i=0; i<keys.size(); i++) {
        if(keys[i].compare(key)==0){
            haskey = true;
            break;
        }
    }
    
    if(!haskey) {
        keys.push_back(key);
        values.push_back(value);
    }
}

void ParamsInterface::AddParams(vector<string> keys, vector<double> values) {
    for(int i=0; i<keys.size(); i++) {
        AddParam(keys[i], values[i]);
    }
}

vector<double> ParamsInterface::LoadParams(vector<string> wanted_keys, vector<double> defaults) {
    vector<double> vals;
    for(int i=0; i<wanted_keys.size(); i++) {
        bool haskey = false;
        for(int j=0; j<keys.size(); j++) {
            if(keys[j].compare(wanted_keys[i])==0) {
                vals.push_back(values[j]);
                haskey = true;
                break;
            }
        }
        if(verbose){
            if(haskey) cout << "" << wanted_keys[i] << " = "<<vals[i]<<endl;
            else cout << "" << wanted_keys[i] << " NOT IN THE PARAMETER LIST "<<endl;
        }
        
        if(!haskey) {
            if(defaults.size()==0) {
                cout << "ParamsInterface Error: No such key exists. " << wanted_keys[i] << endl;
                exit(-1);
            }
            else {
                vals.push_back(defaults[i]);
                if(verbose) cout << "defaults[i]: " << defaults[i] << endl;
            }
        }
    }
    return vals;
}

void ParamsInterface::SaveParams(string loc) {
    string fileloc = loc + parmsfile;
    FILE * fp = OpenFile(fileloc, "w");
    
    for(int i=0; i<keys.size(); i++) {
        fprintf(fp, "%s, %lf\n", keys[i].c_str(), values[i]);
    }
    fclose(fp);
}

void ParamsInterface::LoadParams(std::string loc){
    _paramfile = loc + parmsfile;
    if(Exists(_paramfile)) {
        ReadDelimitedFile(_paramfile, 0);
    } else {
        std::cout << "ParamsInterface::LoadParams() Couldn't get the parameters at: " << _paramfile << std::endl;
        exit(-1);
    }
}
