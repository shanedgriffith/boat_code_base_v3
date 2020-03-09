//
//  ParamsInterface.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/10/15.
//  Copyright (c) 2015 shane. All rights reserved.
//
#pragma once


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include "FileParsing.hpp"

class ParamsInterface: public FileParsing{
private:
    static const std::string parmsfile;
    
    std::string _paramfile;
    std::vector<std::string> keys;
    std::vector<double> values;
protected:
    void ProcessLineEntries(int type, std::vector<std::string> lp);
    void ReadDelimitedFile(std::string file, int type);
public:

    ParamsInterface(){}

    void AddParam(std::string key, double value);
    void AddParams(std::vector<std::string> keys, std::vector<double> values);

    void LoadParams(std::string loc);
    void SaveParams(std::string loc);
    
    std::vector<double> LoadParams(std::vector<std::string> keys, std::vector<double> defaults = std::vector<double>());
    
    
};

