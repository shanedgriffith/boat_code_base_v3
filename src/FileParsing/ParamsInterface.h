//
//  ParamsInterface.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 6/10/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __BundleAdjustOneDataset__ParamsInterface__
#define __BundleAdjustOneDataset__ParamsInterface__

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
protected:
    void ProcessLineEntries(int type, std::vector<std::string> lp);
    void ReadDelimitedFile(std::string file, int type);
public:
    std::string _paramfile;
    std::vector<std::string> keys;
    std::vector<double> values;
    
    ParamsInterface(std::string query_loc) {
        _paramfile = query_loc + parmsfile;
        if(Exists(_paramfile)) {
            ReadDelimitedFile(_paramfile, 0);
        } else {
            std::cout << "Couldn't get the parameters at: " << query_loc << std::endl;
            exit(-1);
        }
    }
    
    void AddParam(std::string key, double value);
    
    void AddParams(std::vector<std::string> keys, std::vector<double> values);
    
    void SaveParams(std::string loc = "");
    
    std::vector<double> LoadParams(std::vector<std::string> keys, std::vector<double> defaults = std::vector<double>());
    
    
};

    
    



#endif /* defined(__BundleAdjustOneDataset__ParamsInterface__) */
