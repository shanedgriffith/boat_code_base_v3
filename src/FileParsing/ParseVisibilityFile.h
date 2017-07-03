//
//  ParseVisibilityFile.h
//  SIFTFlow
//
//  Created by Shane Griffith on 6/19/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SRC_FILEPARSING_PARSEVISIBILITYFILE_H_
#define SRC_FILEPARSING_PARSEVISIBILITYFILE_H_


#include <stdio.h>
#include <vector>
#include <string.h>

#include "FileParsing.hpp"

class ParseVisibilityFile: public FileParsing{
private:
    void ParseFile(std::string filepath);
public:
    std::string visibility_file;
    std::string date1, date2;
    std::vector<int> images1;
    std::vector<int> images2;
    std::vector<int> boat1; //to serve as thread ids
    std::vector<int> boat2;
    
    ParseVisibilityFile(std::string base, std::string date1, std::string date2){
        visibility_file = base + "poses_" + date1 + "_" + date2 + ".txt";
        ParseFile(visibility_file);
    }
};






#endif /* SRC_FILEPARSING_PARSEVISIBILITYFILE_H_ */
