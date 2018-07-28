//
//  FileParsing.hpp
//  SIFTFlow
//
//  Created by Shane Griffith on 1/11/16.
//  Copyright © 2016 shane. All rights reserved.
//

#ifndef SRC_FILEPARSING_FILEPARSING_HPP_
#define SRC_FILEPARSING_FILEPARSING_HPP_

#include <stdio.h>
#include <string>
#include <unistd.h>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <stdlib.h>

class FileParsing {
private:
protected:
	const static int LINESIZE = 10000;

    
    static FILE * OpenFile(std::string filename, const char * op, bool required = true);
    static std::vector<std::string> ParseLine(char * line);
    static std::vector<std::string> ParseLineAdv(char * line, std::string separator);
    
    //TODO: how to implement this the way I intended.
    virtual void ReadDelimitedFile(std::string file, int type);
    virtual void ProcessLineEntries(int type, std::vector<std::string>& lp){}
public:
    bool debug = false;
    bool verbose = false;
    
    static std::string formattime(double seconds);
    static bool DirectoryExists(std::string pzPath );
    static bool Exists(std::string file);
    static void MakeDir(std::string dir);
    static std::vector<std::string> ListFilesInDir(std::string dir, std::string type);
    static std::vector<std::string> ListDirsInDir(std::string dir_name);
    static void AppendToFile(std::string file, std::string data);
};


#endif /* SRC_FILEPARSING_FILEPARSING_HPP_ */
