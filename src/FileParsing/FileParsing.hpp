//
//  FileParsing.hpp
//  SIFTFlow
//
//  Created by Shane Griffith on 1/11/16.
//  Copyright © 2016 shane. All rights reserved.
//

#ifndef FileParsing_hpp
#define FileParsing_hpp

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

    bool debug = false;
    bool verbose = false;
    
    static FILE * OpenFile(std::string filename, const char * op);
    static std::vector<std::string> ParseLine(char * line);
    static std::vector<std::string> ParseLineAdv(char * line, std::string separator);
    
    //TODO: how to implement this the way I intended.
    virtual void ReadDelimitedFile(std::string file, int type);
    virtual void ProcessLineEntries(int type, std::vector<std::string>& lp){}
public:
    std::string formattime(double seconds);
    static bool DirectoryExists(std::string pzPath );
    static bool Exists(std::string file);
    static void MakeDir(std::string dir);
    static std::vector<std::string> ListFilesInDir(std::string dir, std::string type);
    static std::vector<std::string> ListDirsInDir(std::string dir_name);
    static void AppendToFile(std::string file, std::string data);
};


#endif /* FileParsing_hpp */
