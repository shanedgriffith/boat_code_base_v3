//
//  FileParsing.cpp
//  SIFTFlow
//
//  Created by Shane Griffith on 1/11/16.
//  Copyright © 2016 shane. All rights reserved.
//

#include "FileParsing.hpp"
#include <algorithm>


using namespace std;

FILE * FileParsing::OpenFile(string filename, const char * op, bool required) {
    FILE * fs = fopen(filename.c_str(), op);
    if(!fs && required) {cout << "FileParsing: Error. Couldn't open " << filename << "." << endl; exit(-1);}
    return fs;
}

std::vector<std::string> FileParsing::ListFilesInDir(std::string dir_name, std::string type){
    std::vector<std::string> files;
    DIR * directory;
    struct dirent *ent;
    if ((directory = opendir(dir_name.c_str())) != NULL) {
        /* store all the files and directories within directory */
        while ((ent = readdir(directory)) != NULL) {
            if(strstr(ent->d_name, type.c_str())!=NULL) {
                files.push_back(ent->d_name);
            }
//            cout << "name: " << ent->d_name << endl;
        }
        closedir(directory);
    } else {
        /* could not open directory */
        perror ("");
        exit(-1);
    }
    std::sort(files.begin(), files.end());
    return files;
}

std::vector<std::string> FileParsing::ListDirsInDir(std::string dir_name) {
    std::vector<std::string> dirs;
    DIR * directory;
    struct dirent *ent;
    if ((directory = opendir(dir_name.c_str())) != NULL) {
        /* store all the files and directories within directory */
        while ((ent = readdir(directory)) != NULL) {
            if((ent->d_type == DT_UNKNOWN || ent->d_type == DT_DIR) && strstr(ent->d_name, ".")==NULL) {
                dirs.push_back(ent->d_name);
            }
//            cout << "name: " << ent->d_name << endl;
        }
        closedir(directory);
    } else {
        /* could not open directory */
        perror ("");
        exit(-1);
    }

    std::sort(dirs.begin(), dirs.end());
    return dirs;
}

bool FileParsing::DirectoryExists(std::string pzPath )
{
    if ( pzPath.length() == 0) return false;
    
    DIR *pDir;
    bool bExists = false;
    
    pDir = opendir (pzPath.c_str());
    
    if (pDir != NULL) {
        bExists = true;
        (void) closedir (pDir);
    }
    
    return bExists;
}

void FileParsing::MakeDir(std::string dir) {
    mkdir(dir.c_str(), (mode_t) (S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH));
}

std::vector<std::string> FileParsing::ParseLine(char * line) {
    return ParseLineAdv(line, ",");
}

std::vector<std::string> FileParsing::ParseLineAdv(char * line, std::string separator) {
    std::vector<std::string> parsedline;
    int idx = 0;
    const char* tok;
    for (tok = strtok(line, separator.c_str());
         tok && *tok;
         tok = strtok(NULL, string(separator+"\n").c_str()))
    {
        parsedline.push_back(tok);
        idx++;
    }
    return parsedline;
}

std::string FileParsing::formattime(double seconds) {
    int hours = seconds/3600;
    int minutes = (seconds-hours*3600)/60;
    seconds = seconds-hours*3600- minutes*60;
    char time[20];
    sprintf(time, "%02d:%02d:%2.4lf", hours, minutes, seconds);
    string ret = time;
    return ret;
    //return to_string(hours) + ":"+to_string(minutes)+":"+to_string(seconds);
}

void FileParsing::ReadDelimitedFile(std::string file, int type) {
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        std::vector<std::string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

bool FileParsing::Exists(string file) {
    bool filestatus=false;
    ifstream f(file.c_str());
    if(f){
        f.close();
        filestatus = true;
    } else{
        //cout << "FileParsing Error: couldn't open " << file << endl;
        filestatus=false;
    }
    return filestatus;
}


void FileParsing::AppendToFile(string file, std::string data) {
    FILE * fp = OpenFile(file, "a");//_logfile
    if(!fp) {std::cout <<"FileParsing. couldn't append to the file." << std::endl; exit(1);}
    fprintf(fp, "%s", data.c_str());
    fclose(fp);
}
