//
//  ParseVisibilityFile.cpp
//  SIFTFlow
//
//  Created by Shane Griffith on 6/19/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include <iostream>
#include <stdio.h>

#include "ParseVisibilityFile.h"


using namespace std;

void ParseVisibilityFile::ParseFile(string filepath) {
    FILE * fp = OpenFile(filepath,"r");
    char ds1[1024],ds2[1024];
    char line[1024]="";

    if (fgets(line,1023,fp)==NULL || sscanf(line,"%%%[^,],%s",ds1,ds2)!=2) {
        printf("ParseVisibilityFile: Can't parse first line: '%s' of file %s\n",line, filepath.c_str());
        exit(-1);
    }
    
    date1 = string(ds1);
    date2 = string(ds2);
    fgets(line,1023,fp);//do nothing with the second line
    while (!feof(fp) && fgets(line,1023,fp)!=NULL)
    {
        int pose1, seq1, file1, dir1;
        int pose2, seq2, file2, dir2;
        //        if (fgets(line,1023,fp)==NULL) {
        //            break;
        //        }
        if (sscanf(line," %d %d %d %d  %d %d %d %d ",
                   &pose1,&seq1,&dir1,&file1,
                   &pose2,&seq2,&dir2,&file2)!=8) {
            printf("ParseVisibilityFile: Can't parse line: '%s' of file %s\n",line, filepath.c_str());
            break;
        }
        //        char f1[1024],f2[1024];
        //        sprintf(f1,"%s/%s/%04d/%04d.jpg",query_loc.c_str(),ds1,dir1,file1);
        //        sprintf(f2,"%s/%s/%04d/%04d.jpg",query_loc.c_str(),ds2,dir2,file2);
        boat1.push_back(pose1);
        boat2.push_back(pose2);
        images1.push_back(seq1);
        images2.push_back(seq2);
    }
    fclose(fp);
}










